/**
 * Render depth maps from trained Gaussian splatting model
 * Uses the actual Gaussian rasterizer (not point projection)
 * 
 * Usage: render_depth <result_dir> <camera_config> [--max_frames N]
 */

#include <ctime>
#include <sstream>
#include <thread>
#include <filesystem>
#include <memory>
#include <fstream>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <torch/torch.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "include/gaussian_scene.h"
#include "include/gaussian_model.h"
#include "include/gaussian_renderer.h"
#include "include/gaussian_keyframe.h"
#include "include/gaussian_parameters.h"
#include "include/camera.h"

namespace fs = std::filesystem;

struct RenderConfig {
    fs::path result_dir;
    fs::path camera_config;
    fs::path output_dir;
    int max_frames = -1;  // -1 means all frames
    int width = 1200;
    int height = 680;
    float z_near = 0.01f;
    float z_far = 100.0f;
};

// Parse camera poses from TUM format file
std::vector<std::pair<double, Sophus::SE3f>> loadPosesTUM(const fs::path& pose_file) {
    std::vector<std::pair<double, Sophus::SE3f>> poses;
    std::ifstream file(pose_file);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open pose file: " + pose_file.string());
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        double timestamp;
        float tx, ty, tz, qx, qy, qz, qw;
        
        if (!(iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
            continue;
        }
        
        Eigen::Vector3f translation(tx, ty, tz);
        Eigen::Quaternionf quat(qw, qx, qy, qz);
        quat.normalize();
        
        // TUM format is Twc (world to camera), need to invert for Tcw
        Sophus::SE3f Twc(quat, translation);
        Sophus::SE3f Tcw = Twc.inverse();
        
        poses.emplace_back(timestamp, Tcw);
    }
    
    std::cout << "Loaded " << poses.size() << " poses from " << pose_file << std::endl;
    return poses;
}

// Load camera intrinsics from YAML config
Camera loadCameraFromConfig(const fs::path& config_path, int width, int height) {
    cv::FileStorage fs(config_path.string(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Cannot open camera config: " + config_path.string());
    }
    
    Camera camera;
    camera.width_ = width;
    camera.height_ = height;
    
    // Try to read camera intrinsics
    cv::Mat K;
    fs["Camera.K"] >> K;
    if (K.empty()) {
        // Try alternative format (Camera1 for stereo config)
        fs["Camera1.K"] >> K;
    }
    
    double fx, fy, cx, cy;
    if (!K.empty()) {
        fx = K.at<double>(0, 0);
        fy = K.at<double>(1, 1);
        cx = K.at<double>(0, 2);
        cy = K.at<double>(1, 2);
    } else {
        // Try individual parameters
        fs["Camera.fx"] >> fx;
        fs["Camera.fy"] >> fy;
        fs["Camera.cx"] >> cx;
        fs["Camera.cy"] >> cy;
        if (fx <= 0) {
            // Default for Replica dataset
            fx = 600.0;
            fy = 600.0;
            cx = 599.5;
            cy = 339.5;
            std::cout << "Using default Replica camera intrinsics" << std::endl;
        }
    }
    
    // Camera uses params_ vector: [fx, fy, cx, cy]
    camera.params_ = {fx, fy, cx, cy};
    camera.model_id_ = Camera::PINHOLE;
    
    std::cout << "Camera intrinsics: fx=" << fx << ", fy=" << fy 
              << ", cx=" << cx << ", cy=" << cy << std::endl;
    
    return camera;
}

void renderDepthFromGaussians(const RenderConfig& config) {
    // Check inputs
    fs::path ply_path = config.result_dir / "point_cloud.ply";
    fs::path pose_file = config.result_dir / "CameraTrajectory_TUM.txt";
    
    if (!fs::exists(ply_path)) {
        throw std::runtime_error("Point cloud not found: " + ply_path.string());
    }
    if (!fs::exists(pose_file)) {
        throw std::runtime_error("Pose file not found: " + pose_file.string());
    }
    
    // Create output directory
    fs::path output_depth_dir = config.result_dir / "rendered_depth";
    fs::create_directories(output_depth_dir);
    
    std::cout << "=== Gaussian Splatting Depth Renderer ===" << std::endl;
    std::cout << "Result dir: " << config.result_dir << std::endl;
    std::cout << "Output dir: " << output_depth_dir << std::endl;
    std::cout << "Resolution: " << config.width << "x" << config.height << std::endl;
    
    // Load poses
    auto poses = loadPosesTUM(pose_file);
    if (poses.empty()) {
        throw std::runtime_error("No poses loaded!");
    }
    
    // Limit frames if specified
    int num_frames = poses.size();
    if (config.max_frames > 0 && config.max_frames < num_frames) {
        num_frames = config.max_frames;
    }
    std::cout << "Rendering " << num_frames << " frames" << std::endl;
    
    // Load camera
    Camera camera = loadCameraFromConfig(config.camera_config, config.width, config.height);
    
    // Initialize Gaussian model (SH degree 3 to match trained model)
    GaussianModelParams model_params;
    model_params.sh_degree_ = 3;  // Match trained model SH degree
    auto gaussians = std::make_shared<GaussianModel>(model_params);
    
    // Load trained Gaussians from PLY
    std::cout << "Loading Gaussians from " << ply_path << std::endl;
    gaussians->loadPly(ply_path.string());
    std::cout << "Loaded " << gaussians->getXYZ().size(0) << " Gaussians" << std::endl;
    
    // Setup rendering parameters
    GaussianPipelineParams pipe_params;
    pipe_params.convert_SHs_ = false;
    pipe_params.compute_cov3D_ = false;
    
    torch::Tensor background = torch::zeros({3}, torch::device(torch::kCUDA));
    torch::Tensor override_color = torch::Tensor();
    
    // Render each pose
    for (int i = 0; i < num_frames; ++i) {
        const auto& [timestamp, Tcw] = poses[i];
        
        // Create keyframe for rendering
        auto pkf = std::make_shared<GaussianKeyframe>();
        pkf->zfar_ = config.z_far;
        pkf->znear_ = config.z_near;
        pkf->image_width_ = config.width;
        pkf->image_height_ = config.height;
        
        // Set pose (Tcw format expected)
        pkf->setPose(
            Tcw.unit_quaternion().cast<double>(),
            Tcw.translation().cast<double>());
        
        // Set camera parameters
        pkf->setCameraParams(camera);
        
        // Compute transform tensors for rendering
        pkf->computeTransformTensors();
        
        // Render using Gaussian splatting
        auto render_pkg = GaussianRenderer::render(
            pkf,
            config.height,
            config.width,
            gaussians,
            pipe_params,
            background,
            override_color
        );
        
        // Get rendered depth (element 4 of render_pkg)
        torch::Tensor rendered_depth = std::get<4>(render_pkg);  // [1, H, W]
        
        // Convert to CPU and save
        rendered_depth = rendered_depth.squeeze(0).cpu();  // [H, W]
        
        // Convert to 16-bit PNG (depth scale 6553.5 to match Replica GT format)
        cv::Mat depth_mat(config.height, config.width, CV_32FC1, 
                          rendered_depth.data_ptr<float>());
        cv::Mat depth_16u;
        depth_mat.convertTo(depth_16u, CV_16UC1, 6553.5);  // Match Replica GT depth factor
        
        // Save with frame index
        std::ostringstream filename;
        filename << "frame" << std::setw(6) << std::setfill('0') << i << "_depth.png";
        fs::path output_path = output_depth_dir / filename.str();
        cv::imwrite(output_path.string(), depth_16u);
        
        // Progress
        if ((i + 1) % 100 == 0 || i == num_frames - 1) {
            std::cout << "Rendered " << (i + 1) << "/" << num_frames << " frames" << std::endl;
        }
    }
    
    std::cout << "Done! Depth maps saved to " << output_depth_dir << std::endl;
}

void printUsage(const char* progname) {
    std::cout << "Usage: " << progname << " <result_dir> <camera_config> [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --max_frames N    Maximum frames to render (-1 for all, default: -1)" << std::endl;
    std::cout << "  --width W         Image width (default: 1200)" << std::endl;
    std::cout << "  --height H        Image height (default: 680)" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }
    
    RenderConfig config;
    config.result_dir = argv[1];
    config.camera_config = argv[2];
    
    // Parse optional arguments
    for (int i = 3; i < argc; i += 2) {
        std::string arg = argv[i];
        if (arg == "--max_frames" && i + 1 < argc) {
            config.max_frames = std::stoi(argv[i + 1]);
        } else if (arg == "--width" && i + 1 < argc) {
            config.width = std::stoi(argv[i + 1]);
        } else if (arg == "--height" && i + 1 < argc) {
            config.height = std::stoi(argv[i + 1]);
        }
    }
    
    try {
        renderDepthFromGaussians(config);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
