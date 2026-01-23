/**
 * DA3 Depth to Point Cloud Test
 * 
 * Export DA3 depth as PLY point cloud for visual inspection in MeshLab
 * 
 * Usage:
 *   ./test_da3_pointcloud <engine> <image> <output.ply>
 */

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "da3_infer/da3_infer.hpp"

// TUM freiburg1 camera intrinsics
const float FX = 517.3f, FY = 516.5f, CX = 318.6f, CY = 255.3f;

void savePLY(const std::string& path, 
             const std::vector<Eigen::Vector3f>& points,
             const std::vector<Eigen::Vector3i>& colors) {
    std::ofstream ofs(path);
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex " << points.size() << "\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    ofs << "property uchar red\n";
    ofs << "property uchar green\n";
    ofs << "property uchar blue\n";
    ofs << "end_header\n";
    
    for (size_t i = 0; i < points.size(); ++i) {
        ofs << points[i].x() << " " << points[i].y() << " " << points[i].z() << " "
            << colors[i].x() << " " << colors[i].y() << " " << colors[i].z() << "\n";
    }
    ofs.close();
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <engine> <image> <output.ply> [edge_threshold]" << std::endl;
        std::cerr << "  edge_threshold: 0.0 = no filtering, 0.15 = 15% gradient threshold (default)" << std::endl;
        return 1;
    }
    
    std::string engine_path = argv[1];
    std::string image_path = argv[2];
    std::string output_path = argv[3];
    float edge_threshold = (argc > 4) ? std::atof(argv[4]) : 0.15f;
    
    // Load image
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
        std::cerr << "Failed to load: " << image_path << std::endl;
        return 1;
    }
    int h = image.rows, w = image.cols;
    std::cout << "Image: " << w << "x" << h << std::endl;
    std::cout << "Edge threshold: " << edge_threshold << (edge_threshold > 0 ? " (filtering ON)" : " (filtering OFF)") << std::endl;
    
    // Init DA3
    da3::DA3Config config;
    config.engine_path = engine_path;
    config.precision = "fp16";
    da3::DA3Infer da3(config);
    
    if (!da3.isInitialized()) {
        std::cerr << "DA3 init failed" << std::endl;
        return 1;
    }
    
    // Infer depth
    da3::CameraIntrinsics intr{FX, FY, CX, CY, w, h};
    cv::Mat depth = da3.infer(image, intr);
    
    std::cout << "Depth inferred: " << depth.cols << "x" << depth.rows << std::endl;
    
    // Create point cloud
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3i> colors;
    
    int stride = 1;  // Full resolution for accurate edge inspection
    float min_depth = 0.1f, max_depth = 10.0f;
    int edge_filtered = 0;
    
    for (int v = 0; v < h; v += stride) {
        for (int u = 0; u < w; u += stride) {
            float d = depth.at<float>(v, u);
            if (!std::isfinite(d) || d < min_depth || d > max_depth) continue;
            
            // EDGE FILTERING: Skip pixels at depth discontinuities
            if (edge_threshold > 0.0f && v > 0 && v < h-1 && u > 0 && u < w-1) {
                float d_up    = depth.at<float>(v-1, u);
                float d_down  = depth.at<float>(v+1, u);
                float d_left  = depth.at<float>(v, u-1);
                float d_right = depth.at<float>(v, u+1);
                
                if (!std::isfinite(d_up) || !std::isfinite(d_down) ||
                    !std::isfinite(d_left) || !std::isfinite(d_right)) continue;
                
                float max_gradient = std::max({
                    std::abs(d - d_up), std::abs(d - d_down),
                    std::abs(d - d_left), std::abs(d - d_right)
                });
                
                if (max_gradient > edge_threshold * d) {
                    edge_filtered++;
                    continue;
                }
            }
            
            // Backproject
            float x = (u - CX) * d / FX;
            float y = (v - CY) * d / FY;
            points.emplace_back(x, y, d);
            
            // Color (BGR -> RGB)
            cv::Vec3b bgr = image.at<cv::Vec3b>(v, u);
            colors.emplace_back(bgr[2], bgr[1], bgr[0]);
        }
    }
    
    std::cout << "Points: " << points.size() << " (edge-filtered: " << edge_filtered << ")" << std::endl;
    
    // Save PLY
    savePLY(output_path, points, colors);
    std::cout << "Saved: " << output_path << std::endl;
    
    // Also save depth visualization
    double dmin, dmax;
    cv::minMaxLoc(depth, &dmin, &dmax);
    cv::Mat depth_vis;
    depth.convertTo(depth_vis, CV_8UC1, 255.0/dmax);
    cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_MAGMA);
    cv::imwrite(output_path + "_depth.png", depth_vis);
    std::cout << "Saved depth vis: " << output_path << "_depth.png" << std::endl;
    
    return 0;
}
