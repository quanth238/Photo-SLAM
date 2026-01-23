/**
 * Standalone test for DA3Infer (Depth Anything V3 TensorRT)
 * 
 * Usage:
 *   ./test_da3 <engine_path> <image_path> [output_path]
 * 
 * Example:
 *   ./test_da3 ../models/DA3METRIC-LARGE.engine /path/to/image.jpg depth_output.png
 */

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "da3_infer/da3_infer.hpp"

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <engine_path> <image_path> [output_path]" << std::endl;
        std::cerr << "Example: " << argv[0] << " ../models/DA3METRIC-LARGE.engine test.jpg depth.png" << std::endl;
        return 1;
    }

    std::string engine_path = argv[1];
    std::string image_path = argv[2];
    std::string output_path = argc > 3 ? argv[3] : "depth_output.png";

    // Load test image
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
        std::cerr << "Failed to load image: " << image_path << std::endl;
        return 1;
    }
    std::cout << "Loaded image: " << image.cols << "x" << image.rows << std::endl;

    // Initialize DA3Infer
    std::cout << "Initializing DA3Infer with engine: " << engine_path << std::endl;
    
    da3::DA3Config config;
    config.engine_path = engine_path;
    config.precision = "fp16";
    config.sky_threshold = 0.3f;
    config.sky_depth_cap = 200.0f;

    da3::DA3Infer da3(config);
    
    if (!da3.isInitialized()) {
        std::cerr << "Failed to initialize DA3Infer!" << std::endl;
        return 1;
    }

    auto [h, w] = da3.getModelInputSize();
    std::cout << "Model input size: " << h << "x" << w << std::endl;

    // Set up camera intrinsics (example values, adjust for your camera)
    da3::CameraIntrinsics intrinsics{
        517.0f,  // fx
        517.0f,  // fy
        static_cast<float>(image.cols) / 2.0f,  // cx
        static_cast<float>(image.rows) / 2.0f,  // cy
        image.cols,  // width
        image.rows   // height
    };

    // Warm up
    std::cout << "Warming up..." << std::endl;
    cv::Mat depth_warmup = da3.infer(image, intrinsics);

    // Benchmark
    const int num_runs = 10;
    std::cout << "Running " << num_runs << " inferences..." << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    cv::Mat depth;
    for (int i = 0; i < num_runs; ++i) {
        depth = da3.infer(image, intrinsics);
    }
    auto end = std::chrono::high_resolution_clock::now();
    
    double avg_ms = std::chrono::duration<double, std::milli>(end - start).count() / num_runs;
    std::cout << "Average inference time: " << avg_ms << " ms (" << 1000.0/avg_ms << " FPS)" << std::endl;

    // Analyze depth output
    if (depth.empty()) {
        std::cerr << "Inference failed - empty depth map!" << std::endl;
        return 1;
    }

    double min_val, max_val;
    cv::minMaxLoc(depth, &min_val, &max_val);
    cv::Scalar mean_val = cv::mean(depth);
    
    std::cout << "\n=== Depth Statistics ===" << std::endl;
    std::cout << "Output size: " << depth.cols << "x" << depth.rows << std::endl;
    std::cout << "Depth range: " << min_val << " - " << max_val << " meters" << std::endl;
    std::cout << "Mean depth: " << mean_val[0] << " meters" << std::endl;

    // Visualize and save
    cv::Mat depth_vis;
    depth.convertTo(depth_vis, CV_8UC1, 255.0 / max_val);
    cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_MAGMA);
    
    cv::imwrite(output_path, depth_vis);
    std::cout << "\nSaved depth visualization to: " << output_path << std::endl;

    std::cout << "\n=== DA3 Test PASSED ===" << std::endl;
    return 0;
}
