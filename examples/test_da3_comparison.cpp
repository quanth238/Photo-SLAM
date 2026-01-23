/**
 * DA3 vs Ground Truth Depth Comparison Test
 * 
 * Compares DA3 TensorRT depth estimation with ground truth sensor depth
 * from TUM RGB-D dataset. Computes scale alignment and error metrics.
 * 
 * Usage:
 *   ./test_da3_comparison <engine_path> <rgb_image> <gt_depth_image> <depth_factor> [output_dir]
 * 
 * TUM depth_factor = 5000 (depth value / 5000 = meters)
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "da3_infer/da3_infer.hpp"

// TUM freiburg1 camera intrinsics
const float TUM_FX = 517.3f;
const float TUM_FY = 516.5f;
const float TUM_CX = 318.6f;
const float TUM_CY = 255.3f;

struct DepthMetrics {
    double abs_rel;     // Absolute relative error
    double sq_rel;      // Squared relative error
    double rmse;        // Root mean squared error
    double rmse_log;    // RMSE in log space
    double a1, a2, a3;  // Threshold accuracy (delta < 1.25, 1.25^2, 1.25^3)
    double scale;       // Median scale factor
    int valid_pixels;   // Number of valid pixels used
};

DepthMetrics computeMetrics(const cv::Mat& pred, const cv::Mat& gt, 
                            float min_depth = 0.1f, float max_depth = 10.0f) {
    DepthMetrics m = {0};
    
    std::vector<double> pred_vals, gt_vals;
    
    // Collect valid pixels
    for (int y = 0; y < pred.rows; ++y) {
        for (int x = 0; x < pred.cols; ++x) {
            float p = pred.at<float>(y, x);
            float g = gt.at<float>(y, x);
            
            if (g > min_depth && g < max_depth && p > 0.01f && std::isfinite(p) && std::isfinite(g)) {
                pred_vals.push_back(p);
                gt_vals.push_back(g);
            }
        }
    }
    
    m.valid_pixels = pred_vals.size();
    if (m.valid_pixels < 100) {
        std::cerr << "Warning: Only " << m.valid_pixels << " valid pixels!" << std::endl;
        return m;
    }
    
    // Compute median scale (gt / pred)
    std::vector<double> ratios;
    for (size_t i = 0; i < pred_vals.size(); ++i) {
        ratios.push_back(gt_vals[i] / pred_vals[i]);
    }
    std::sort(ratios.begin(), ratios.end());
    m.scale = ratios[ratios.size() / 2];
    
    // Compute metrics after scale alignment
    double sum_abs_rel = 0, sum_sq_rel = 0, sum_sq = 0, sum_log_sq = 0;
    int count_a1 = 0, count_a2 = 0, count_a3 = 0;
    
    for (size_t i = 0; i < pred_vals.size(); ++i) {
        double p_scaled = pred_vals[i] * m.scale;
        double g = gt_vals[i];
        
        double diff = std::abs(p_scaled - g);
        sum_abs_rel += diff / g;
        sum_sq_rel += (diff * diff) / g;
        sum_sq += diff * diff;
        
        double log_diff = std::log(p_scaled) - std::log(g);
        sum_log_sq += log_diff * log_diff;
        
        double ratio = std::max(p_scaled / g, g / p_scaled);
        if (ratio < 1.25) count_a1++;
        if (ratio < 1.25 * 1.25) count_a2++;
        if (ratio < 1.25 * 1.25 * 1.25) count_a3++;
    }
    
    int n = pred_vals.size();
    m.abs_rel = sum_abs_rel / n;
    m.sq_rel = sum_sq_rel / n;
    m.rmse = std::sqrt(sum_sq / n);
    m.rmse_log = std::sqrt(sum_log_sq / n);
    m.a1 = (double)count_a1 / n;
    m.a2 = (double)count_a2 / n;
    m.a3 = (double)count_a3 / n;
    
    return m;
}

cv::Mat visualizeComparison(const cv::Mat& rgb, const cv::Mat& pred, 
                            const cv::Mat& gt, double scale) {
    // Create visualization grid
    int h = rgb.rows, w = rgb.cols;
    cv::Mat vis(h * 2, w * 2, CV_8UC3);
    
    // Top-left: RGB
    cv::Mat rgb_region = vis(cv::Rect(0, 0, w, h));
    cv::resize(rgb, rgb_region, cv::Size(w, h));
    
    // Top-right: GT depth
    cv::Mat gt_norm, gt_color;
    double gt_min, gt_max;
    cv::minMaxLoc(gt, &gt_min, &gt_max);
    gt.convertTo(gt_norm, CV_8UC1, 255.0 / gt_max);
    cv::applyColorMap(gt_norm, gt_color, cv::COLORMAP_MAGMA);
    gt_color.copyTo(vis(cv::Rect(w, 0, w, h)));
    
    // Bottom-left: DA3 prediction (scaled)
    cv::Mat pred_scaled = pred * scale;
    cv::Mat pred_norm, pred_color;
    pred_scaled.convertTo(pred_norm, CV_8UC1, 255.0 / gt_max);
    cv::applyColorMap(pred_norm, pred_color, cv::COLORMAP_MAGMA);
    pred_color.copyTo(vis(cv::Rect(0, h, w, h)));
    
    // Bottom-right: Error map
    cv::Mat error = cv::abs(pred_scaled - gt);
    cv::Mat error_norm, error_color;
    error.convertTo(error_norm, CV_8UC1, 255.0 / 0.5); // 0.5m max error for vis
    cv::applyColorMap(error_norm, error_color, cv::COLORMAP_JET);
    error_color.copyTo(vis(cv::Rect(w, h, w, h)));
    
    // Add labels
    cv::putText(vis, "RGB", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 2);
    cv::putText(vis, "GT Depth (Sensor)", cv::Point(w+10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 2);
    cv::putText(vis, "DA3 Prediction (Scaled)", cv::Point(10, h+30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 2);
    cv::putText(vis, "Error Map", cv::Point(w+10, h+30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 2);
    
    return vis;
}

int main(int argc, char* argv[]) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " <engine_path> <rgb_image> <gt_depth_image> <depth_factor> [output_dir]" << std::endl;
        std::cerr << "TUM depth_factor = 5000" << std::endl;
        return 1;
    }

    std::string engine_path = argv[1];
    std::string rgb_path = argv[2];
    std::string gt_depth_path = argv[3];
    float depth_factor = std::stof(argv[4]);
    std::string output_dir = argc > 5 ? argv[5] : ".";

    // Load images
    cv::Mat rgb = cv::imread(rgb_path);
    cv::Mat gt_depth_raw = cv::imread(gt_depth_path, cv::IMREAD_UNCHANGED);
    
    if (rgb.empty()) {
        std::cerr << "Failed to load RGB: " << rgb_path << std::endl;
        return 1;
    }
    if (gt_depth_raw.empty()) {
        std::cerr << "Failed to load GT depth: " << gt_depth_path << std::endl;
        return 1;
    }
    
    // Convert GT depth to meters
    cv::Mat gt_depth;
    gt_depth_raw.convertTo(gt_depth, CV_32FC1, 1.0 / depth_factor);
    
    std::cout << "RGB size: " << rgb.cols << "x" << rgb.rows << std::endl;
    std::cout << "GT depth size: " << gt_depth.cols << "x" << gt_depth.rows << std::endl;
    
    // Initialize DA3
    std::cout << "\nInitializing DA3Infer..." << std::endl;
    da3::DA3Config config;
    config.engine_path = engine_path;
    config.precision = "fp16";
    config.sky_threshold = 0.3f;
    config.sky_depth_cap = 200.0f;

    da3::DA3Infer da3(config);
    if (!da3.isInitialized()) {
        std::cerr << "Failed to initialize DA3!" << std::endl;
        return 1;
    }
    
    // Run inference
    da3::CameraIntrinsics intrinsics{TUM_FX, TUM_FY, TUM_CX, TUM_CY, rgb.cols, rgb.rows};
    
    std::cout << "Running DA3 inference..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    cv::Mat pred_depth = da3.infer(rgb, intrinsics);
    auto end = std::chrono::high_resolution_clock::now();
    double inference_ms = std::chrono::duration<double, std::milli>(end - start).count();
    
    std::cout << "Inference time: " << inference_ms << " ms" << std::endl;
    
    // Compute metrics
    std::cout << "\n=== Depth Comparison ===" << std::endl;
    
    double pred_min, pred_max, gt_min, gt_max;
    cv::minMaxLoc(pred_depth, &pred_min, &pred_max);
    cv::minMaxLoc(gt_depth, &gt_min, &gt_max);
    
    std::cout << "DA3 depth range:  " << pred_min << " - " << pred_max << " m" << std::endl;
    std::cout << "GT depth range:   " << gt_min << " - " << gt_max << " m" << std::endl;
    
    DepthMetrics metrics = computeMetrics(pred_depth, gt_depth);
    
    std::cout << "\n=== Metrics (after scale alignment) ===" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Scale factor (median): " << metrics.scale << std::endl;
    std::cout << "Valid pixels: " << metrics.valid_pixels << " / " << (pred_depth.rows * pred_depth.cols) << std::endl;
    std::cout << "\n";
    std::cout << "Abs Rel:  " << metrics.abs_rel << std::endl;
    std::cout << "Sq Rel:   " << metrics.sq_rel << std::endl;
    std::cout << "RMSE:     " << metrics.rmse << " m" << std::endl;
    std::cout << "RMSE log: " << metrics.rmse_log << std::endl;
    std::cout << "\n";
    std::cout << "δ < 1.25:    " << (metrics.a1 * 100) << " %" << std::endl;
    std::cout << "δ < 1.25²:   " << (metrics.a2 * 100) << " %" << std::endl;
    std::cout << "δ < 1.25³:   " << (metrics.a3 * 100) << " %" << std::endl;
    
    // Create visualization
    cv::Mat vis = visualizeComparison(rgb, pred_depth, gt_depth, metrics.scale);
    std::string vis_path = output_dir + "/da3_comparison.png";
    cv::imwrite(vis_path, vis);
    std::cout << "\nSaved comparison to: " << vis_path << std::endl;
    
    // Save individual outputs
    cv::Mat pred_scaled_vis, gt_vis;
    cv::Mat pred_scaled = pred_depth * metrics.scale;
    pred_scaled.convertTo(pred_scaled_vis, CV_8UC1, 255.0 / gt_max);
    gt_depth.convertTo(gt_vis, CV_8UC1, 255.0 / gt_max);
    cv::applyColorMap(pred_scaled_vis, pred_scaled_vis, cv::COLORMAP_MAGMA);
    cv::applyColorMap(gt_vis, gt_vis, cv::COLORMAP_MAGMA);
    
    cv::imwrite(output_dir + "/da3_pred_scaled.png", pred_scaled_vis);
    cv::imwrite(output_dir + "/gt_depth.png", gt_vis);
    
    std::cout << "\n=== Test Complete ===" << std::endl;
    return 0;
}
