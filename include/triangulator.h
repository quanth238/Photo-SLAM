/**
 * Triangulator for CorrInit (pixel-space, OpenCV convention)
 *
 * This file is part of Photo-SLAM
 */

#pragma once

#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

struct Match2D
{
    float u0;
    float v0;
    float u1;
    float v1;
    float conf;
};

struct TriangulatedPoint
{
    Eigen::Vector3f Xw;
    float reproj_err = 0.0f;
    float parallax_deg = 0.0f;
    bool valid = false;
    std::size_t match_idx = 0;
};

struct TriangulationStats
{
    int num_input = 0;
    int num_triangulated = 0;
    int num_cheirality_pass = 0;
    int num_reproj_pass = 0;
    int num_parallax_pass = 0;
    float median_reproj = 0.0f;
    float p90_reproj = 0.0f;
    float median_parallax = 0.0f;
    float p90_parallax = 0.0f;
};

class Triangulator
{
public:
    static std::vector<TriangulatedPoint> triangulate(
        const cv::Mat& K0,
        const Sophus::SE3f& Tcw0,
        const cv::Mat& K1,
        const Sophus::SE3f& Tcw1,
        const std::vector<Match2D>& matches,
        float reproj_err_px,
        float min_parallax_deg,
        TriangulationStats* stats = nullptr);
};
