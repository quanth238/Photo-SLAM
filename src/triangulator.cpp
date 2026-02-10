/**
 * Triangulator for CorrInit (pixel-space, OpenCV convention)
 *
 * This file is part of Photo-SLAM
 */

#include "include/triangulator.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <opencv2/calib3d.hpp>

namespace
{
cv::Mat makeProjectionMatrix(const cv::Mat& K, const Sophus::SE3f& Tcw)
{
    cv::Mat Rcw(3, 3, CV_32F);
    cv::Mat tcw(3, 1, CV_32F);
    Eigen::Matrix3f R = Tcw.rotationMatrix();
    Eigen::Vector3f t = Tcw.translation();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c)
            Rcw.at<float>(r, c) = R(r, c);
        tcw.at<float>(r, 0) = t(r);
    }
    cv::Mat Rt = cv::Mat::zeros(3, 4, CV_32F);
    Rcw.copyTo(Rt(cv::Rect(0, 0, 3, 3)));
    tcw.copyTo(Rt(cv::Rect(3, 0, 1, 3)));
    return K * Rt;
}

float computeParallaxDeg(
    const Eigen::Vector3f& Xw,
    const Sophus::SE3f& Tcw0,
    const Sophus::SE3f& Tcw1)
{
    Sophus::SE3f Twc0 = Tcw0.inverse();
    Sophus::SE3f Twc1 = Tcw1.inverse();
    Eigen::Vector3f C0 = Twc0.translation();
    Eigen::Vector3f C1 = Twc1.translation();
    Eigen::Vector3f v0 = Xw - C0;
    Eigen::Vector3f v1 = Xw - C1;
    float n0 = v0.norm();
    float n1 = v1.norm();
    if (n0 <= 1e-6f || n1 <= 1e-6f)
        return 0.0f;
    float cosang = v0.dot(v1) / (n0 * n1);
    cosang = std::max(-1.0f, std::min(1.0f, cosang));
    return std::acos(cosang) * 180.0f / static_cast<float>(M_PI);
}

void computeReprojection(
    const cv::Mat& K,
    const Sophus::SE3f& Tcw,
    const Eigen::Vector3f& Xw,
    float& u,
    float& v)
{
    Eigen::Vector3f Xc = Tcw.rotationMatrix() * Xw + Tcw.translation();
    float z = Xc.z();
    float x = Xc.x();
    float y = Xc.y();
    float fx = K.at<float>(0, 0);
    float fy = K.at<float>(1, 1);
    float cx = K.at<float>(0, 2);
    float cy = K.at<float>(1, 2);
    u = fx * (x / z) + cx;
    v = fy * (y / z) + cy;
}

float medianFromVector(std::vector<float>& v)
{
    if (v.empty())
        return 0.0f;
    std::size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    return v[mid];
}

float p90FromVector(std::vector<float>& v)
{
    if (v.empty())
        return 0.0f;
    std::size_t idx = static_cast<std::size_t>(std::floor(0.9 * (v.size() - 1)));
    std::nth_element(v.begin(), v.begin() + idx, v.end());
    return v[idx];
}
} // namespace

std::vector<TriangulatedPoint> Triangulator::triangulate(
    const cv::Mat& K0,
    const Sophus::SE3f& Tcw0,
    const cv::Mat& K1,
    const Sophus::SE3f& Tcw1,
    const std::vector<Match2D>& matches,
    float reproj_err_px,
    float min_parallax_deg,
    TriangulationStats* stats)
{
    std::vector<TriangulatedPoint> results;
    results.reserve(matches.size());

    if (stats)
        stats->num_input = static_cast<int>(matches.size());

    if (matches.empty())
        return results;

    cv::Mat P0 = makeProjectionMatrix(K0, Tcw0);
    cv::Mat P1 = makeProjectionMatrix(K1, Tcw1);

    std::vector<cv::Point2f> pts0;
    std::vector<cv::Point2f> pts1;
    pts0.reserve(matches.size());
    pts1.reserve(matches.size());
    for (const auto& m : matches) {
        pts0.emplace_back(m.u0, m.v0);
        pts1.emplace_back(m.u1, m.v1);
    }

    cv::Mat X4;
    cv::triangulatePoints(P0, P1, pts0, pts1, X4);

    if (X4.cols != static_cast<int>(matches.size()))
        return results;

    std::vector<float> reproj_errors;
    std::vector<float> parallax_vals;
    reproj_errors.reserve(matches.size());
    parallax_vals.reserve(matches.size());

    for (int i = 0; i < X4.cols; ++i) {
        TriangulatedPoint tp;
        tp.match_idx = static_cast<std::size_t>(i);

        float w = X4.at<float>(3, i);
        if (std::abs(w) < 1e-8f) {
            tp.valid = false;
            results.push_back(tp);
            continue;
        }

        Eigen::Vector3f Xw;
        Xw.x() = X4.at<float>(0, i) / w;
        Xw.y() = X4.at<float>(1, i) / w;
        Xw.z() = X4.at<float>(2, i) / w;

        Eigen::Vector3f Xc0 = Tcw0.rotationMatrix() * Xw + Tcw0.translation();
        Eigen::Vector3f Xc1 = Tcw1.rotationMatrix() * Xw + Tcw1.translation();
        if (Xc0.z() <= 0.0f || Xc1.z() <= 0.0f) {
            tp.valid = false;
            results.push_back(tp);
            continue;
        }

        if (stats)
            stats->num_cheirality_pass++;

        float u0, v0, u1, v1;
        computeReprojection(K0, Tcw0, Xw, u0, v0);
        computeReprojection(K1, Tcw1, Xw, u1, v1);
        float du0 = u0 - matches[i].u0;
        float dv0 = v0 - matches[i].v0;
        float du1 = u1 - matches[i].u1;
        float dv1 = v1 - matches[i].v1;
        float e0 = std::sqrt(du0 * du0 + dv0 * dv0);
        float e1 = std::sqrt(du1 * du1 + dv1 * dv1);
        float e = std::max(e0, e1);

        if (e > reproj_err_px) {
            tp.valid = false;
            results.push_back(tp);
            continue;
        }

        if (stats)
            stats->num_reproj_pass++;

        float parallax = computeParallaxDeg(Xw, Tcw0, Tcw1);
        if (parallax < min_parallax_deg) {
            tp.valid = false;
            results.push_back(tp);
            continue;
        }

        if (stats)
            stats->num_parallax_pass++;

        tp.valid = true;
        tp.Xw = Xw;
        tp.reproj_err = e;
        tp.parallax_deg = parallax;
        results.push_back(tp);

        reproj_errors.push_back(e);
        parallax_vals.push_back(parallax);
    }

    if (stats) {
        stats->num_triangulated = static_cast<int>(results.size());
        stats->median_reproj = medianFromVector(reproj_errors);
        stats->p90_reproj = p90FromVector(reproj_errors);
        stats->median_parallax = medianFromVector(parallax_vals);
        stats->p90_parallax = p90FromVector(parallax_vals);
    }

    return results;
}
