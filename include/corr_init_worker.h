/**
 * CorrInit worker (EfficientLoFTR sidecar)
 *
 * This file is part of Photo-SLAM
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#ifdef WITH_CORRINIT_ZMQ
namespace zmq
{
class context_t;
class socket_t;
} // namespace zmq
#endif

struct KeyframeSnapshot
{
    unsigned long kfid = 0;
    unsigned long camera_id = 0;
    Sophus::SE3f Tcw;
    std::vector<float> intr; // fx, fy, cx, cy
    cv::Mat image_undist;     // CV_32FC3, RGB, undistorted
};

struct CorrInitTask
{
    KeyframeSnapshot ref;
    std::vector<KeyframeSnapshot> neighbors;
};

struct CorrInitSeedPacket
{
    unsigned long ref_kfid = 0;
    unsigned long neighbor_kfid = 0;
    std::vector<Eigen::Vector3f> points;
    std::vector<cv::Vec3f> colors;
};

class CorrInitWorker
{
public:
    CorrInitWorker(
        const std::string& zmq_endpoint,
        const std::string& log_path,
        const std::string& debug_dir,
        int debug_every_n,
        int debug_max_tasks,
        int queue_capacity,
        int num_seeds,
        int num_oversample,
        float reproj_err_px,
        float min_parallax_deg);

    ~CorrInitWorker();

    void start();
    void stop();

    void enqueueTask(CorrInitTask&& task);
    bool tryPopSeedPacket(CorrInitSeedPacket& out_packet);
    void logIntegration(const CorrInitSeedPacket& packet, int iter);

private:
    void threadLoop();
    bool processTask(const CorrInitTask& task, CorrInitSeedPacket& out_packet, std::size_t queue_size);
    void logCsvLine(const std::string& line);
    void logTaskStats(
        const std::string& event,
        unsigned long ref_kfid,
        unsigned long neighbor_kfid,
        int matches_raw,
        int matches_in_bounds,
        int oversample,
        int num_triangulated,
        int num_cheirality_pass,
        int num_reproj_pass,
        int num_parallax_pass,
        int seeds_out,
        float median_reproj,
        float p90_reproj,
        float median_parallax,
        float p90_parallax,
        double task_ms,
        std::size_t queue_size,
        int iter,
        const std::string& note);
    void logSimpleEvent(const std::string& event, const std::string& note);

#ifdef WITH_CORRINIT_ZMQ
    bool ensureZmqSocket();
    void resetZmqSocket();
#endif

private:
    std::string zmq_endpoint_;
    std::string log_path_;
    std::string debug_dir_;
    int debug_every_n_ = 0;
    int debug_max_tasks_ = 0;
    std::atomic<int> debug_saved_{0};
    std::size_t debug_task_index_ = 0;
    int queue_capacity_ = 8;
    int num_seeds_ = 512;
    int num_oversample_ = 2048;
    float reproj_err_px_ = 3.0f;
    float min_parallax_deg_ = 1.0f;

    std::atomic<bool> running_{false};
    std::thread worker_thread_;

    std::mutex mutex_tasks_;
    std::condition_variable cv_tasks_;
    std::deque<CorrInitTask> task_queue_;

    std::mutex mutex_results_;
    std::deque<CorrInitSeedPacket> result_queue_;

    std::size_t task_counter_ = 0;

    std::mutex log_mutex_;
    std::ofstream log_file_;
    bool log_header_written_ = false;

#ifdef WITH_CORRINIT_ZMQ
    std::unique_ptr<zmq::context_t> zmq_context_;
    std::unique_ptr<zmq::socket_t> zmq_socket_;
#endif
};
