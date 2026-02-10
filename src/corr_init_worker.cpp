/**
 * CorrInit worker (EfficientLoFTR sidecar)
 *
 * This file is part of Photo-SLAM
 */

#include "include/corr_init_worker.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <numeric>
#include <sstream>

#include <opencv2/imgproc.hpp>
#include <jsoncpp/json/json.h>

#include "include/triangulator.h"

#ifdef WITH_CORRINIT_ZMQ
#include <zmq.hpp>
#endif

namespace
{
cv::Mat toGrayU8(const cv::Mat& img_rgb_f32)
{
    cv::Mat gray_f32, gray_u8;
    if (img_rgb_f32.empty())
        return gray_u8;
    cv::cvtColor(img_rgb_f32, gray_f32, cv::COLOR_RGB2GRAY);
    gray_f32.convertTo(gray_u8, CV_8UC1, 255.0);
    return gray_u8;
}

bool parseHeaderJson(const std::string& json_str, int& num_matches)
{
    Json::CharReaderBuilder builder;
    std::string errs;
    Json::Value root;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(json_str.data(), json_str.data() + json_str.size(), &root, &errs))
        return false;
    num_matches = root.get("num_matches", 0).asInt();
    return true;
}

long long nowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

std::string escapeCsv(const std::string& input)
{
    std::string out;
    out.reserve(input.size() + 2);
    out.push_back('"');
    for (char c : input) {
        if (c == '"')
            out.append("\"\"");
        else if (c == '\n' || c == '\r')
            out.push_back(' ');
        else
            out.push_back(c);
    }
    out.push_back('"');
    return out;
}
} // namespace

CorrInitWorker::CorrInitWorker(
    const std::string& zmq_endpoint,
    const std::string& log_path,
    int queue_capacity,
    int num_seeds,
    int num_oversample,
    float reproj_err_px,
    float min_parallax_deg)
    : zmq_endpoint_(zmq_endpoint),
      log_path_(log_path),
      queue_capacity_(queue_capacity),
      num_seeds_(num_seeds),
      num_oversample_(num_oversample),
      reproj_err_px_(reproj_err_px),
      min_parallax_deg_(min_parallax_deg)
{
}

CorrInitWorker::~CorrInitWorker()
{
    stop();
}

void CorrInitWorker::start()
{
    if (running_)
        return;
    running_ = true;
    logSimpleEvent("worker_start", std::string("endpoint=") + zmq_endpoint_);
    worker_thread_ = std::thread(&CorrInitWorker::threadLoop, this);
}

void CorrInitWorker::stop()
{
    if (!running_)
        return;
    running_ = false;
    cv_tasks_.notify_all();
    if (worker_thread_.joinable())
        worker_thread_.join();
#ifdef WITH_CORRINIT_ZMQ
    resetZmqSocket();
#endif
    logSimpleEvent("worker_stop", "ok");
}

void CorrInitWorker::enqueueTask(CorrInitTask&& task)
{
    std::unique_lock<std::mutex> lock(mutex_tasks_);
    if (static_cast<int>(task_queue_.size()) >= queue_capacity_) {
        task_queue_.pop_front(); // drop oldest
        logTaskStats("queue_drop", task.ref.kfid,
            task.neighbors.empty() ? 0 : task.neighbors.front().kfid,
            -1, -1, -1, -1, -1, -1, -1, 0,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0, task_queue_.size(), -1, "drop_oldest");
    }
    task_queue_.push_back(std::move(task));
    cv_tasks_.notify_one();
}

bool CorrInitWorker::tryPopSeedPacket(CorrInitSeedPacket& out_packet)
{
    std::unique_lock<std::mutex> lock(mutex_results_);
    if (result_queue_.empty())
        return false;
    out_packet = std::move(result_queue_.front());
    result_queue_.pop_front();
    return true;
}

void CorrInitWorker::logCsvLine(const std::string& line)
{
    if (log_path_.empty())
        return;

    std::lock_guard<std::mutex> lock(log_mutex_);
    if (!log_header_written_) {
        try {
            std::filesystem::path path(log_path_);
            if (path.has_parent_path())
                std::filesystem::create_directories(path.parent_path());
            bool need_header = true;
            if (std::filesystem::exists(path)) {
                auto fsize = std::filesystem::file_size(path);
                need_header = (fsize == 0);
            }
            log_file_.open(path, std::ios::out | std::ios::app);
            if (!log_file_.is_open())
                return;
            if (need_header) {
                log_file_
                    << "timestamp_ms,event,ref_kfid,neighbor_kfid,"
                    << "matches_raw,matches_in_bounds,oversample,triangulated,"
                    << "cheirality_pass,reproj_pass,parallax_pass,seeds_out,"
                    << "median_reproj,p90_reproj,median_parallax,p90_parallax,"
                    << "task_ms,queue_size,iter,note"
                    << "\n";
                log_file_.flush();
            }
            log_header_written_ = true;
        }
        catch (...) {
            return;
        }
    }

    if (!log_file_.is_open())
        return;

    log_file_ << line << "\n";
    log_file_.flush();
}

void CorrInitWorker::logTaskStats(
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
    const std::string& note)
{
    std::ostringstream oss;
    oss << nowMs() << ","
        << event << ","
        << ref_kfid << ","
        << neighbor_kfid << ","
        << matches_raw << ","
        << matches_in_bounds << ","
        << oversample << ","
        << num_triangulated << ","
        << num_cheirality_pass << ","
        << num_reproj_pass << ","
        << num_parallax_pass << ","
        << seeds_out << ","
        << std::fixed << std::setprecision(4)
        << median_reproj << ","
        << p90_reproj << ","
        << median_parallax << ","
        << p90_parallax << ","
        << std::setprecision(2)
        << task_ms << ","
        << queue_size << ","
        << iter << ","
        << escapeCsv(note);
    logCsvLine(oss.str());
}

void CorrInitWorker::logSimpleEvent(const std::string& event, const std::string& note)
{
    logTaskStats(event, 0, 0,
        -1, -1, -1, -1, -1, -1, -1, -1,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0, 0, -1, note);
}

void CorrInitWorker::logIntegration(const CorrInitSeedPacket& packet, int iter)
{
    logTaskStats("integrate", packet.ref_kfid, packet.neighbor_kfid,
        -1, -1, -1, -1, -1, -1, -1,
        static_cast<int>(packet.points.size()),
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0, 0, iter, "ok");
}

#ifdef WITH_CORRINIT_ZMQ
bool CorrInitWorker::ensureZmqSocket()
{
    if (zmq_socket_)
        return true;
    try {
        if (!zmq_context_)
            zmq_context_ = std::make_unique<zmq::context_t>(1);
        zmq_socket_ = std::make_unique<zmq::socket_t>(*zmq_context_, zmq::socket_type::req);
        zmq_socket_->set(zmq::sockopt::linger, 0);
        zmq_socket_->set(zmq::sockopt::rcvtimeo, 5000);
        zmq_socket_->set(zmq::sockopt::sndtimeo, 5000);
        zmq_socket_->connect(zmq_endpoint_);
        logSimpleEvent("zmq_connect", "ok");
        return true;
    }
    catch (const zmq::error_t& e) {
        logSimpleEvent("zmq_error", std::string("connect:") + e.what());
        resetZmqSocket();
        return false;
    }
}

void CorrInitWorker::resetZmqSocket()
{
    if (zmq_socket_) {
        try {
            zmq_socket_->close();
        }
        catch (...) {
        }
    }
    zmq_socket_.reset();
}
#endif

void CorrInitWorker::threadLoop()
{
#ifdef WITH_CORRINIT_ZMQ
    ensureZmqSocket();
#endif
    while (running_) {
        CorrInitTask task;
        std::size_t queue_size = 0;
        {
            std::unique_lock<std::mutex> lock(mutex_tasks_);
            cv_tasks_.wait(lock, [&] { return !running_ || !task_queue_.empty(); });
            if (!running_)
                break;
            task = std::move(task_queue_.front());
            task_queue_.pop_front();
            queue_size = task_queue_.size();
        }

        CorrInitSeedPacket packet;
        if (processTask(task, packet, queue_size)) {
            std::unique_lock<std::mutex> lock(mutex_results_);
            result_queue_.push_back(std::move(packet));
        }
    }
}

bool CorrInitWorker::processTask(const CorrInitTask& task, CorrInitSeedPacket& out_packet, std::size_t queue_size)
{
#ifndef WITH_CORRINIT_ZMQ
    (void)task;
    (void)out_packet;
    (void)queue_size;
    return false;
#else
    auto t_start = std::chrono::steady_clock::now();
    auto taskMs = [&]() -> double {
        auto t_end = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(t_end - t_start).count();
    };

    if (task.neighbors.empty()) {
        logTaskStats("task_fail", task.ref.kfid, 0,
            -1, -1, -1, -1, -1, -1, -1, 0,
            0.0f, 0.0f, 0.0f, 0.0f, taskMs(), queue_size, -1, "no_neighbors");
        return false;
    }

    const KeyframeSnapshot& ref = task.ref;
    cv::Mat img0 = toGrayU8(ref.image_undist);
    if (img0.empty()) {
        logTaskStats("task_fail", ref.kfid, 0,
            -1, -1, -1, -1, -1, -1, -1, 0,
            0.0f, 0.0f, 0.0f, 0.0f, taskMs(), queue_size, -1, "empty_ref_image");
        return false;
    }
    if (!img0.isContinuous())
        img0 = img0.clone();

    cv::Mat K0 = (cv::Mat_<float>(3, 3)
                    << ref.intr[0], 0.0f, ref.intr[2],
                       0.0f, ref.intr[1], ref.intr[3],
                       0.0f, 0.0f, 1.0f);

    struct Candidate
    {
        Eigen::Vector3f Xw;
        float err = 0.0f;
        float parallax = 0.0f;
        float u0 = 0.0f;
        float v0 = 0.0f;
        unsigned long nb_kfid = 0;
    };

    std::vector<Candidate> all_candidates;
    all_candidates.reserve(static_cast<std::size_t>(num_oversample_) * task.neighbors.size());

    int total_matches_raw = 0;
    int total_matches_in_bounds = 0;
    int total_oversample = 0;
    int total_triangulated = 0;
    int total_cheirality = 0;
    int total_reproj = 0;
    int total_parallax = 0;

    auto medianFromVector = [](std::vector<float>& v) -> float {
        if (v.empty())
            return 0.0f;
        std::size_t mid = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + mid, v.end());
        return v[mid];
    };
    auto p90FromVector = [](std::vector<float>& v) -> float {
        if (v.empty())
            return 0.0f;
        std::size_t idx = static_cast<std::size_t>(std::floor(0.9 * (v.size() - 1)));
        std::nth_element(v.begin(), v.begin() + idx, v.end());
        return v[idx];
    };

    for (const auto& nb : task.neighbors) {
        cv::Mat img1 = toGrayU8(nb.image_undist);
        if (img1.empty()) {
            logTaskStats("task_fail", ref.kfid, nb.kfid,
                -1, -1, -1, -1, -1, -1, -1, 0,
                0.0f, 0.0f, 0.0f, 0.0f, taskMs(), queue_size, -1, "empty_neighbor_image");
            continue;
        }
        if (!img1.isContinuous())
            img1 = img1.clone();

        if (!ensureZmqSocket()) {
            logTaskStats("task_fail", ref.kfid, nb.kfid,
                -1, -1, -1, -1, -1, -1, -1, 0,
                0.0f, 0.0f, 0.0f, 0.0f, taskMs(), queue_size, -1, "zmq_unavailable");
            return false;
        }

        Json::Value header;
        header["req_id"] = static_cast<Json::UInt64>(ref.kfid);
        header["h0"] = img0.rows;
        header["w0"] = img0.cols;
        header["h1"] = img1.rows;
        header["w1"] = img1.cols;
        Json::StreamWriterBuilder wbuilder;
        std::string header_str = Json::writeString(wbuilder, header);
        zmq::message_t header_msg(header_str.size());
        std::memcpy(header_msg.data(), header_str.data(), header_str.size());
        zmq::message_t img0_msg(img0.total());
        std::memcpy(img0_msg.data(), img0.data, img0.total());
        zmq::message_t img1_msg(img1.total());
        std::memcpy(img1_msg.data(), img1.data, img1.total());

        zmq::message_t resp_header;
        zmq::message_t resp_mkpts0;
        zmq::message_t resp_mkpts1;
        zmq::message_t resp_mconf;

        bool zmq_ok = true;
        try {
            if (!zmq_socket_->send(header_msg, zmq::send_flags::sndmore))
                zmq_ok = false;
            if (zmq_ok && !zmq_socket_->send(img0_msg, zmq::send_flags::sndmore))
                zmq_ok = false;
            if (zmq_ok && !zmq_socket_->send(img1_msg, zmq::send_flags::none))
                zmq_ok = false;
            if (zmq_ok && !zmq_socket_->recv(resp_header, zmq::recv_flags::none))
                zmq_ok = false;
            if (zmq_ok && !zmq_socket_->recv(resp_mkpts0, zmq::recv_flags::none))
                zmq_ok = false;
            if (zmq_ok && !zmq_socket_->recv(resp_mkpts1, zmq::recv_flags::none))
                zmq_ok = false;
            if (zmq_ok && !zmq_socket_->recv(resp_mconf, zmq::recv_flags::none))
                zmq_ok = false;
        }
        catch (const zmq::error_t& e) {
            zmq_ok = false;
            logTaskStats("task_fail", ref.kfid, nb.kfid,
                -1, -1, -1, -1, -1, -1, -1, 0,
                0.0f, 0.0f, 0.0f, 0.0f, taskMs(), queue_size, -1,
                std::string("zmq_error:") + e.what());
        }

        if (!zmq_ok) {
            logTaskStats("task_fail", ref.kfid, nb.kfid,
                -1, -1, -1, -1, -1, -1, -1, 0,
                0.0f, 0.0f, 0.0f, 0.0f, taskMs(), queue_size, -1, "zmq_io");
            resetZmqSocket();
            continue;
        }

        std::string resp_header_str(static_cast<char*>(resp_header.data()), resp_header.size());
        int num_matches = 0;
        if (!parseHeaderJson(resp_header_str, num_matches)) {
            logTaskStats("task_fail", ref.kfid, nb.kfid,
                -1, -1, -1, -1, -1, -1, -1, 0,
                0.0f, 0.0f, 0.0f, 0.0f, taskMs(), queue_size, -1, "bad_header");
            continue;
        }

        if (num_matches <= 0) {
            logTaskStats("task_nb", ref.kfid, nb.kfid,
                0, 0, 0, 0, 0, 0, 0, 0,
                0.0f, 0.0f, 0.0f, 0.0f, taskMs(), queue_size, -1, "zero_matches");
            continue;
        }

        const std::size_t expected_mkpts_bytes = static_cast<std::size_t>(num_matches) * 2 * sizeof(float);
        const std::size_t expected_conf_bytes = static_cast<std::size_t>(num_matches) * sizeof(float);
        if (resp_mkpts0.size() < expected_mkpts_bytes ||
            resp_mkpts1.size() < expected_mkpts_bytes ||
            resp_mconf.size() < expected_conf_bytes) {
            logTaskStats("task_fail", ref.kfid, nb.kfid,
                num_matches, 0, 0, 0, 0, 0, 0, 0,
                0.0f, 0.0f, 0.0f, 0.0f, taskMs(), queue_size, -1, "bad_payload");
            resetZmqSocket();
            continue;
        }

        const float* mkpts0_ptr = reinterpret_cast<const float*>(resp_mkpts0.data());
        const float* mkpts1_ptr = reinterpret_cast<const float*>(resp_mkpts1.data());
        const float* mconf_ptr = reinterpret_cast<const float*>(resp_mconf.data());

        std::vector<Match2D> matches;
        matches.reserve(num_matches);
        for (int i = 0; i < num_matches; ++i) {
            Match2D m;
            m.u0 = mkpts0_ptr[2 * i];
            m.v0 = mkpts0_ptr[2 * i + 1];
            m.u1 = mkpts1_ptr[2 * i];
            m.v1 = mkpts1_ptr[2 * i + 1];
            m.conf = mconf_ptr[i];
            if (m.u0 < 0 || m.v0 < 0 || m.u0 >= ref.image_undist.cols || m.v0 >= ref.image_undist.rows)
                continue;
            if (m.u1 < 0 || m.v1 < 0 || m.u1 >= nb.image_undist.cols || m.v1 >= nb.image_undist.rows)
                continue;
            matches.push_back(m);
        }

        if (matches.empty()) {
            logTaskStats("task_nb", ref.kfid, nb.kfid,
                num_matches, 0, 0, 0, 0, 0, 0, 0,
                0.0f, 0.0f, 0.0f, 0.0f, taskMs(), queue_size, -1, "all_oob");
            continue;
        }

        // Deterministic top-N' by confidence (LoFTR)
        int M = static_cast<int>(matches.size());
        int Np = std::min(num_oversample_, M);
        std::vector<int> indices(M);
        std::iota(indices.begin(), indices.end(), 0);
        if (Np < M) {
            std::nth_element(indices.begin(), indices.begin() + Np, indices.end(),
                [&](int a, int b) { return matches[a].conf > matches[b].conf; });
            indices.resize(Np);
        }

        std::vector<Match2D> selected;
        selected.reserve(Np);
        for (int idx : indices)
            selected.push_back(matches[idx]);

        cv::Mat K1 = (cv::Mat_<float>(3, 3)
                        << nb.intr[0], 0.0f, nb.intr[2],
                           0.0f, nb.intr[1], nb.intr[3],
                           0.0f, 0.0f, 1.0f);

        TriangulationStats stats;
        auto tri_points = Triangulator::triangulate(
            K0, ref.Tcw, K1, nb.Tcw,
            selected, reproj_err_px_, min_parallax_deg_, &stats);

        int neighbor_valids = 0;
        for (std::size_t i = 0; i < tri_points.size(); ++i) {
            const auto& tp = tri_points[i];
            if (!tp.valid)
                continue;
            const auto& m = selected[tp.match_idx];
            Candidate c;
            c.Xw = tp.Xw;
            c.err = tp.reproj_err;
            c.parallax = tp.parallax_deg;
            c.u0 = m.u0;
            c.v0 = m.v0;
            c.nb_kfid = nb.kfid;
            all_candidates.push_back(c);
            neighbor_valids++;
        }

        total_matches_raw += num_matches;
        total_matches_in_bounds += static_cast<int>(matches.size());
        total_oversample += static_cast<int>(selected.size());
        total_triangulated += stats.num_triangulated;
        total_cheirality += stats.num_cheirality_pass;
        total_reproj += stats.num_reproj_pass;
        total_parallax += stats.num_parallax_pass;

        logTaskStats("task_nb", ref.kfid, nb.kfid,
            num_matches,
            static_cast<int>(matches.size()),
            static_cast<int>(selected.size()),
            stats.num_triangulated,
            stats.num_cheirality_pass,
            stats.num_reproj_pass,
            stats.num_parallax_pass,
            neighbor_valids,
            stats.median_reproj,
            stats.p90_reproj,
            stats.median_parallax,
            stats.p90_parallax,
            taskMs(), queue_size, -1, "ok");
    }

    if (all_candidates.empty()) {
        logTaskStats("task", ref.kfid, 0,
            total_matches_raw,
            total_matches_in_bounds,
            total_oversample,
            total_triangulated,
            total_cheirality,
            total_reproj,
            total_parallax,
            0,
            0.0f, 0.0f, 0.0f, 0.0f,
            taskMs(), queue_size, -1, "no_valid");
        return false;
    }

    std::vector<float> final_errs;
    std::vector<float> final_parallax;
    final_errs.reserve(all_candidates.size());
    final_parallax.reserve(all_candidates.size());
    for (const auto& c : all_candidates) {
        final_errs.push_back(c.err);
        final_parallax.push_back(c.parallax);
    }

    std::sort(all_candidates.begin(), all_candidates.end(),
        [](const Candidate& a, const Candidate& b) { return a.err < b.err; });

    int N = std::min(num_seeds_, static_cast<int>(all_candidates.size()));
    out_packet.ref_kfid = ref.kfid;
    out_packet.neighbor_kfid = (task.neighbors.size() == 1) ? task.neighbors.front().kfid : 0;
    out_packet.points.reserve(N);
    out_packet.colors.reserve(N);

    for (int i = 0; i < N; ++i) {
        const auto& c = all_candidates[i];
        int x = static_cast<int>(std::round(c.u0));
        int y = static_cast<int>(std::round(c.v0));
        x = std::max(0, std::min(x, ref.image_undist.cols - 1));
        y = std::max(0, std::min(y, ref.image_undist.rows - 1));
        cv::Vec3f color = ref.image_undist.at<cv::Vec3f>(y, x);
        out_packet.points.push_back(c.Xw);
        out_packet.colors.push_back(color);
    }

    task_counter_++;
    logTaskStats("task", ref.kfid, 0,
        total_matches_raw,
        total_matches_in_bounds,
        total_oversample,
        total_triangulated,
        total_cheirality,
        total_reproj,
        total_parallax,
        static_cast<int>(out_packet.points.size()),
        medianFromVector(final_errs),
        p90FromVector(final_errs),
        medianFromVector(final_parallax),
        p90FromVector(final_parallax),
        taskMs(), queue_size, -1, "ok");

    return !out_packet.points.empty();
#endif
}
