#include "DataRecorder.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iomanip>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace uuv
{
DataRecorder::DataRecorder(const std::string& episodeDir, const EpisodeConfig& cfg)
    : episodeDir_(episodeDir), egoDir_(episodeDir + "/images/ego"), wristDir_(episodeDir + "/images/wrist"),
      goalX_(cfg.goalX), goalY_(cfg.goalY)
{
    // A rerun in the same episode directory must start from a clean image
    // sequence, otherwise stale high-index PNGs would confuse the ROS bridge.
    std::filesystem::remove_all(egoDir_);
    std::filesystem::remove_all(wristDir_);
    std::filesystem::create_directories(egoDir_);
    std::filesystem::create_directories(wristDir_);

    stream_.open(episodeDir_ + "/samples.csv", std::ios::out);
    stream_ << std::fixed << std::setprecision(9);
    stream_ << "timestamp";
    for (int i = 0; i < 29; ++i)
        stream_ << ",state_" << i;
    for (int i = 0; i < 13; ++i)
        stream_ << ",action_" << i;
    for (int i = 0; i < 6; ++i)
        stream_ << ",target_" << i;
    stream_ << ",pos_x,pos_y,pos_z,dvl_alt,goal_dist,min_sonar,avoidance,yaw,goal_bearing\n";

    std::ofstream cfgOut(episodeDir_ + "/episode_meta.txt");
    cfgOut << "task=" << cfg.task << "\n";
    cfgOut << "seed=" << cfg.seed << "\n";
    cfgOut << "episode_index=" << cfg.episodeIndex << "\n";
    cfgOut << "start=" << cfg.startX << " " << cfg.startY << " " << cfg.startZ << " " << cfg.startYaw << "\n";
    cfgOut << "goal=" << cfg.goalX << " " << cfg.goalY << " " << cfg.goalZ << " " << cfg.goalYaw << "\n";
    cfgOut << "success_radius=" << cfg.successRadius << "\n";
}

void DataRecorder::addFrame(int frameIndex, double timestamp, const std::array<float, 29>& state,
                            const std::array<float, 13>& action, const std::array<float, 6>& target,
                            const std::array<double, 3>& position, double dvlAltitude,
                            double goalDistance, double minSonar, bool avoidance, double yaw,
                            const cv::Mat& egoBgr, const cv::Mat& wristBgr)
{
    cv::Mat ego;
    cv::Mat wrist;
    if (!egoBgr.empty())
    {
        cv::resize(egoBgr, ego, cv::Size(320, 240), 0, 0, cv::INTER_LINEAR);
        lastEgo_ = ego.clone();
    }
    else
    {
        ego = lastEgo_.clone();
    }
    if (!wristBgr.empty())
    {
        cv::resize(wristBgr, wrist, cv::Size(320, 240), 0, 0, cv::INTER_LINEAR);
        lastWrist_ = wrist.clone();
    }
    else
    {
        wrist = lastWrist_.clone();
    }

    if (!ego.empty())
    {
        char name[64];
        std::snprintf(name, sizeof(name), "/%06d.png", frameIndex);
        cv::imwrite(egoDir_ + name, ego, {cv::IMWRITE_PNG_COMPRESSION, 3});
    }
    if (!wrist.empty())
    {
        char name[64];
        std::snprintf(name, sizeof(name), "/%06d.png", frameIndex);
        cv::imwrite(wristDir_ + name, wrist, {cv::IMWRITE_PNG_COMPRESSION, 3});
    }

    stream_ << timestamp;
    for (float v : state)
        stream_ << "," << v;
    for (float v : action)
        stream_ << "," << v;
    for (float v : target)
        stream_ << "," << v;
    const double worldBearing = std::atan2(goalY_ - position[1], goalX_ - position[0]);
    const double bearing = std::atan2(std::sin(worldBearing - yaw), std::cos(worldBearing - yaw));
    stream_ << "," << position[0] << "," << position[1] << "," << position[2] << "," << dvlAltitude
            << "," << goalDistance << "," << minSonar << "," << (avoidance ? 1 : 0)
            << "," << yaw << "," << bearing << "\n";
    ++frameCount_;
}

void DataRecorder::finalize(bool success, const std::string& reason, double simTime,
                            double goalDistance, double minClearance, const std::string& extra)
{
    if (stream_.is_open())
        stream_.flush();
    std::ofstream out(episodeDir_ + "/result.txt");
    out << "success=" << (success ? 1 : 0) << "\n";
    out << "reason=" << reason << "\n";
    out << "frames=" << frameCount_ << "\n";
    out << "sim_time=" << std::fixed << std::setprecision(6) << simTime << "\n";
    out << "goal_distance=" << goalDistance << "\n";
    out << "min_clearance=" << minClearance << "\n";
    if (!extra.empty())
        out << "extra=" << extra << "\n";
}
}  // namespace uuv
