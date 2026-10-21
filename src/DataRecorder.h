#pragma once

#include <array>
#include <fstream>
#include <string>

#include <opencv2/core.hpp>

#include "EpisodeConfig.h"

namespace uuv
{
class DataRecorder
{
public:
    DataRecorder(const std::string& episodeDir, const EpisodeConfig& cfg);

    bool isOpen() const { return stream_.is_open(); }

    void addFrame(int frameIndex, double timestamp, const std::array<float, 29>& state,
                  const std::array<float, 13>& action, const std::array<float, 6>& target,
                  const std::array<double, 3>& position, double dvlAltitude, double goalDistance,
                  double minSonar, bool avoidance, const cv::Mat& egoBgr, const cv::Mat& wristBgr);

    void finalize(bool success, const std::string& reason, double simTime, double goalDistance,
                  double minClearance, const std::string& extra = "");

    int frames() const { return frameCount_; }

private:
    std::string episodeDir_;
    std::string egoDir_;
    std::string wristDir_;
    std::ofstream stream_;
    int frameCount_ = 0;
    cv::Mat lastEgo_;
    cv::Mat lastWrist_;
};
}  // namespace uuv
