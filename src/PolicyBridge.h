#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace uuv
{
/* Minimal TCP bridge that streams the same observation packet as the USIMNavTest
 * simulator so that the openpi policy client (eval_policy_sim.py) can drive the
 * BlueROV2 in this scene:
 *
 *   server -> client: [u32 total][29 x f32 state][u32 n][ego RGB 240x320x3]
 *                     [u32 n][wrist RGB 240x320x3][3 x f32 dists][3 x f32 robot pos]
 *   client -> server: 8 x f32 normalized thruster PWM
 */
class PolicyBridge
{
public:
    explicit PolicyBridge(int port);
    ~PolicyBridge();

    bool connected() const { return client_ >= 0; }

    // Sends one observation. Returns false if no client is connected / send failed.
    bool sendFrame(const std::array<float, 29>& state, const cv::Mat& egoBgr, const cv::Mat& wristBgr,
                   double goalDistance, const std::array<double, 3>& position, double goalBearing,
                   double routeDistance, double routeBearing);

    // Drains pending 8-float commands. Returns true and fills pwm when one was available.
    bool popCommand(std::array<double, 8>& pwm);

    void closeClient();

private:
    void tryAccept();
    void drain();

    int listenFd_ = -1;
    int client_ = -1;
    std::deque<std::array<double, 8>> commands_;
    bool awaitingAck_ = false;  // true after a frame until the policy replies
    std::vector<uint8_t> rxBuffer_;
};
}  // namespace uuv
