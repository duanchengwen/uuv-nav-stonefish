#pragma once

#include <array>
#include <vector>

#include "EpisodeConfig.h"

namespace uuv
{
struct RobotState
{
    std::array<double, 3> position{};     // world NED [m]
    std::array<double, 4> orientation{};  // quaternion x,y,z,w (world <- body)
    std::array<double, 3> velocity{};     // body frame [m/s]
    std::array<double, 3> angularRate{};  // body frame [rad/s]
    double yaw = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
};

struct AutopilotOutput
{
    std::array<double, 8> pwm{};
    std::array<double, 6> targetLocal{};  // target pose in body frame (USIM target_pos)
    std::array<double, 3> targetWorld{};
    double targetYaw = 0.0;
    double goalDistance = 0.0;
    double minSonar = 0.0;
    bool avoidanceActive = false;
};

class Autopilot
{
public:
    explicit Autopilot(const EpisodeConfig& cfg, bool directMode = false);

    void reset();
    AutopilotOutput step(const RobotState& state, const std::vector<double>& sonar, double dt);

private:
    struct Vec3
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    static double wrapAngle(double a);
    static std::array<double, 9> rotationMatrix(const std::array<double, 4>& q);
    static Vec3 rotate(const std::array<double, 9>& R, const Vec3& v);
    static Vec3 rotateTranspose(const std::array<double, 9>& R, const Vec3& v);
    static std::array<double, 3> quatToRpy(const std::array<double, 4>& q);

    Vec3 selectTarget(const RobotState& state, const std::vector<double>& sonar, double minSonar,
                      bool& avoidanceActive);
    Vec3 planLocalTarget(const RobotState& state, const std::vector<double>& sonar, const Vec3& goal,
                         double goalDist, bool& avoidanceActive);
    std::array<double, 8> allocate(double fx, double fy, double fz, double roll, double pitch, double yaw) const;

    EpisodeConfig cfg_;
    std::vector<double> cumulative_;
    std::size_t nearestIndex_ = 0;

    // Direct-bearing expert: steer at the goal, detour around sonar obstacles, then resume.
    bool directMode_ = false;
    bool detourActive_ = false;
    double detourSign_ = 1.0;
    int detourHold_ = 0;
    // Committed detour side: -1 = left, +1 = right, 0 = none. Kept until the direct path clears,
    // which prevents left/right dithering in symmetric gate situations.
    int detourSide_ = 0;
    int clearHold_ = 0;

    // Integrators
    double ix_ = 0.0;
    double iy_ = 0.0;
    double iz_ = 0.0;
    double iyaw_ = 0.0;

    // Last target for reporting
    Vec3 lastTarget_{};
    double lastTargetYaw_ = 0.0;
    double lastMinSonar_ = 0.0;
    bool lastAvoidance_ = false;
};
}  // namespace uuv
