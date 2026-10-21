#include "Autopilot.h"

#include <algorithm>
#include <cmath>

namespace uuv
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double clamp(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}
}  // namespace

Autopilot::Autopilot(const EpisodeConfig& cfg) : cfg_(cfg)
{
    cumulative_.resize(cfg_.path.size(), 0.0);
    for (std::size_t i = 1; i < cfg_.path.size(); ++i)
    {
        const auto& a = cfg_.path[i - 1];
        const auto& b = cfg_.path[i];
        cumulative_[i] = cumulative_[i - 1] +
                         std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) +
                                   (b.z - a.z) * (b.z - a.z));
    }
}

void Autopilot::reset()
{
    nearestIndex_ = 0;
    ix_ = iy_ = iz_ = iyaw_ = 0.0;
}

double Autopilot::wrapAngle(double a)
{
    while (a > kPi)
        a -= 2.0 * kPi;
    while (a < -kPi)
        a += 2.0 * kPi;
    return a;
}

std::array<double, 9> Autopilot::rotationMatrix(const std::array<double, 4>& q)
{
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double xx = x * x, yy = y * y, zz = z * z;
    const double xy = x * y, xz = x * z, yz = y * z;
    const double wx = w * x, wy = w * y, wz = w * z;
    return {1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),     2.0 * (xz + wy),
            2.0 * (xy + wz),     1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
            2.0 * (xz - wy),     2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)};
}

Autopilot::Vec3 Autopilot::rotate(const std::array<double, 9>& R, const Vec3& v)
{
    return {R[0] * v.x + R[1] * v.y + R[2] * v.z, R[3] * v.x + R[4] * v.y + R[5] * v.z,
            R[6] * v.x + R[7] * v.y + R[8] * v.z};
}

Autopilot::Vec3 Autopilot::rotateTranspose(const std::array<double, 9>& R, const Vec3& v)
{
    return {R[0] * v.x + R[3] * v.y + R[6] * v.z, R[1] * v.x + R[4] * v.y + R[7] * v.z,
            R[2] * v.x + R[5] * v.y + R[8] * v.z};
}

std::array<double, 3> Autopilot::quatToRpy(const std::array<double, 4>& q)
{
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double sinr = 2.0 * (w * x + y * z);
    const double cosr = 1.0 - 2.0 * (x * x + y * y);
    const double roll = std::atan2(sinr, cosr);
    const double sinp = 2.0 * (w * y - z * x);
    const double pitch = std::fabs(sinp) >= 1.0 ? std::copysign(kPi / 2.0, sinp) : std::asin(sinp);
    const double siny = 2.0 * (w * z + x * y);
    const double cosy = 1.0 - 2.0 * (y * y + z * z);
    const double yaw = std::atan2(siny, cosy);
    return {roll, pitch, yaw};
}

Autopilot::Vec3 Autopilot::selectTarget(const RobotState& state, const std::vector<double>& sonar,
                                        double minSonar, bool& avoidanceActive)
{
    const Vec3 goal{cfg_.goalX, cfg_.goalY, cfg_.depth};
    const double goalDist =
        std::sqrt((goal.x - state.position[0]) * (goal.x - state.position[0]) +
                  (goal.y - state.position[1]) * (goal.y - state.position[1]) +
                  (goal.z - state.position[2]) * (goal.z - state.position[2]));

    Vec3 target;
    double targetYaw;

    if (goalDist < 1.6)
    {
        target = goal;
        targetYaw = cfg_.goalYaw;
    }
    else
    {
        // Nearest path point (global search; paths are short).
        std::size_t nearest = 0;
        double best = 1e18;
        for (std::size_t i = 0; i < cfg_.path.size(); ++i)
        {
            const double dx = cfg_.path[i].x - state.position[0];
            const double dy = cfg_.path[i].y - state.position[1];
            const double d2 = dx * dx + dy * dy;
            if (d2 < best)
            {
                best = d2;
                nearest = i;
            }
        }
        nearestIndex_ = nearest;

        // Pure-pursuit carrot: first path point farther than the lookahead.
        double lookahead = 1.0;
        if (minSonar < 2.5)
            lookahead = 0.55;
        std::size_t idx = nearest;
        while (idx + 1 < cfg_.path.size())
        {
            const double dx = cfg_.path[idx].x - state.position[0];
            const double dy = cfg_.path[idx].y - state.position[1];
            if (std::sqrt(dx * dx + dy * dy) >= lookahead)
                break;
            ++idx;
        }
        if (idx + 1 >= cfg_.path.size())
        {
            target = goal;
            targetYaw = cfg_.goalYaw;
        }
        else
        {
            target = {cfg_.path[idx].x, cfg_.path[idx].y, cfg_.depth};
            targetYaw = cfg_.path[idx].yaw;
        }
    }

    // Local sonar avoidance: repulsive lateral offset. The map-based A* path is
    // already safe; this layer reacts to unmapped or moving obstacles.
    avoidanceActive = false;
    if (!sonar.empty() && goalDist > 2.5)
    {
        const double d0 = 4.0;
        double repY = 0.0;
        double repForward = 0.0;
        const std::size_t n = sonar.size();
        for (std::size_t i = 0; i < n; ++i)
        {
            const double angle = -kPi / 4.0 + (2.0 * kPi / 4.0) * static_cast<double>(i) /
                                                 static_cast<double>(n - 1);
            const double r = sonar[i];
            if (r <= 0.05 || r >= d0)
                continue;
            const double w = (1.0 / std::max(r, 0.35) - 1.0 / d0);
            const double w2 = w * w;
            repY -= std::sin(angle) * w2 * 2.0;
            repForward -= std::cos(angle) * w2 * 0.8;
        }
        repY = clamp(repY, -1.8, 1.8);
        if (std::fabs(repY) > 0.03 || repForward < -0.05)
        {
            avoidanceActive = true;
            const double yaw = state.yaw;
            target.x += -std::sin(yaw) * repY;
            target.y += std::cos(yaw) * repY;
        }
    }

    lastTarget_ = target;
    lastTargetYaw_ = targetYaw;
    lastMinSonar_ = minSonar;
    lastAvoidance_ = avoidanceActive;
    return target;
}

std::array<double, 8> Autopilot::allocate(double fx, double fy, double fz, double roll, double pitch,
                                          double yaw) const
{
    std::array<double, 8> p{};
    p[0] = -fx + fy + yaw;
    p[1] = -fx - fy - yaw;
    p[2] = fx + fy - yaw;
    p[3] = fx - fy + yaw;

    p[4] = fz + roll - pitch;
    p[5] = fz - roll - pitch;
    p[6] = fz + roll + pitch;
    p[7] = fz - roll + pitch;

    for (auto& value : p)
        value = clamp(value, -0.85, 0.85);
    return p;
}

AutopilotOutput Autopilot::step(const RobotState& state, const std::vector<double>& sonar, double dt)
{
    dt = clamp(dt, 0.02, 0.5);

    double minSonar = 1e9;
    for (double r : sonar)
    {
        if (r > 0.05 && r < minSonar)
            minSonar = r;
    }
    if (minSonar > 1e8)
        minSonar = 50.0;
    lastMinSonar_ = minSonar;

    const auto R = rotationMatrix(state.orientation);
    bool avoidanceActive = false;
    const Vec3 target = selectTarget(state, sonar, minSonar, avoidanceActive);
    const double targetYaw = lastTargetYaw_;

    const Vec3 goal{cfg_.goalX, cfg_.goalY, cfg_.depth};
    const double goalDist =
        std::sqrt((goal.x - state.position[0]) * (goal.x - state.position[0]) +
                  (goal.y - state.position[1]) * (goal.y - state.position[1]) +
                  (goal.z - state.position[2]) * (goal.z - state.position[2]));

    // Target pose in body frame, matching the USIM target_pos definition.
    const Vec3 delta{target.x - state.position[0], target.y - state.position[1],
                     target.z - state.position[2]};
    const Vec3 deltaBody = rotateTranspose(R, delta);
    const double halfYaw = 0.5 * targetYaw;
    const std::array<double, 4> qTarget{0.0, 0.0, std::sin(halfYaw), std::cos(halfYaw)};
    const auto Rt = rotationMatrix(qTarget);
    std::array<double, 9> Rrel{};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
        {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k)
                sum += R[k * 3 + r] * Rt[k * 3 + c];
            Rrel[r * 3 + c] = sum;
        }
    const double relRoll = std::atan2(Rrel[7], Rrel[8]);
    const double relPitch = std::asin(clamp(-Rrel[6], -1.0, 1.0));
    const double relYaw = std::atan2(Rrel[3], Rrel[0]);

    const Vec3 errorBody = rotateTranspose(R, Vec3{target.x - state.position[0],
                                                  target.y - state.position[1],
                                                  target.z - state.position[2]});

    double vmax = 0.85;
    if (goalDist < 2.0)
        vmax = clamp(0.60 * goalDist, 0.12, 0.60);

    double sonarScale = 1.0;
    if (goalDist > 3.0 && minSonar < 2.0)
        sonarScale = clamp((minSonar - 0.8) / 1.2, 0.0, 1.0);

    double vxDes = clamp(0.9 * errorBody.x, -vmax, vmax) * sonarScale;
    double vyDes = clamp(1.0 * errorBody.y, -0.30, 0.30) * sonarScale;
    double vzDes = clamp(1.2 * errorBody.z, -0.30, 0.30);

    // If the goal is dead ahead and close, make sure we actually converge.
    if (goalDist < 1.6)
    {
        vxDes = clamp(0.6 * goalDist, 0.0, 0.4);
        if (std::fabs(relYaw) > 0.6)
            vxDes = 0.0;
    }

    const double ex = vxDes - state.velocity[0];
    const double ey = vyDes - state.velocity[1];
    const double ez = vzDes - state.velocity[2];

    ix_ = clamp(ix_ + ex * dt, -0.6, 0.6);
    iy_ = clamp(iy_ + ey * dt, -0.35, 0.35);
    iz_ = clamp(iz_ + ez * dt, -0.55, 0.55);

    double fx = 2.5 * ex + 0.4 * ix_;
    double fy = 1.5 * ey + 0.2 * iy_;
    double fz = 1.5 * ez + 0.4 * iz_;

    // Attitude loops (small-angle PD).
    const double yawErr = wrapAngle(targetYaw - state.yaw);
    iyaw_ = clamp(iyaw_ + yawErr * dt, -0.3, 0.3);
    const double yawCmd = 1.35 * yawErr + 0.08 * iyaw_ - 0.25 * state.angularRate[2];
    const double rollCmd = -0.55 * state.roll - 0.10 * state.angularRate[0];
    const double pitchCmd = -0.55 * state.pitch - 0.10 * state.angularRate[1];

    AutopilotOutput out;
    out.pwm = allocate(fx, fy, fz, rollCmd, pitchCmd, yawCmd);
    out.targetLocal = {deltaBody.x, deltaBody.y, deltaBody.z, relRoll, relPitch, relYaw};
    out.targetWorld = {target.x, target.y, target.z};
    out.targetYaw = targetYaw;
    out.goalDistance = goalDist;
    out.minSonar = minSonar;
    out.avoidanceActive = avoidanceActive;
    return out;
}
}  // namespace uuv
