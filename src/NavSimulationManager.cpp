#include "NavSimulationManager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>

#include <core/GraphicalSimulationApp.h>
#include <core/Robot.h>
#include <core/ScenarioParser.h>
#include <core/SimulationApp.h>
#include <actuators/Thruster.h>
#include <sensors/Sample.h>
#include <sensors/ScalarSensor.h>
#include <sensors/scalar/DVL.h>
#include <sensors/scalar/IMU.h>
#include <sensors/scalar/Multibeam.h>
#include <sensors/scalar/Odometry.h>
#include <sensors/scalar/Pressure.h>
#include <sensors/vision/ColorCamera.h>

#include <opencv2/imgproc.hpp>

namespace uuv
{
namespace
{
constexpr const char* kRobotName = "bluerov2";
constexpr const char* kThrusterNames[8] = {"FrontRight", "FrontLeft", "BackRight", "BackLeft",
                                          "DiveFrontRight", "DiveFrontLeft", "DiveBackRight",
                                          "DiveBackLeft"};
constexpr double kSeabedZ = 3.0;

double wrapAngle(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}

bool readLast(sf::ScalarSensor* sensor, std::vector<double>& values)
{
    if (sensor == nullptr)
        return false;
    std::unique_ptr<const std::vector<sf::Sample>> history(sensor->getHistory());
    if (!history || history->empty())
        return false;
    const auto data = history->back().getData();
    values.assign(data.begin(), data.end());
    return !values.empty();
}
}  // namespace

NavSimulationManager::NavSimulationManager(sf::Scalar stepsPerSecond, std::string scenePath,
                                           std::string episodeDir, bool thrustTest, bool showCamera,
                                           int policyPort, bool directExpert)
    : sf::SimulationManager(stepsPerSecond, sf::SolverType::SOLVER_SI,
                            sf::CollisionFilteringType::COLLISION_EXCLUSIVE),
      scenePath_(std::move(scenePath)),
      episodeDir_(std::move(episodeDir)),
      thrustTest_(thrustTest),
      showCamera_(showCamera)
{
    std::string error;
    if (!loadEpisode(episodeDir_, cfg_, error))
        throw std::runtime_error("episode config: " + error);
    autopilot_ = std::make_unique<Autopilot>(cfg_, directExpert);
    recorder_ = std::make_unique<DataRecorder>(episodeDir_, cfg_);
    if (policyPort > 0)
    {
        bridge_ = std::make_unique<PolicyBridge>(policyPort);
        // The expert always runs the thrusters near the cruise operating point, so the recorded
        // state PWM sits around +-0.8 (ch0-3) and +0.1 (ch4-7). Start from the same point:
        // feeding zeros is far outside the training distribution.
        policyPwm_ = {-0.804, -0.777, 0.778, 0.797, 0.122, 0.119, 0.096, 0.093};
    }
}

NavSimulationManager::~NavSimulationManager()
{
    stopRequested_ = true;
    if (controlThread_.joinable())
        controlThread_.join();
}

void NavSimulationManager::BuildScenario()
{
    sf::ScenarioParser parser(this);
    if (!parser.Parse(scenePath_))
        throw std::runtime_error("failed to parse scenario: " + scenePath_);

    robot_ = getRobot(kRobotName);
    if (robot_ == nullptr)
        throw std::runtime_error("robot 'bluerov2' not found");

    for (int i = 0; i < 8; ++i)
    {
        thrusters_[i] = dynamic_cast<sf::Thruster*>(robot_->getActuator(std::string(kRobotName) + "/" + kThrusterNames[i]));
        if (thrusters_[i] == nullptr)
            throw std::runtime_error(std::string("thruster not found: ") + kThrusterNames[i]);
    }

    egoCamera_ = dynamic_cast<sf::ColorCamera*>(getSensor(std::string(kRobotName) + "/camera_ego"));
    wristCamera_ = dynamic_cast<sf::ColorCamera*>(getSensor(std::string(kRobotName) + "/camera_wrist"));
    odometry_ = dynamic_cast<sf::Odometry*>(getSensor(std::string(kRobotName) + "/odometry"));
    imu_ = dynamic_cast<sf::IMU*>(getSensor(std::string(kRobotName) + "/imu_filter"));
    dvl_ = dynamic_cast<sf::DVL*>(getSensor(std::string(kRobotName) + "/dvl"));
    pressure_ = dynamic_cast<sf::Pressure*>(getSensor(std::string(kRobotName) + "/pressure"));
    multibeam_ = dynamic_cast<sf::Multibeam*>(getSensor(std::string(kRobotName) + "/multibeam"));

    if (!egoCamera_ || !wristCamera_ || !odometry_ || !imu_ || !dvl_ || !pressure_ || !multibeam_)
        throw std::runtime_error("one or more required sensors are missing");

    if (showCamera_)
    {
        // Put the forward ego view into the main window (HUD occupies the left panel).
        egoCamera_->setDisplayOnScreen(true, 180, 0, 1.0f);
    }

    egoCamera_->InstallNewDataHandler([this](sf::ColorCamera* cam) {
        unsigned int w = 0;
        unsigned int h = 0;
        cam->getResolution(w, h);
        cv::Mat rgb(h, w, CV_8UC3, cam->getImageDataPointer());
        std::lock_guard<std::mutex> lock(imageMutex_);
        cv::cvtColor(rgb, egoImage_, cv::COLOR_RGB2BGR);
    });
    wristCamera_->InstallNewDataHandler([this](sf::ColorCamera* cam) {
        unsigned int w = 0;
        unsigned int h = 0;
        cam->getResolution(w, h);
        cv::Mat rgb(h, w, CV_8UC3, cam->getImageDataPointer());
        std::lock_guard<std::mutex> lock(imageMutex_);
        cv::cvtColor(rgb, wristImage_, cv::COLOR_RGB2BGR);
    });

    std::cout << "[nav] scenario loaded: " << scenePath_ << std::endl;
    std::cout << "[nav] episode=" << episodeDir_ << " goal=(" << cfg_.goalX << "," << cfg_.goalY
              << "," << cfg_.goalZ << ") path_points=" << cfg_.path.size()
              << " obstacles=" << cfg_.obstacles.size() << std::endl;
}

void NavSimulationManager::SimulationStepCompleted(sf::Scalar)
{
    // Deliberately empty: actuator commands and recording run on the control thread,
    // outside the physics step callback.
}

void NavSimulationManager::startControl()
{
    if (controlStarted_.exchange(true))
        return;
    controlThread_ = std::thread(&NavSimulationManager::controlLoop, this);
}

void NavSimulationManager::shutdown()
{
    stopRequested_ = true;
    if (controlThread_.joinable())
        controlThread_.join();
}

void NavSimulationManager::setPwm(const std::array<double, 8>& pwm)
{
    for (int i = 0; i < 8; ++i)
        thrusters_[i]->setSetpoint(pwm[i]);
}

void NavSimulationManager::readSensors(RobotState& state, std::vector<double>& sonar,
                                       double& dvlAltitude, double& pressure,
                                       std::array<double, 3>& imuAv,
                                       std::array<double, 3>& imuLa)
{
    std::vector<double> odom;
    if (!readLast(odometry_, odom) || odom.size() < 13)
        return;

    state.position = {odom[0], odom[1], odom[2]};
    state.velocity = {odom[3], odom[4], odom[5]};
    state.orientation = {odom[6], odom[7], odom[8], odom[9]};
    state.angularRate = {odom[10], odom[11], odom[12]};

    const double x = state.orientation[0];
    const double y = state.orientation[1];
    const double z = state.orientation[2];
    const double w = state.orientation[3];
    state.roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    state.pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    state.yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    std::vector<double> imu;
    if (readLast(imu_, imu) && imu.size() >= 9)
    {
        imuAv = {imu[3], imu[4], imu[5]};
        imuLa = {imu[6], imu[7], imu[8]};
    }

    std::vector<double> dvl;
    if (readLast(dvl_, dvl) && dvl.size() >= 4)
        dvlAltitude = dvl[3];

    std::vector<double> press;
    if (readLast(pressure_, press) && !press.empty())
        pressure = press[0];

    readLast(multibeam_, sonar);
}

bool NavSimulationManager::checkCollision(double x, double y, double z, double& minClearance) const
{
    minClearance = 1e9;
    bool collision = false;

    for (const auto& obs : cfg_.obstacles)
    {
        const double d = std::hypot(x - obs.x, y - obs.y);
        const double clearance = d - obs.radius;
        minClearance = std::min(minClearance, clearance);
        const bool verticalOverlap = (z + 0.30 >= obs.zmin) && (z - 0.30 <= obs.zmax);
        if (verticalOverlap && clearance < 0.27)
            collision = true;
    }

    if (z > kSeabedZ - 0.25)
        collision = true;
    if (z < 0.15)
        collision = true;

    return collision;
}

void NavSimulationManager::processSample()
{
    const double simTime = getSimulationTime();
    if (lastSampleTime_ >= 0.0 && simTime - lastSampleTime_ < 0.045)
        return;

    RobotState robotState;
    std::vector<double> sonar;
    double dvlAltitude = 0.0;
    double pressurePa = 0.0;
    std::array<double, 3> imuAv{};
    std::array<double, 3> imuLa{};
    readSensors(robotState, sonar, dvlAltitude, pressurePa, imuAv, imuLa);
    if (lastSampleTime_ < 0.0 && robotState.position[0] == 0.0 && robotState.position[1] == 0.0 &&
        robotState.position[2] == 0.0)
        return;

    const double dt = lastSampleTime_ >= 0.0 ? std::clamp(simTime - lastSampleTime_, 0.02, 0.5) : 0.1;
    lastSampleTime_ = simTime;

    // Collect the observation first; the control source is either the scripted autopilot
    // or an external policy connected over PolicyBridge.
    std::array<double, 8> pwm{};
    std::array<float, 6> targetLocal{};
    double goalDistance = 0.0;
    double minSonar = 1e9;
    double goalBearing = 0.0;
    double routeDistance = 0.0;
    double routeBearing = 0.0;
    bool avoidanceActive = false;

    if (bridge_)
    {
        std::array<double, 8> cmd{};
        if (bridge_->popCommand(cmd))
            policyPwm_ = cmd;
        pwm = policyPwm_;
        setPwm(pwm);
        const double dx = cfg_.goalX - robotState.position[0];
        const double dy = cfg_.goalY - robotState.position[1];
        const double dz = cfg_.goalZ - robotState.position[2];
        goalDistance = std::sqrt(dx * dx + dy * dy + dz * dz);
        goalBearing = wrapAngle(std::atan2(dy, dx) - robotState.yaw);

        // Global-planner style look-ahead: walk 5 m along the planned route from the closest
        // path point. The route bearing stays small, like the expert's own steering signal.
        const auto& path = cfg_.path;
        if (path.size() >= 2)
        {
            size_t best = 0;
            double bestd = 1e18;
            for (size_t i = 0; i < path.size(); ++i)
            {
                const double d = std::hypot(path[i].x - robotState.position[0],
                                            path[i].y - robotState.position[1]);
                if (d < bestd)
                {
                    bestd = d;
                    best = i;
                }
            }
            size_t i = best;
            double acc = 0.0;
            while (i + 1 < path.size() && acc < 5.0)
            {
                acc += std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
                ++i;
            }
            const double rdx = path[i].x - robotState.position[0];
            const double rdy = path[i].y - robotState.position[1];
            routeDistance = std::hypot(rdx, rdy);
            routeBearing = wrapAngle(std::atan2(rdy, rdx) - robotState.yaw);
        }
    }
    else
    {
        auto output = autopilot_->step(robotState, sonar, dt);
        pwm = output.pwm;
        setPwm(pwm);
        goalDistance = output.goalDistance;
        minSonar = output.minSonar;
        avoidanceActive = output.avoidanceActive;
        for (int i = 0; i < 6; ++i)
            targetLocal[i] = static_cast<float>(output.targetLocal[i]);
    }

    std::array<float, 29> state{};
    for (int i = 0; i < 8; ++i)
        state[5 + i] = static_cast<float>(pwm[i]);
    state[18] = static_cast<float>(robotState.velocity[0]);
    state[19] = static_cast<float>(robotState.velocity[1]);
    state[20] = static_cast<float>(robotState.velocity[2]);
    state[21] = static_cast<float>(imuAv[0]);
    state[22] = static_cast<float>(imuAv[1]);
    state[23] = static_cast<float>(imuAv[2]);
    state[24] = static_cast<float>(imuLa[0]);
    state[25] = static_cast<float>(imuLa[1]);
    state[26] = static_cast<float>(imuLa[2]);
    state[27] = static_cast<float>(pressurePa / 10000.0);
    state[28] = static_cast<float>(dvlAltitude);

    std::array<float, 13> action{};
    for (int i = 0; i < 8; ++i)
        action[5 + i] = static_cast<float>(pwm[i]);

    // The dataset state uses DVL velocity, not odometry velocity, for dvl_v.
    std::vector<double> dvl;
    if (readLast(dvl_, dvl) && dvl.size() >= 4)
    {
        state[18] = static_cast<float>(dvl[0]);
        state[19] = static_cast<float>(dvl[1]);
        state[20] = static_cast<float>(dvl[2]);
        state[28] = static_cast<float>(dvl[3]);
    }

    cv::Mat ego;
    cv::Mat wrist;
    {
        std::lock_guard<std::mutex> lock(imageMutex_);
        ego = egoImage_.clone();
        wrist = wristImage_.clone();
    }

    if ((ego.empty() || wrist.empty()) && frameIndex_ == 0)
        return;

    if (bridge_)
        bridge_->sendFrame(state, ego, wrist, goalDistance, robotState.position, goalBearing,
                           routeDistance, routeBearing);

    double minClearance = 1e9;
    const bool collision =
        checkCollision(robotState.position[0], robotState.position[1], robotState.position[2], minClearance);
    minClearance_ = std::min(minClearance_, minClearance);

    recorder_->addFrame(frameIndex_, simTime, state, action, targetLocal, robotState.position,
                        dvlAltitude, goalDistance, minSonar, avoidanceActive, robotState.yaw, ego,
                        wrist);

    if (frameIndex_ % 10 == 0 || simTime - lastPrintTime_ > 2.0)
    {
        lastPrintTime_ = simTime;
        std::printf(
            "[nav] t=%6.2f pos=(%6.2f,%6.2f,%5.2f) goal=%5.2f v=(%5.2f,%5.2f,%5.2f) sonar=%5.2f "
            "avoid=%d pwm=(%5.2f,%5.2f,%5.2f,%5.2f)\n",
            simTime, robotState.position[0], robotState.position[1], robotState.position[2],
            goalDistance, robotState.velocity[0], robotState.velocity[1], robotState.velocity[2],
            minSonar, avoidanceActive ? 1 : 0, pwm[0], pwm[1], pwm[2],
            pwm[3]);
        std::fflush(stdout);
    }

    ++frameIndex_;

    const double yawError = wrapAngle(cfg_.goalYaw - robotState.yaw);
    const double speed = std::sqrt(robotState.velocity[0] * robotState.velocity[0] +
                                   robotState.velocity[1] * robotState.velocity[1] +
                                   robotState.velocity[2] * robotState.velocity[2]);

    if (goalDistance < cfg_.successRadius && std::fabs(yawError) < cfg_.successYaw && speed < 0.55)
        ++goalHold_;
    else
        goalHold_ = 0;

    bool success = false;
    std::string reason;
    if (collision)
    {
        reason = "collision";
    }
    else if (goalHold_ >= 2)
    {
        success = true;
        reason = "success";
    }
    else if (simTime > cfg_.maxTime)
    {
        reason = "timeout";
    }
    else if (frameIndex_ > 20 && ego.empty() && wrist.empty())
    {
        reason = "no_camera_images";
    }

    if (!reason.empty())
    {
        std::array<double, 8> zero{};
        setPwm(zero);
        recorder_->finalize(success, reason, simTime, goalDistance, minClearance_);
        std::printf("[nav] episode finished: success=%d reason=%s frames=%d t=%.2f goal=%.3f clearance=%.3f\n",
                    success ? 1 : 0, reason.c_str(), recorder_->frames(), simTime, goalDistance,
                    minClearance_);
        std::fflush(stdout);
        finished_ = true;
    }
}

void NavSimulationManager::controlLoop()
{
    // Let sensors and the rendering pipeline produce the first samples.
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    if (thrustTest_)
    {
        thrustTestLoop();
        return;
    }

    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    while (!stopRequested_.load() && !finished_.load())
    {
        next += std::chrono::milliseconds(100);
        std::this_thread::sleep_until(next);
        if (sf::SimulationApp::getApp()->getState() != sf::SimulationState::RUNNING)
            break;
        processSample();
    }
    std::array<double, 8> zero{};
    setPwm(zero);
}

void NavSimulationManager::thrustTestLoop()
{
    std::array<double, 8> forward{-0.85, -0.85, 0.85, 0.85, 0.0, 0.0, 0.0, 0.0};
    setPwm(forward);
    std::this_thread::sleep_for(std::chrono::seconds(4));
    RobotState state;
    std::vector<double> sonar;
    double alt = 0.0;
    double pressure = 0.0;
    std::array<double, 3> av{};
    std::array<double, 3> la{};
    readSensors(state, sonar, alt, pressure, av, la);
    std::array<double, 8> zero{};
    setPwm(zero);
    std::printf("[thrust-test] pos=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f)\n", state.position[0],
                state.position[1], state.position[2], state.velocity[0], state.velocity[1],
                state.velocity[2]);
    std::fflush(stdout);
    finished_ = true;
}
}  // namespace uuv
