#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include <core/SimulationManager.h>

#include "Autopilot.h"
#include "DataRecorder.h"
#include "EpisodeConfig.h"
#include "PolicyBridge.h"

namespace sf
{
class ColorCamera;
class DVL;
class IMU;
class Multibeam;
class Odometry;
class Pressure;
class Robot;
class Thruster;
}  // namespace sf

namespace uuv
{
class NavSimulationManager : public sf::SimulationManager
{
public:
    NavSimulationManager(sf::Scalar stepsPerSecond, std::string scenePath, std::string episodeDir,
                         bool thrustTest = false, bool showCamera = false, int policyPort = 0,
                         bool directExpert = false);
    ~NavSimulationManager() override;

    void BuildScenario() override;
    void SimulationStepCompleted(sf::Scalar dt) override;

    void startControl();
    void shutdown();
    bool isFinished() const { return finished_.load(); }

private:
    void controlLoop();
    void thrustTestLoop();
    void processSample();
    void readSensors(RobotState& state, std::vector<double>& sonar, double& dvlAltitude,
                     double& pressure, std::array<double, 3>& imuAv, std::array<double, 3>& imuLa);
    bool checkCollision(double x, double y, double z, double& minClearance) const;
    void setPwm(const std::array<double, 8>& pwm);

    std::string scenePath_;
    std::string episodeDir_;
    bool thrustTest_ = false;
    bool showCamera_ = false;

    EpisodeConfig cfg_;
    std::unique_ptr<Autopilot> autopilot_;
    std::unique_ptr<DataRecorder> recorder_;
    std::unique_ptr<PolicyBridge> bridge_;  // non-null when an external policy drives the ROV
    std::array<double, 8> policyPwm_{};

    sf::Robot* robot_ = nullptr;
    std::array<sf::Thruster*, 8> thrusters_{};
    sf::ColorCamera* egoCamera_ = nullptr;
    sf::ColorCamera* wristCamera_ = nullptr;
    sf::Odometry* odometry_ = nullptr;
    sf::IMU* imu_ = nullptr;
    sf::DVL* dvl_ = nullptr;
    sf::Pressure* pressure_ = nullptr;
    sf::Multibeam* multibeam_ = nullptr;

    std::mutex imageMutex_;
    cv::Mat egoImage_;
    cv::Mat wristImage_;

    std::thread controlThread_;
    std::atomic<bool> stopRequested_{false};
    std::atomic<bool> finished_{false};
    std::atomic<bool> controlStarted_{false};

    int frameIndex_ = 0;
    int goalHold_ = 0;
    double minClearance_ = 1e9;
    double lastPrintTime_ = -1.0;
    double lastSampleTime_ = -1.0;
};
}  // namespace uuv
