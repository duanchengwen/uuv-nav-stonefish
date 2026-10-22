#include <cstdlib>
#include <iostream>
#include <string>

#include <core/GraphicalSimulationApp.h>
#include <core/SimulationApp.h>

#include "NavSimulationManager.h"

namespace
{
const std::string kDataPath = std::string(DATA_DIR_PATH);

class NavApp : public sf::GraphicalSimulationApp
{
public:
    NavApp(const std::string& title, const std::string& dataPath, const sf::RenderSettings& rs,
           const sf::HelperSettings& hs, uuv::NavSimulationManager* manager)
        : sf::GraphicalSimulationApp(title, dataPath, rs, hs, manager), manager_(manager)
    {
    }

    void StartSimulation() override
    {
        sf::GraphicalSimulationApp::StartSimulation();
        manager_->startControl();
    }

    void ProcessInputs() override
    {
        if (manager_ != nullptr && manager_->isFinished())
        {
            StopSimulation();
            Quit();
        }
    }

private:
    uuv::NavSimulationManager* manager_ = nullptr;
};
}  // namespace

int main(int argc, char** argv)
{
    std::string scenePath;
    std::string episodeDir;
    bool thrustTest = false;
    bool showCamera = false;
    int policyPort = 0;
    bool directExpert = false;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--scene" && i + 1 < argc)
            scenePath = argv[++i];
        else if (arg == "--episode-dir" && i + 1 < argc)
            episodeDir = argv[++i];
        else if (arg == "--thrust-test")
            thrustTest = true;
        else if (arg == "--show-camera")
            showCamera = true;
        else if (arg == "--policy-bridge" && i + 1 < argc)
            policyPort = std::stoi(argv[++i]);
        else if (arg == "--direct-expert")
            directExpert = true;
        else if (arg == "--help" || arg == "-h")
        {
            std::cout << "Usage: uuv_nav --scene <scene.scn> --episode-dir <dir> [--thrust-test] "
                         "[--show-camera] [--policy-bridge <port>]\n";
            return 0;
        }
        else
        {
            std::cerr << "unknown argument: " << arg << std::endl;
            return 2;
        }
    }

    if (scenePath.empty() || episodeDir.empty())
    {
        std::cerr << "Usage: uuv_nav --scene <scene.scn> --episode-dir <dir> [--thrust-test] "
                     "[--show-camera] [--policy-bridge <port>]\n";
        return 2;
    }
    if (scenePath.front() != '/')
        scenePath = kDataPath + scenePath;

    sf::RenderSettings rs;
    rs.windowW = 1200;
    rs.windowH = 900;
    rs.aa = sf::RenderQuality::LOW;
    rs.shadows = sf::RenderQuality::MEDIUM;
    rs.ao = sf::RenderQuality::LOW;
    rs.atmosphere = sf::RenderQuality::LOW;
    rs.ocean = sf::RenderQuality::MEDIUM;
    rs.ssr = sf::RenderQuality::LOW;
    rs.verticalSync = false;

    sf::HelperSettings hs;
    hs.showFluidDynamics = false;
    hs.showCoordSys = false;
    hs.showBulletDebugInfo = false;
    hs.showSensors = false;
    hs.showActuators = false;
    hs.showForces = false;
    hs.showOceanVelocityField = false;
    hs.showJoints = false;

    uuv::NavSimulationManager* manager = nullptr;
    try
    {
        manager = new uuv::NavSimulationManager(500.0, scenePath, episodeDir, thrustTest, showCamera,
                                                policyPort, directExpert);
    }
    catch (const std::exception& e)
    {
        std::cerr << "initialization failed: " << e.what() << std::endl;
        return 1;
    }

    {
        NavApp app("USIM UUV charge navigation", kDataPath, rs, hs, manager);
        app.Run(true, true, 0);
        // Stop the control thread while the Stonefish app (and its sensors/actuators)
        // is still alive.
        manager->shutdown();
    }

    // The simulation manager is intentionally leaked: Stonefish releases OpenGL
    // resources in its destructor after the application has already destroyed the
    // GL context, which crashes on some drivers. The process exits immediately.
    return 0;
}
