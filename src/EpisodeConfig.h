#pragma once

#include <array>
#include <string>
#include <vector>

namespace uuv
{
struct Waypoint
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
};

struct Obstacle
{
    double x = 0.0;
    double y = 0.0;
    double radius = 1.0;
    double zmin = 0.0;
    double zmax = 0.0;
    std::string type;
};

struct EpisodeConfig
{
    std::string task = "Go to the charge station";
    std::string split = "train";
    std::string episodeName;
    int seed = 0;
    int episodeIndex = 0;

    double startX = 0.0;
    double startY = 0.0;
    double startZ = 1.2;
    double startYaw = 0.0;

    double goalX = 0.0;
    double goalY = 0.0;
    double goalZ = 1.2;
    double goalYaw = 0.0;

    double depth = 1.2;
    double successRadius = 0.6;
    double successYaw = 0.40;
    double maxTime = 150.0;

    std::vector<Waypoint> path;
    std::vector<Obstacle> obstacles;
};

bool loadEpisode(const std::string& episodeDir, EpisodeConfig& cfg, std::string& error);
}  // namespace uuv
