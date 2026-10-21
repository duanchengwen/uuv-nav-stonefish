#include "EpisodeConfig.h"

#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>

namespace uuv
{
namespace
{
std::string trim(const std::string& s)
{
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
        return "";
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

std::vector<std::string> splitCsv(const std::string& line)
{
    std::vector<std::string> values;
    std::stringstream ss(line);
    std::string value;
    while (std::getline(ss, value, ','))
        values.push_back(trim(value));
    return values;
}

bool readKeyValues(const std::string& path, std::map<std::string, std::string>& out)
{
    std::ifstream file(path);
    if (!file)
        return false;

    std::string line;
    while (std::getline(file, line))
    {
        line = trim(line);
        if (line.empty() || line[0] == '#')
            continue;
        const auto eq = line.find('=');
        if (eq == std::string::npos)
            continue;
        out[trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
    }
    return true;
}

double getDouble(const std::map<std::string, std::string>& kv, const std::string& key, double fallback)
{
    const auto it = kv.find(key);
    if (it == kv.end())
        return fallback;
    try
    {
        return std::stod(it->second);
    }
    catch (...)
    {
        return fallback;
    }
}

std::string getString(const std::map<std::string, std::string>& kv, const std::string& key, const std::string& fallback)
{
    const auto it = kv.find(key);
    return it == kv.end() ? fallback : it->second;
}
}  // namespace

bool loadEpisode(const std::string& episodeDir, EpisodeConfig& cfg, std::string& error)
{
    const std::string base = episodeDir.empty() ? "" : episodeDir + "/";
    std::map<std::string, std::string> kv;
    if (!readKeyValues(base + "episode.cfg", kv))
    {
        error = "cannot read " + base + "episode.cfg";
        return false;
    }

    cfg.task = getString(kv, "task", cfg.task);
    cfg.split = getString(kv, "split", cfg.split);
    cfg.episodeName = getString(kv, "episode_name", cfg.episodeName);
    cfg.seed = static_cast<int>(getDouble(kv, "seed", 0));
    cfg.episodeIndex = static_cast<int>(getDouble(kv, "episode_index", 0));

    cfg.startX = getDouble(kv, "start_x", cfg.startX);
    cfg.startY = getDouble(kv, "start_y", cfg.startY);
    cfg.startZ = getDouble(kv, "start_z", cfg.startZ);
    cfg.startYaw = getDouble(kv, "start_yaw", cfg.startYaw);

    cfg.goalX = getDouble(kv, "goal_x", cfg.goalX);
    cfg.goalY = getDouble(kv, "goal_y", cfg.goalY);
    cfg.goalZ = getDouble(kv, "goal_z", cfg.goalZ);
    cfg.goalYaw = getDouble(kv, "goal_yaw", cfg.goalYaw);

    cfg.depth = getDouble(kv, "depth", cfg.depth);
    cfg.successRadius = getDouble(kv, "success_radius", cfg.successRadius);
    cfg.successYaw = getDouble(kv, "success_yaw", cfg.successYaw);
    cfg.maxTime = getDouble(kv, "max_time", cfg.maxTime);

    // Path: x,y,z,yaw (one waypoint per line, optional header)
    {
        std::ifstream file(base + "path.csv");
        if (!file)
        {
            error = "cannot read " + base + "path.csv";
            return false;
        }
        std::string line;
        while (std::getline(file, line))
        {
            line = trim(line);
            if (line.empty() || line[0] == '#')
                continue;
            if (line.find("x") != std::string::npos && line.find(",") != std::string::npos &&
                line.find_first_of("0123456789") == std::string::npos)
                continue;
            const auto values = splitCsv(line);
            if (values.size() < 4)
                continue;
            try
            {
                cfg.path.push_back({std::stod(values[0]), std::stod(values[1]), std::stod(values[2]),
                                    std::stod(values[3])});
            }
            catch (...)
            {
                continue;
            }
        }
    }

    // Obstacles: x,y,radius,zmin,zmax,type
    {
        std::ifstream file(base + "obstacles.csv");
        if (file)
        {
            std::string line;
            while (std::getline(file, line))
            {
                line = trim(line);
                if (line.empty() || line[0] == '#')
                    continue;
                if (line.find("radius") != std::string::npos)
                    continue;
                const auto values = splitCsv(line);
                if (values.size() < 5)
                    continue;
                try
                {
                    Obstacle o;
                    o.x = std::stod(values[0]);
                    o.y = std::stod(values[1]);
                    o.radius = std::stod(values[2]);
                    o.zmin = std::stod(values[3]);
                    o.zmax = std::stod(values[4]);
                    o.type = values.size() > 5 ? values[5] : "obstacle";
                    cfg.obstacles.push_back(o);
                }
                catch (...)
                {
                    continue;
                }
            }
        }
    }

    if (cfg.path.size() < 2)
    {
        error = "path.csv contains fewer than 2 waypoints";
        return false;
    }
    return true;
}
}  // namespace uuv
