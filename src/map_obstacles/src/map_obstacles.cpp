// Implementation of the offline map obstacles: CSV loading and the closed-shape check.
#include "map_obstacles/map_obstacles.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace map_obstacles {

MapObstacles loadMapObstacles(const std::string& path, const double horizontal_tol_deg) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("cannot open map obstacles file " + path);
    }
    std::string line;
    if (!std::getline(file, line) || line.rfind("nx,ny,nz,offset,obstacle", 0) != 0) {
        throw std::runtime_error(path + ":1: expected the header nx,ny,nz,offset,obstacle");
    }

    MapObstacles map_obstacles;
    std::unordered_map<int, std::size_t> index_of_id;  // obstacle ID -> index in map_obstacles.map_obstacle
    for (int row = 2; std::getline(file, line); ++row) {
        const std::size_t first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] == '#') {
            continue;
        }
        std::istringstream fields(line);
        double value[4];
        int id;
        char c[4];
        if (!(fields >> value[0] >> c[0] >> value[1] >> c[1] >> value[2] >> c[2] >> value[3] >> c[3] >> id) ||
            c[0] != ',' || c[1] != ',' || c[2] != ',' || c[3] != ',' ||
            (fields >> std::ws, !fields.eof())) {
            throw std::runtime_error(path + ":" + std::to_string(row) +
                                     ": expected 4 numbers + 1 integer nx,ny,nz,offset,obstacle");
        }
        const Eigen::Vector3d normal(value[0], value[1], value[2]);
        const double length = normal.norm();
        if (length < 1e-9) {
            throw std::runtime_error(path + ":" + std::to_string(row) + ": the normal is zero");
        }

        const auto [entry, is_new] = index_of_id.try_emplace(id, map_obstacles.map_obstacle.size());
        if (is_new) {
            map_obstacles.map_obstacle.push_back({id, {}});
        }
        map_obstacles.map_obstacle[entry->second].planes.push_back({normal / length, value[3] / length});
    }

    const std::vector<int> removed = obstacleFeasibilityCheck(map_obstacles, horizontal_tol_deg);
    if (!removed.empty()) {
        std::cerr << path << ": " << removed.size() << " obstacle(s) are not closed and were removed, IDs:";
        for (const int id : removed) {
            std::cerr << ' ' << id;
        }
        std::cerr << '\n';
    }
    return map_obstacles;
}

bool isClosedObstacle(const MapObstacle& obstacle, const double horizontal_tol_deg) {
    const double tol = horizontal_tol_deg * M_PI / 180.0;
    const double cos_tol = std::cos(tol);
    bool has_top = false;
    bool has_bottom = false;
    std::vector<double> side_angles;  // direction of each side normal seen from above [rad]
    for (const HalfPlane& hp : obstacle.planes) {
        const Eigen::Vector3d& n = hp.normal;  // unit length, out of the obstacle
        if (n.z() >= cos_tol) {
            has_top = true;
        } else if (n.z() <= -cos_tol) {
            has_bottom = true;
        } else {
            side_angles.push_back(std::atan2(n.y(), n.x()));  // horizontal part >= sin(tol) > 0
        }
    }
    if (!has_top || !has_bottom || side_angles.size() < 3) {
        return false;
    }

    // closed from the side iff the side normals surround the obstacle: no angular gap of 180 deg or more between
    // neighbouring directions (otherwise the obstacle is open towards the middle of that gap). This also needs at
    // least 3 non-parallel sides.
    std::sort(side_angles.begin(), side_angles.end());
    double max_gap = side_angles.front() + 2.0 * M_PI - side_angles.back();  // wrap-around gap
    for (std::size_t i = 1; i < side_angles.size(); ++i) {
        max_gap = std::max(max_gap, side_angles[i] - side_angles[i - 1]);
    }
    return max_gap < M_PI - tol;
}

std::vector<int> obstacleFeasibilityCheck(MapObstacles& map_obstacles, const double horizontal_tol_deg) {
    std::vector<int> removed;
    auto& obstacles = map_obstacles.map_obstacle;
    obstacles.erase(std::remove_if(obstacles.begin(), obstacles.end(),
                                   [&](const MapObstacle& obs) {
                                       if (isClosedObstacle(obs, horizontal_tol_deg)) {
                                           return false;
                                       }
                                       removed.push_back(obs.ID);
                                       return true;
                                   }),
                    obstacles.end());
    return removed;
}

}  // namespace map_obstacles
