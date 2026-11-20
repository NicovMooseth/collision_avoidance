#include "clustering.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace clustering {

// Is point p inside the convex region of the half-planes?
// The region is the intersection of the obstacle sides of its half-planes (HalfPlane convention, constraints.hpp):
// normals point out of the obstacle towards free space, free side normal.dot(x) + offset >= 0.
// eps [m] inflates the region: points up to eps outside it count as inside.
bool insideObstacle(const std::vector<HalfPlane>& planes, const Eigen::Vector3d& p, const double eps) {
    for (const auto& hp : planes)
        if (hp.normal.dot(p) + hp.offset > eps) // on the free side of one plane -> outside the obstacle
            return false;
    return true;
}

bool insideAnyObstacle(const Obstacles& obstacles, const Eigen::Vector3d& point) {
    for (const auto& obs : obstacles.obstacles) 
        if (insideObstacle(obs, point))
            return true;
    return false;
}

void removeKnownPoints(const Obstacles& obstacles, std::vector<Eigen::Vector3d>& points) {
    points.erase(std::remove_if(points.begin(), points.end(),
                                [&](const Eigen::Vector3d& p) {
                                    return insideAnyObstacle(obstacles, p);
                                }),
                points.end());
}

/// @brief Read the obstacles of the offline map from a CSV file: header line "nx,ny,nz,offset,obstacle", then one
///        half-plane per row, obstacle = integer ID of the convex obstacle the plane belongs to. Free side
///        n·p + offset >= 0 (map frame, n facing away from the obstacle). The normal is scaled to unit length
///        (offset with it). Rows of one obstacle need not be contiguous; obstacles are returned in the order their
///        ID first appears. Empty lines are skipped.
/// @throws std::runtime_error if the file cannot be opened or a row is not 4 numbers + 1 integer / has a zero normal
MapObstacles loadMapObstacles(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("cannot open map obstacles file " + path);
    }
    MapObstacles map_obstacles;
    std::unordered_map<int, std::size_t> index_of_id;  // obstacle ID -> index in map_obstacles.map_obstacle
    std::string line;
    std::getline(file, line);  // header
    for (int row = 2; std::getline(file, line); ++row) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) {
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
    return map_obstacles;
}


// DBSCAN CLUSTERING
constexpr int UNDEFINED = -1;  // not visited yet
constexpr int NOISE = 0;

// The label of each point: NOISE (0) or a cluster id (1, 2, ...).
// eps [m]: neighbourhood radius. min_points: neighbours a point needs to be a core point, itself included.
std::vector<int> dbscan_fast(const std::vector<Eigen::Vector3d>& points, const float eps, const std::size_t min_points) {
    if (!(eps > 0.0f)) {
        throw std::invalid_argument("dbscan: eps must be positive");
    }
    const collision_avoidance::VoxelGrid grid(points, eps);
    std::vector<int> labels(points.size(), UNDEFINED);
    std::vector<int> neighbours;
    std::vector<int> to_visit;  // labelled points of this cluster whose neighbours are not checked yet
    int cluster = NOISE;        // the last cluster id used

    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        if (labels[i] != UNDEFINED) {
            continue;
        }
        grid.find_neighbours(i, neighbours);
        if (neighbours.size() < min_points) {
            labels[i] = NOISE;  // may still become a border point of a later cluster
            continue;
        }
        ++cluster;
        labels[i] = cluster;
        to_visit.push_back(i);
        while (!to_visit.empty()) {
            const int point = to_visit.back();
            to_visit.pop_back();
            grid.find_neighbours(point, neighbours);
            if (neighbours.size() < min_points) {
                continue;  // a border point: the cluster ends here
            }
            for (const int neighbour : neighbours) {
                if (labels[neighbour] == UNDEFINED) {
                    to_visit.push_back(neighbour);  // labelled now, so each point is queued only once
                }
                if (labels[neighbour] == UNDEFINED || labels[neighbour] == NOISE) {
                    labels[neighbour] = cluster;
                }
            }
        }
    }
    return labels;
}

// Representing clusters as boxes (or spheres)
// Idea: fit a convex shape around known points
// Get center point of cluster
// Check what points of current scan were visible in previous scan
// For motion estimation (linear or static) only use the previously not occluded points

// 1. Convex shape per cluster
std::optional<LidarObstacle> clusterToObstacle(const std::vector<Eigen::Vetor3d>& points, const std::vector<int> labels) {
    if (!(labels.any() > 0)) {
        return; // no labels
    }
    LidarObstacle lid_obs;
    for (int label = 1; label <= labels; label++) {
        planes = fitConvexShape(points, label);
        lid_obs.ID = num_obs_lidar_past + label;
        lid_obs.from_map = false;
        lid_obs.planes = planes;
    }
    
}

// 2. Center point 

// 3. Averaging previous center points (tracking based on closest cluster)

// 4. Check for linear / static 


} // namespace clustering


// TESTING --------------------------------------------------------------------------------------------

namespace collision_avoidance {
Obstacle makeBox(int id, const Eigen::Vector3d& center, const Eigen::Vector3d& size) {
    const Eigen::Vector3d half = 0.5 * size;
    Obstacle box;
    box.ID = id;
    for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d n = Eigen::Vector3d::Zero();

        // normal out of the box, offset so that the face (center + half * n) lies on the plane
        n[axis] = 1.0;                                            // +axis face
        box.planes.push_back({n, -(n.dot(center) + half[axis])});

        n[axis] = -1.0;                                           // -axis face
        box.planes.push_back({n, -(n.dot(center) + half[axis])});
    }
    return box;
}

} 

int main() {
    using namespace collision_avoidance;

    Obstacles obs;
    obs.obstacles.push_back(makeBox(0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));

    // quick check
    auto inside = [](const Obstacle& o, const Eigen::Vector3d& p) {
        for (const auto& hp : o.planes)
            if (hp.normal.dot(p) + hp.offset > 0.0) return false;
        return true;
    };

    auto start = std::chrono::steady_clock::now();
    inside(obs.obstacles[0], {0.0, 0.0, 0.0});
    double elapsed = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
    std::cout << elapsed << " microseconds" << '\n';
    std::cout << inside(obs.obstacles[0], {0.0, 0.0, 0.0}) << "\n";   // 1 (center)

    start = std::chrono::steady_clock::now();
    inside(obs.obstacles[0], {0.4, -0.4, 0.4});
    elapsed = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
    std::cout << elapsed << " microseconds" << '\n';
    std::cout << inside(obs.obstacles[0], {0.4, -0.4, 0.4}) << "\n";  // 1

    start = std::chrono::steady_clock::now();
    inside(obs.obstacles[0], {0.6, 0.0, 0.0});
    elapsed = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
    std::cout << elapsed << " microseconds" << '\n';
    std::cout << inside(obs.obstacles[0], {0.6, 0.0, 0.0}) << "\n";   // 0
}