#include "clustering/clustering.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <optional>

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

bool insideObstacle(const MapObstacle& obstacle, const Eigen::Vector3d& p, const double eps) {
    return insideObstacle(obstacle.planes, p, eps);
}

bool insideObstacle(const LidarObstacle& obstacle, const Eigen::Vector3d& p, const double eps) {
    return insideObstacle(obstacle.planes, p, eps);
}

namespace {

// the same loop for both obstacle vectors (std::vector<MapObstacle>, std::vector<LidarObstacle>)
template <typename ObstacleT>
bool insideAny(const std::vector<ObstacleT>& obstacles, const Eigen::Vector3d& p, const double eps) {
    for (const ObstacleT& obstacle : obstacles) {
        if (insideObstacle(obstacle, p, eps)) {
            return true;
        }
    }
    return false;
}

}  // namespace

bool insideAnyObstacle(const MapObstacles& obstacles, const Eigen::Vector3d& p, const double eps) {
    return insideAny(obstacles.map_obstacle, p, eps);
}

bool insideAnyObstacle(const LidarObstacles& obstacles, const Eigen::Vector3d& p, const double eps) {
    return insideAny(obstacles.lidar_obstacles, p, eps);
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

    const std::vector<int> removed = obstacleFeasibilityCheck(map_obstacles);
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
    const double cos_tol = std::cos(horizontal_tol_deg * M_PI / 180.0);
    bool has_top = false;
    bool has_bottom = false;
    std::vector<double> side_angles;  // direction of each side normal seen from above [rad]
    for (const HalfPlane& hp : obstacle.planes) {
        const Eigen::Vector3d& n = hp.normal;  // unit length (loadMapObstacles), out of the obstacle
        if (n.z() >= cos_tol) {
            has_top = true;
        } else if (n.z() <= -cos_tol) {
            has_bottom = true;
        } else {
            side_angles.push_back(std::atan2(n.y(), n.x()));
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
    return max_gap < M_PI - horizontal_tol_deg * M_PI / 180.0;
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

// DBSCAN CLUSTERING
constexpr int UNDEFINED = -1;  // not visited yet
constexpr int NOISE = 0;

// The label of each point: NOISE (0) or a cluster id (1, 2, ...).
// eps [m]: neighbourhood radius. min_points: neighbours a point needs to be a core point, itself included.
std::vector<int> dbscan_fast(const std::vector<Eigen::Vector3d>& points, const float eps, const std::size_t min_points) {
    if (!(eps > 0.0f)) {
        throw std::invalid_argument("dbscan: eps must be positive");
    }
    const clustering::VoxelGrid grid(points, eps);
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

// BOX FIT

std::vector<Eigen::Vector2d> convexHull2d(std::vector<Eigen::Vector2d> points) {
    // > 0: o -> a -> b turns left (counter-clockwise), < 0: right, 0: collinear
    auto cross = [](const Eigen::Vector2d& o, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    };
    // sort with increasing x (increasing y if same x)
    std::sort(points.begin(), points.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    points.erase(std::unique(points.begin(), points.end()), points.end());
    if (points.size() < 3) {
        return points;
    }

    std::vector<Eigen::Vector2d> hull(2 * points.size());
    std::size_t k = 0;
    // lower hull, left -> right: drop the last point while it does not make a left turn
    for (std::size_t i = 0; i < points.size(); ++i) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], points[i]) <= 0) {
            --k;
        }
        hull[k++] = points[i];
    }
    // upper hull, right -> left (the last point of the lower hull is its first point)
    const std::size_t lower = k + 1;
    for (std::size_t i = points.size() - 1; i-- > 0;) {
        while (k >= lower && cross(hull[k - 2], hull[k - 1], points[i]) <= 0) {
            --k;
        }
        hull[k++] = points[i];
    }
    hull.resize(k - 1);  // the last point is the first one again
    return hull;
}

std::vector<HalfPlane> fitAlignedBox(const std::vector<Eigen::Vector3d>& points, const double min_size) {
    if (points.empty()) {
        throw std::invalid_argument("fitAlignedBox: no points");
    }
    const double inf = std::numeric_limits<double>::infinity();

    std::vector<Eigen::Vector2d> xy;
    xy.reserve(points.size());
    double z_min = inf, z_max = -inf;
    for (const auto& p : points) {
        xy.push_back(p.head<2>());
        z_min = std::min(z_min, p.z());
        z_max = std::max(z_max, p.z());
    }
    const std::vector<Eigen::Vector2d> hull = convexHull2d(std::move(xy));

    // extent of the hull along u and the perpendicular w = u rotated by +90 deg
    struct Frame {
        Eigen::Vector2d u, w;
        double u_min, u_max, w_min, w_max;
    };
    auto frame = [&hull, inf](const Eigen::Vector2d& u) {
        Frame f{u, Eigen::Vector2d(-u.y(), u.x()), inf, -inf, inf, -inf};
        for (const auto& q : hull) {
            f.u_min = std::min(f.u_min, f.u.dot(q));  f.u_max = std::max(f.u_max, f.u.dot(q));
            f.w_min = std::min(f.w_min, f.w.dot(q));  f.w_max = std::max(f.w_max, f.w.dot(q));
        }
        return f;
    };

    // the smallest rectangle around a convex polygon has one side along one of its edges: try every edge
    Frame best = frame(Eigen::Vector2d::UnitX());  // fallback for a single point
    double best_area = inf;
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const Eigen::Vector2d edge = hull[(i + 1) % hull.size()] - hull[i];
        if (edge.norm() < 1e-9) {
            continue;
        }
        const Frame f = frame(edge.normalized());
        const double area = (f.u_max - f.u_min) * (f.w_max - f.w_min);
        if (area < best_area) {
            best_area = area;
            best = f;
        }
    }

    // flat clusters (a wall, a table top) and single points get at least min_size per side
    auto widen = [min_size](double& lo, double& hi) {
        const double grow = std::max(0.0, min_size - (hi - lo)) / 2.0;
        lo -= grow;
        hi += grow;
    };
    widen(best.u_min, best.u_max);
    widen(best.w_min, best.w_max);
    widen(z_min, z_max);

    // free side n·p + offset >= 0: the face at s = s_max has n = +axis, offset = -s_max,
    // the face at s = s_min has n = -axis, offset = +s_min
    const Eigen::Vector3d U(best.u.x(), best.u.y(), 0.0);
    const Eigen::Vector3d W(best.w.x(), best.w.y(), 0.0);
    const Eigen::Vector3d Z = Eigen::Vector3d::UnitZ();
    std::vector<HalfPlane> box;
    box.reserve(6);
    box.push_back(HalfPlane{U, -best.u_max});   // side at u_max
    box.push_back(HalfPlane{-U, best.u_min});   // side at u_min
    box.push_back(HalfPlane{W, -best.w_max});   // side at w_max
    box.push_back(HalfPlane{-W, best.w_min});   // side at w_min
    box.push_back(HalfPlane{Z, -z_max});        // top
    box.push_back(HalfPlane{-Z, z_min});        // bottom
    return box;
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