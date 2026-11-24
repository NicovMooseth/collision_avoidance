#include "clustering/clustering.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <optional>
#include <iostream>

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

void removeKnownPoints(const MapObstacles& obstacles, std::vector<Eigen::Vector3d>& points, const double margin) {
    int origin_size = points.size();
    std::cout << origin_size << " original size" << '\n';
    points.erase(std::remove_if(points.begin(), points.end(),
                                [&](const Eigen::Vector3d& p) {
                                    return insideAnyObstacle(obstacles, p, margin);
                                }),
                points.end());
    std::cout << origin_size - points.size() << " points removed" << '\n';
}

// DBSCAN CLUSTERING
constexpr int UNDEFINED = -1;  // not visited yet
constexpr int NOISE = 0;

// The label of each point: NOISE (0) or a cluster id (1, 2, ...).
// eps [m]: neighbourhood radius. min_points: neighbours a point needs to be a core point, itself included.
std::vector<int> dbscan_fast(const std::vector<Eigen::Vector3d>& points, const ClusteringParams& params) {
    if (!(params.eps > 0.0f)) {
        throw std::invalid_argument("dbscan: params.eps must be positive");
    }
    // the voxel grid works on float points (half the memory, enough for LiDAR precision)
    std::vector<Eigen::Vector3f> points_float;
    points_float.reserve(points.size());
    for (const Eigen::Vector3d& p : points) {
        points_float.push_back(p.cast<float>());
    }
    const clustering::VoxelGrid grid(points_float, static_cast<float>(params.eps));
    std::vector<int> labels(points.size(), UNDEFINED);
    std::vector<int> neighbours;
    std::vector<int> to_visit;  // labelled points of this cluster whose neighbours are not checked yet
    int cluster = NOISE;        // the last cluster id used

    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        if (labels[i] != UNDEFINED) {
            continue;
        }
        grid.find_neighbours(i, neighbours);
        if (neighbours.size() < params.min_points) {
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
            if (neighbours.size() < params.min_points) {
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

std::vector<HalfPlane> fitAlignedBox(const std::vector<Eigen::Vector3d>& points, const ClusteringParams& params) {
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
    auto widen = [&params](double& lo, double& hi) {
        const double grow = std::max(0.0, params.min_size - (hi - lo)) / 2.0;
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


// UPDATE LIDAR OBSTACLES

Eigen::Vector3d avgPoint(const std::vector<Eigen::Vector3d>& points) {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
        sum += p;
    }
    return sum / static_cast<double>(points.size());
}

void updateHistory(LidarObstacle& obstacle, const Measurement& measurement, const ClusteringParams& params) {
    const std::size_t max_measurements = std::max<std::size_t>(1, params.num_measurements);
    obstacle.history.push_back(measurement);
    while (obstacle.history.size() > max_measurements) {
        obstacle.history.pop_front();
    }
}

void updateTime(LidarObstacle& obstacle, const double stamp) {
    obstacle.last_seen = stamp;
}

void updatePlanes(LidarObstacle& obstacle, const std::vector<Eigen::Vector3d>& cluster_points,
                  const ClusteringParams& params) {
    obstacle.planes = fitAlignedBox(cluster_points, params);
}

// Least-squares line through the centers over time, c(t) = mean_center + v (t - mean_stamp). Uses every
// measurement of the history, so it is less noisy than (newest - oldest) / dt and handles irregular time steps
// (skipped scans, occlusions).
void updateMotion(LidarObstacle& obstacle, const ClusteringParams& params) {
    const auto& history = obstacle.history;

    // default: not enough information for a velocity -> static
    obstacle.v = Eigen::Vector3d::Zero();
    obstacle.motion = 0;

    const std::size_t min_measurements = std::max<std::size_t>(2, params.min_measurements);
    if (history.size() < min_measurements) {
        return;
    }

    // mean stamp and mean center of the history
    double mean_stamp = 0.0;
    Eigen::Vector3d mean_center = Eigen::Vector3d::Zero();
    for (const Measurement& measurement : history) {
        mean_stamp += measurement.stamp;
        mean_center += measurement.center;
    }
    mean_stamp /= static_cast<double>(history.size());
    mean_center /= static_cast<double>(history.size());

    // v = sum (t - mean_stamp)(c - mean_center) / sum (t - mean_stamp)^2
    //   = time_center_covariance_sum / time_variance_sum
    double time_variance_sum = 0.0;
    Eigen::Vector3d time_center_covariance_sum = Eigen::Vector3d::Zero();
    for (const Measurement& measurement : history) {
        const double time_offset = measurement.stamp - mean_stamp;
        time_variance_sum += time_offset * time_offset;
        time_center_covariance_sum += time_offset * (measurement.center - mean_center);
    }
    if (time_variance_sum < 1e-9) {  // all measurements at (nearly) the same time
        return;
    }
    const Eigen::Vector3d v = time_center_covariance_sum / time_variance_sum;

    // slow tracks are static: their velocity is mostly measurement noise of the center
    if (v.norm() > params.max_static_speed) {
        obstacle.v = v;
        obstacle.motion = 1;
    }
}

void updateLidarObs(LidarObstacle& obstacle, const std::vector<Eigen::Vector3d>& cluster_points, const double stamp,
                    const ClusteringParams& params) {
    if (cluster_points.empty()) {
        throw std::invalid_argument("updateLidarObs: no cluster points");
    }
    updateHistory(obstacle, Measurement{stamp, avgPoint(cluster_points)}, params);
    updateTime(obstacle, stamp);
    updatePlanes(obstacle, cluster_points, params);
    updateMotion(obstacle, params);
}

// ASSOCIATION

std::vector<std::vector<Eigen::Vector3d>> splitClusters(const std::vector<Eigen::Vector3d>& points,
                                                        const std::vector<int>& labels) {
    if (labels.size() != points.size()) {
        throw std::invalid_argument("splitClusters: one label per point needed");
    }
    const int num_clusters = labels.empty() ? 0 : *std::max_element(labels.begin(), labels.end());
    std::vector<std::vector<Eigen::Vector3d>> clusters(static_cast<std::size_t>(std::max(0, num_clusters)));
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (labels[i] > NOISE) {
            clusters[static_cast<std::size_t>(labels[i] - 1)].push_back(points[i]);
        }
    }
    return clusters;
}

Eigen::Vector3d predictCenter(const LidarObstacle& obstacle, const double stamp) {
    const Measurement& last = obstacle.history.back();
    return last.center + obstacle.v * (stamp - last.stamp);
}

std::vector<int> associateClusters(LidarObstacles& tracks, const std::vector<std::vector<Eigen::Vector3d>>& clusters,
                                   const double stamp, const ClusteringParams& params, int& next_track_id) {
    auto& track_list = tracks.lidar_obstacles;
    const std::size_t num_tracks = track_list.size();  // tracks of earlier scans; new ones are appended below

    std::vector<Eigen::Vector3d> cluster_centers;
    cluster_centers.reserve(clusters.size());
    for (const auto& cluster : clusters) {
        cluster_centers.push_back(avgPoint(cluster));
    }

    // candidate pairs within the threshold
    struct Candidate {
        double distance;
        std::size_t track;
        std::size_t cluster;
    };
    std::vector<Candidate> candidates;
    for (std::size_t t = 0; t < num_tracks; ++t) {
        if (track_list[t].history.empty()) {
            continue;  // no position to compare with
        }
        const Eigen::Vector3d predicted_center = predictCenter(track_list[t], stamp);
        for (std::size_t c = 0; c < clusters.size(); ++c) {
            const double distance = (cluster_centers[c] - predicted_center).norm();
            if (distance <= params.max_association_distance) {
                candidates.push_back({distance, t, c});
            }
        }
    }

    // closest pairs first; each track and each cluster at most once
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.distance < b.distance; });
    std::vector<bool> track_matched(num_tracks, false);
    std::vector<bool> cluster_matched(clusters.size(), false);
    std::vector<int> cluster_track_ids(clusters.size(), -1);
    for (const Candidate& candidate : candidates) {
        if (track_matched[candidate.track] || cluster_matched[candidate.cluster]) {
            continue;
        }
        track_matched[candidate.track] = true;
        cluster_matched[candidate.cluster] = true;
        updateLidarObs(track_list[candidate.track], clusters[candidate.cluster], stamp, params);
        cluster_track_ids[candidate.cluster] = track_list[candidate.track].ID;
    }

    // clusters without a track close enough: new tracks
    for (std::size_t c = 0; c < clusters.size(); ++c) {
        if (cluster_matched[c]) {
            continue;
        }
        LidarObstacle track{};
        track.ID = next_track_id++;
        updateLidarObs(track, clusters[c], stamp, params);
        cluster_track_ids[c] = track.ID;
        track_list.push_back(std::move(track));
    }
    return cluster_track_ids;
}

}  // namespace clustering
