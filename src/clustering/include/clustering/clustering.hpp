// Obstacles of the LiDAR scans, point-in-obstacle tests (offline map and LiDAR obstacles), box fit and a voxel grid
// for fast fixed-radius neighbour search in a point cloud (used by DBSCAN). The offline map itself (MapObstacle,
// loadMapObstacles) is in the map_obstacles package.
// Plain C++ / Eigen only.
#pragma once

#include "map_obstacles/map_obstacles.hpp"
#include "pcd_constraints/constraints.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

namespace clustering {

// the points sorted into cubic voxels of (just over) eps. Two points within eps are at most one voxel apart per axis.
// Keeps a reference to points: they must outlive the grid and must not change while it is used.
class VoxelGrid {
public:
    VoxelGrid(const std::vector<Eigen::Vector3f>& points, const float eps)
        : points_(points), eps_squared_(eps * eps), voxel_of_point_(points.size()) {
        const float voxel_size = eps * 1.001f;  // the margin covers float rounding at the voxel borders

        std::unordered_map<std::uint64_t, int> voxel_ids;
        voxel_ids.reserve(points.size());
        for (int i = 0; i < static_cast<int>(points.size()); ++i) {
            const Eigen::Vector3i voxel = voxel_coordinates(points[i], voxel_size);
            const auto [entry, is_new] = voxel_ids.try_emplace(voxel_key(voxel), static_cast<int>(voxels_.size()));
            if (is_new) {
                voxels_.push_back({voxel, {}, {}});
            }
            voxels_[entry->second].points.push_back(i);
            voxel_of_point_[i] = entry->second;
        }

        // the occupied voxels around each voxel: looked up once per voxel instead of 27 hash lookups per query
        for (Voxel& voxel : voxels_) {
            for (int x = -1; x <= 1; ++x) {
                for (int y = -1; y <= 1; ++y) {
                    for (int z = -1; z <= 1; ++z) {
                        const auto entry = voxel_ids.find(voxel_key(voxel.coordinates + Eigen::Vector3i(x, y, z)));
                        if (entry != voxel_ids.end()) {
                            voxel.around.push_back(entry->second);
                        }
                    }
                }
            }
        }
    }

    // all points within eps of points[index], the point itself included. Fills neighbours: reuses its memory.
    void find_neighbours(const int index, std::vector<int>& neighbours) const {
        neighbours.clear();
        const Eigen::Vector3f point = points_[index];
        for (const int voxel : voxels_[voxel_of_point_[index]].around) {
            for (const int candidate : voxels_[voxel].points) {
                if ((point - points_[candidate]).squaredNorm() <= eps_squared_) {
                    neighbours.push_back(candidate);
                }
            }
        }
    }

private:
    struct Voxel {
        Eigen::Vector3i coordinates;
        std::vector<int> points;  // indices into points_
        std::vector<int> around;  // this voxel and its occupied neighbours, indices into voxels_
    };

    static Eigen::Vector3i voxel_coordinates(const Eigen::Vector3f& point, const float voxel_size) {
        return (point / voxel_size).array().floor().cast<int>();
    }

    // 21 bits per axis, shifted to be positive: voxel coordinates up to +-2^20, enough for eps > 0.1 mm at MAX_RANGE
    static std::uint64_t voxel_key(const Eigen::Vector3i& voxel) {
        constexpr int OFFSET = 1 << 20;
        return std::uint64_t{static_cast<std::uint32_t>(voxel.x() + OFFSET)} << 42 |
               std::uint64_t{static_cast<std::uint32_t>(voxel.y() + OFFSET)} << 21 |
               std::uint64_t{static_cast<std::uint32_t>(voxel.z() + OFFSET)};
    }

    const std::vector<Eigen::Vector3f>& points_;
    float eps_squared_;
    std::vector<int> voxel_of_point_;
    std::vector<Voxel> voxels_;
};

/// The half-plane type of the workspace (pcd_constraints/constraints.hpp), not a copy of it.
using pcd_constraints::HalfPlane;

/// Obstacles of the offline map (map_obstacles package), not copies of them.
using map_obstacles::MapObstacle;
using map_obstacles::MapObstacles;

/// One measurement of a tracked obstacle.
struct Measurement {
    double stamp;            ///< scan stamp [s]; time steps vary when scans are skipped or the obstacle was hidden
    Eigen::Vector3d center;  ///< center of the cluster, world frame [m]
};

/// One tracked obstacle (track): its own measurement history and current estimate.
struct LidarObstacle {
    int ID;
    double first_seen = 0.0;         ///< stamp of the first measurement, when the track was created [s]
    double last_seen = 0.0;          ///< stamp of the last matched measurement [s]
    double lifetime = 0.0;           ///< last_seen - first_seen: how long the track has been followed [s]
    int num_matches = 0;             ///< measurements matched so far (all, not only those kept in history) [-]
    bool confirmed = false;          ///< matched in at least ClusteringParams::min_matches scans; only confirmed
                                     ///< tracks are published, unconfirmed ones are deleted after one missed scan
    int motion;                      ///< 0: static, 1: linear
    std::vector<HalfPlane> planes;   ///< shape of the latest measurement, world frame
    std::deque<Measurement> history; ///< last measurements, oldest first (at most history.num_measurements)
    Eigen::Vector3d v = Eigen::Vector3d::Zero();  ///< velocity estimate, world frame [m/s]
};

/// All tracked LiDAR obstacles.
struct LidarObstacles {
    std::vector<LidarObstacle> lidar_obstacles;
};

/// Parameters of the clustering library, filled by the node from its ROS parameters.
struct ClusteringParams {
    double eps = 0.3;                  ///< DBSCAN neighbourhood radius [m]
    std::size_t min_points = 15;       ///< DBSCAN: neighbours of a core point, itself included [-]
    double min_size = 0.1;             ///< fitAlignedBox: minimum side length of a box [m]
    std::size_t num_measurements = 5;  ///< measurements kept per track [-]
    std::size_t min_measurements = 3;  ///< measurements a track needs before its velocity is estimated [-]
    double max_static_speed = 0.3;     ///< tracks slower than this count as static [m/s]
    double max_association_distance = 0.5;  ///< a cluster further than this from every predicted track center
                                            ///< starts a new track [m]
    double known_obstacle_margin = 0.1;     ///< removeKnownPoints: points up to this far outside a map obstacle
                                            ///< count as known (LiDAR noise, odometry drift) [m]
    std::size_t min_matches = 1;       ///< a track is confirmed once it was matched in this many scans [-]
};

/// @brief Is p inside the convex region of these half-planes (intersection of their obstacle sides)?
///        eps [m] inflates the region: points up to eps outside it count as inside.
bool insideObstacle(const std::vector<HalfPlane>& planes, const Eigen::Vector3d& p, double eps);

/// @brief Is p inside the obstacle (insideObstacle of its planes)?
///        For a moving LidarObstacle the planes are tested as stored, i.e. where it was when they were computed.
bool insideObstacle(const MapObstacle& obstacle, const Eigen::Vector3d& p, double eps);
bool insideObstacle(const LidarObstacle& obstacle, const Eigen::Vector3d& p, double eps);

/// @brief Is p inside any of the obstacles (insideObstacle)?
bool insideAnyObstacle(const MapObstacles& obstacles, const Eigen::Vector3d& p, double eps);
bool insideAnyObstacle(const LidarObstacles& obstacles, const Eigen::Vector3d& p, double eps);

/// @brief Convex hull of 2D points (Andrew's monotone chain), counter-clockwise, first point not repeated.
///        Collinear points on the edges and duplicates are dropped. Fewer than 3 distinct points: returns them
///        (sorted by x, then y); all points on one line: its two end points.
std::vector<Eigen::Vector2d> convexHull2d(std::vector<Eigen::Vector2d> points);

/// @brief Smallest-area box around the points with vertical sides and horizontal top / bottom.
///        The horizontal orientation is the one of the smallest rectangle around the 2D convex hull (one side
///        along a hull edge); the height is the z range of the points. Sides shorter than min_size [m] (flat
///        clusters, a single point) are widened to min_size around their middle.
/// @param points at least one point, world frame
/// @return 6 half-planes, normals out of the box: +u, -u, +w, -w (sides, u along the chosen hull edge), +z, -z
/// @throws std::invalid_argument if points is empty
std::vector<HalfPlane> fitAlignedBox(const std::vector<Eigen::Vector3d>& points, const ClusteringParams& params);

/// @brief Mean of the points (center of a cluster).
/// @param points at least one point
Eigen::Vector3d avgPoint(const std::vector<Eigen::Vector3d>& points);

/// @brief Update the track with the cluster matched to it in the current scan (or a new track with its first
///        cluster): history (updateHistory), last_seen (updateTime), shape (updatePlanes), then velocity and
///        motion type from the updated history (updateMotion). Confirms the track once num_matches reaches
///        params.min_matches (stays confirmed afterwards).
/// @param obstacle the track
/// @param cluster_points points of the matched cluster, world frame, at least one
/// @param stamp scan stamp [s]
void updateLidarObs(LidarObstacle& obstacle, const std::vector<Eigen::Vector3d>& cluster_points, double stamp,
                    const ClusteringParams& params);

/// @brief Append the measurement and drop the oldest ones beyond params.num_measurements (at least 1 is kept).
void updateHistory(LidarObstacle& obstacle, const Measurement& measurement, const ClusteringParams& params);

/// @brief last_seen = stamp [s], lifetime = last_seen - first_seen, num_matches + 1.
///        A new track (num_matches == 0) gets first_seen = stamp.
void updateTime(LidarObstacle& obstacle, double stamp);

/// @brief planes = box around the cluster (fitAlignedBox): the shape of the latest measurement.
void updatePlanes(LidarObstacle& obstacle, const std::vector<Eigen::Vector3d>& cluster_points,
                  const ClusteringParams& params);

/// @brief v = least-squares line through the centers of the history over time; motion = 1 (linear) if
///        |v| > params.max_static_speed, else 0 (static). With fewer than params.min_measurements (at least 2)
///        measurements, or all at the same time: v = 0, motion = 0.
void updateMotion(LidarObstacle& obstacle, const ClusteringParams& params);

/// @brief Remove the points that lie inside any known obstacle, or up to margin outside it (in place).
///        The margin moves every plane of an obstacle outwards by margin (insideObstacle eps), so it covers
///        points on the surface that scatter to its free side (LiDAR noise, odometry drift).
/// @param margin [m], >= 0
void removeKnownPoints(const MapObstacles& obstacles, std::vector<Eigen::Vector3d>& points, double margin);

/// @brief DBSCAN: the label of each point, NOISE (0) or a cluster id (1, 2, ...).
///        params.eps [m]: neighbourhood radius; params.min_points: neighbours of a core point, itself included.
std::vector<int> dbscan_fast(const std::vector<Eigen::Vector3d>& points, const ClusteringParams& params);

/// @brief The points of each cluster: result[k] holds the points with label k + 1. Noise (label 0) is dropped.
std::vector<std::vector<Eigen::Vector3d>> splitClusters(const std::vector<Eigen::Vector3d>& points,
                                                        const std::vector<int>& labels);

/// @brief Center of the track at stamp [s]: last measured center + v * (stamp - last measurement stamp).
///        v is zero for static tracks, so they stay where they were measured last.
/// @param obstacle a track with at least one measurement
Eigen::Vector3d predictCenter(const LidarObstacle& obstacle, double stamp);

/// @brief Match the clusters of one scan to the tracks and update them (updateLidarObs).
///        Greedy nearest neighbour: all track-cluster pairs with the distance between the predicted track
///        center (predictCenter) and the cluster center below params.max_association_distance, shortest first;
///        each track and each cluster is used at most once. Unmatched clusters start new tracks (ID
///        next_track_id++). Unmatched tracks are not changed (deleted later by the timeout).
/// @param tracks updated in place
/// @param clusters points of each cluster of the scan (splitClusters), world frame, none empty
/// @param stamp scan stamp [s]
/// @return the track ID of each cluster (same order as clusters)
std::vector<int> associateClusters(LidarObstacles& tracks, const std::vector<std::vector<Eigen::Vector3d>>& clusters,
                                   double stamp, const ClusteringParams& params, int& next_track_id);

}  // namespace clustering
