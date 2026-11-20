// Known obstacles of the offline map (convex polytopes from half-planes) and a voxel grid for fast fixed-radius
// neighbour search in a point cloud (used by DBSCAN).
// Plain C++ / Eigen only.
#pragma once

#include "pcd_constraints/constraints.hpp"

#include <Eigen/Core>

#include <cstdint>
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

/// @brief Convex obstacle: the intersection of the obstacle sides of its half-planes.
///        HalfPlane convention: normals point out of the obstacle towards free space, free side
///        normal.dot(x) + offset >= 0.
struct MapObstacle {
    int ID;
    std::vector<HalfPlane> planes;
};

struct MapObstacles {
    std::vector<MapObstacle> map_obstacle;
};

struct LidarObstacle {
    int ID;
    bool active; 
    int motion; // 0: static, 1: linear
    std::vector<HalfPlane> planes;
    Eigen::Vector3d center;
    Eigen::Vector3d v;
};

struct LidarObstacles {
    std::vector<LidarObstacle> lidar_obstacles;
};

/// @brief Is p inside the convex region of these half-planes (intersection of their obstacle sides)?
///        eps [m] inflates the region: points up to eps outside it count as inside.
bool insideObstacle(const std::vector<HalfPlane>& planes, const Eigen::Vector3d& p, double eps = 1e-3);

/// @brief Is p inside the obstacle (insideObstacle of its planes)?
///        For a moving LidarObstacle the planes are tested as stored, i.e. where it was when they were computed.
bool insideObstacle(const MapObstacle& obstacle, const Eigen::Vector3d& p, double eps = 1e-3);
bool insideObstacle(const LidarObstacle& obstacle, const Eigen::Vector3d& p, double eps = 1e-3);

/// @brief Is p inside any of the obstacles (insideObstacle)?
bool insideAnyObstacle(const MapObstacles& obstacles, const Eigen::Vector3d& p, double eps = 1e-3);
bool insideAnyObstacle(const LidarObstacles& obstacles, const Eigen::Vector3d& p, double eps = 1e-3);

/// @brief Remove the points that lie inside any known obstacle (in place).
void removeKnownPoints(const Obstacles& obstacles, std::vector<Eigen::Vector3d>& points);

/// @brief Read the obstacles of the offline map from a CSV file: header line "nx,ny,nz,offset,obstacle"
/// @throws std::runtime_error if the file cannot be opened or a row is not 4 numbers + 1 integer / has a zero normal
MapObstacles loadMapObstacles(const std::string& path);

/// @brief Is the obstacle a closed (bounded) shape?
///        Needs a top plane (normal within horizontal_tol_deg of +z), a bottom plane (within horizontal_tol_deg of
///        -z) and side planes whose normals, seen from above, surround the
///        obstacle: no gap of 180 deg - horizontal_tol_deg or more between neighbouring directions. 
/// @param obstacle normals of unit length, out of the obstacle
/// @param horizontal_tol_deg [deg] max angle of a top / bottom normal from vertical
bool isClosedObstacle(const MapObstacle& obstacle, double horizontal_tol_deg = 5.0);

/// @brief Remove the obstacles that are not closed (isClosedObstacle) from map_obstacles
/// @return IDs of the removed obstacles, in their original order
std::vector<int> obstacleFeasibilityCheck(MapObstacles& map_obstacles, double horizontal_tol_deg = 5.0);

}  // namespace clustering
