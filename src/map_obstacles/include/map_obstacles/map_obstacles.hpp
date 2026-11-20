// Obstacles of the offline map: convex polytopes given by half-planes, loaded from a CSV file.
// Shared by every package that needs the map (cbf_safety_filter: collision checks, clustering: removal of known
// points), so that none of them depends on another.
// Plain C++ / Eigen only — no ROS includes (convention rule 1).
//
// Convention (pcd_constraints::HalfPlane): the unit normal points out of the obstacle towards free space, the free
// side of a plane is normal.dot(p) + offset >= 0, and p is inside the obstacle iff normal.dot(p) + offset < 0 for
// all its planes. Map / world frame.
#pragma once

#include "pcd_constraints/constraints.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace map_obstacles {

using pcd_constraints::HalfPlane;

/// @brief Convex obstacle of the offline map: the intersection of the obstacle sides of its half-planes.
struct MapObstacle {
    int ID;                         ///< obstacle column of the CSV
    std::vector<HalfPlane> planes;  ///< unit normals, out of the obstacle
};

struct MapObstacles {
    std::vector<MapObstacle> map_obstacle;
};

/// @brief Read the obstacles of the offline map from a CSV file: header line "nx,ny,nz,offset,obstacle", then one
///        half-plane per row, obstacle = integer ID of the convex obstacle the plane belongs to. The normal is
///        scaled to unit length (offset with it). Rows of one obstacle need not be contiguous; obstacles are
///        returned in the order their ID first appears. Empty lines and lines starting with '#' are skipped.
///        Obstacles that are not closed (isClosedObstacle) are removed, with a warning listing their IDs on
///        std::cerr.
/// @param path CSV file
/// @param horizontal_tol_deg [deg] passed to isClosedObstacle
/// @throws std::runtime_error if the file cannot be opened, the header is wrong or a row is not 4 numbers +
///         1 integer / has a zero normal
MapObstacles loadMapObstacles(const std::string& path, double horizontal_tol_deg = 5.0);

/// @brief Is the obstacle a closed (bounded) shape?
///        Needs a top plane (normal within horizontal_tol_deg of +z), a bottom plane (within horizontal_tol_deg of
///        -z) and side planes (all others, sloped ones included) whose normals, seen from above, surround the
///        obstacle: no gap of 180 deg - horizontal_tol_deg or more between neighbouring directions. That implies at
///        least 3 non-parallel sides (3 sides facing 0, 30 and 60 deg are not enough: open towards 210 deg).
///        Only the normals are checked, not whether the planes enclose a non-empty region.
/// @param obstacle normals of unit length, out of the obstacle
/// @param horizontal_tol_deg [deg] max angle of a top / bottom normal from vertical
bool isClosedObstacle(const MapObstacle& obstacle, double horizontal_tol_deg = 5.0);

/// @brief Remove the obstacles that are not closed (isClosedObstacle) from map_obstacles (in place).
/// @return IDs of the removed obstacles, in their original order
std::vector<int> obstacleFeasibilityCheck(MapObstacles& map_obstacles, double horizontal_tol_deg = 5.0);

}  // namespace map_obstacles
