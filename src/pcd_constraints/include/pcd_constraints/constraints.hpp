// Shared constraint types used by both the clustering algorithms (which produce them)
// and the CBF safety filter (which consumes them).
// Plain C++ / Eigen only — no ROS includes in this folder (convention rule 1).
#pragma once

#include <Eigen/Core>

namespace pcd_constraints {

/// @brief Half-plane constraint on the drone position x:  normal.dot(x) <= offset
struct HalfPlane {
    Eigen::Vector3d normal;  /// < unit length, towards the obstacle
    double offset;           /// < [m] distance from the LIDAR to the plane
};

}  // namespace pcd_constraints
