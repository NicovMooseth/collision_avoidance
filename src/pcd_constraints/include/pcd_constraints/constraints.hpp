// Shared constraint types used by both the clustering algorithms (which produce them)
// and the CBF safety filter (which consumes them).
// Plain C++ / Eigen only — no ROS includes in this folder (convention rule 1).
#pragma once

#include <Eigen/Core>

namespace pcd_constraints {

/// @brief Half-plane constraint on the drone position x, free side:  normal.dot(x) + offset >= 0
///        (as shape_msgs/Plane: coef = [normal, offset], the plane is normal.dot(x) + offset = 0)
struct HalfPlane {
    Eigen::Vector3d normal;  /// < unit length, facing the free side (towards the LiDAR / origin, away from the obstacle)
    double offset;           /// < [m] distance from the origin of the frame to the plane, > 0 if the origin is free
};

}  // namespace pcd_constraints
