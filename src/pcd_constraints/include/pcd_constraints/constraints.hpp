// The one half-plane type of the workspace, shared by every package: pcd_constraints (scan half-planes),
// clustering (map / LiDAR obstacles) and cbf_safety_filter (consumes them). Do not define another one; other
// namespaces use it with `using pcd_constraints::HalfPlane;`.
// Plain C++ / Eigen only — no ROS includes in this folder (convention rule 1).
#pragma once

#include <Eigen/Core>

namespace pcd_constraints {

/// @brief Half-plane on the drone position x, free side:  normal.dot(x) + offset >= 0
///        (as shape_msgs/Plane: coef = [normal, offset], the plane is normal.dot(x) + offset = 0).
///        As a side of a convex obstacle: the normal points out of the obstacle, x is inside the obstacle iff
///        normal.dot(x) + offset < 0 for all its planes.
struct HalfPlane {
    Eigen::Vector3d normal;  /// < unit length, facing the free side (out of the obstacle; for a scan: towards the LiDAR)
    double offset;           /// < [m] signed distance from the origin of the frame to the plane, > 0 if the origin is free
};

}  // namespace pcd_constraints
