#pragma once // load it only once

#include <vector>

#include <Eigen/Core>

#include "collision_avoidance/constraints.hpp"

namespace collision_avoidance {
    /// @brief Compute the half-plane constraint from the iteratively chosen closest points. Points behind a found half-plane are removed
    /// @param points scan points to find safe set from
    /// @param min_range [m] points closer are removed
    /// @param max_range [m] points farther away are removed
    /// @return collection of all half-planes found from the algorithm
    std::vector<HalfPlane> computeConstraints(const std::vector<Eigen::Vector3f>& points, float min_range, float max_range);

}