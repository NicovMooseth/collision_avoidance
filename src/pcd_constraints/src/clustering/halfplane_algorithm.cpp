#include "pcd_constraints/clustering/halfplane_algorithm.hpp"

#include <optional>

namespace pcd_constraints {

namespace {

std::optional<Eigen::Vector3f> nearestPoint(const std::vector<Eigen::Vector3f>& points, float min_range,
                                            float max_range) {
    std::optional<Eigen::Vector3f> nearestP;
    std::optional<float> nearest;

    for (const auto& p : points) {
        const float range = p.norm();
        if (range < min_range || range > max_range) {
            continue;
        }
        if (!nearest || range < *nearest) {
            nearest = range;
            nearestP = p;
        }
    }
    return nearestP;
}

std::vector<Eigen::Vector3f> removePoints(const std::vector<Eigen::Vector3f>& points,
                                          const Eigen::Vector3f& nearestPoint) {
    const float dist = nearestPoint.norm();
    const Eigen::Vector3f normal = -nearestPoint / dist;
    const float d = normal.dot(nearestPoint);

    std::vector<Eigen::Vector3f> kept;
    kept.reserve(points.size());

    constexpr float eps = 1e-4f;

    for (const auto& p : points) {
        if (normal.dot(p) - d > eps) {
            kept.push_back(p);
        }
    }
    return kept;
}

}  // namespace

std::vector<HalfPlane> computeConstraints(const std::vector<Eigen::Vector3f>& points, float min_range, float max_range) {
    std::vector<HalfPlane> planes;
    std::vector<Eigen::Vector3f> remaining = points;
    while (auto nearestP = nearestPoint(remaining, min_range, max_range)) {
        const float dist = nearestP->norm();
        remaining = removePoints(remaining, *nearestP);
        planes.push_back({(*nearestP / dist).cast<double>(), static_cast<double>(dist)});
    }
    return planes;
}

}  // namespace pcd_constraints
