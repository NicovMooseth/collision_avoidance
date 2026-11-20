#include "nodes/clustering_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <string>

namespace {

/// @brief map.path -> absolute path: a relative path is relative to the share directory of map_obstacles
///        (its maps/ folder is installed there), an absolute path is returned unchanged.
std::string resolveMapPath(const std::string& path) {
    const std::filesystem::path p(path);
    if (p.is_absolute()) {
        return path;
    }
    return (std::filesystem::path(ament_index_cpp::get_package_share_directory("map_obstacles")) / p).string();
}

/// @brief The finite points of the cloud, in the frame of the cloud (sensor frame).
std::vector<Eigen::Vector3d> cloudToPoints(const sensor_msgs::msg::PointCloud2& cloud) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
            points.emplace_back(*x, *y, *z);
        }
    }
    return points;
}

}  // namespace

ClusteringNode::ClusteringNode() : Node("clustering_node") {
    param_listener_ =
        std::make_unique<clustering::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    // sensor -> body extrinsic; fixed_size validation guarantees 9 and 3 entries
    const auto& r = params_.extrinsics.fast_lio_to_body.rotation;
    const auto& t = params_.extrinsics.fast_lio_to_body.translation;
    T_BS_ = Eigen::Isometry3d::Identity();
    T_BS_.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(r.data());  // file is row-major
    T_BS_.translation() = Eigen::Vector3d(t[0], t[1], t[2]);

    // obstacles of the offline map: map.yaml gives the parameters, the CSV the half-planes
    map_loader_ =
        std::make_unique<clustering_map::ParamListener>(get_node_parameters_interface());
    map_params_ = map_loader_->get_params();
    const std::string map_path = resolveMapPath(map_params_.map.path);
    obstacles_ = map_obstacles::loadMapObstacles(map_path);
    RCLCPP_INFO(get_logger(), "Loaded %zu obstacles of the offline map from %s",
                obstacles_.map_obstacle.size(), map_path.c_str());

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        params_.topics.input.points, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2& cloud) { cloudCallback(cloud); }
    );

    state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        params_.topics.input.state, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry& msg) { stateCallback(msg); }
    );

    // PUBLISHING
}

std::optional<Eigen::Isometry3d> ClusteringNode::poseAt(const rclcpp::Time& stamp) const {
    // nearest stored pose in time (states come at ~50 Hz, so at most ~10 ms away)
    const std::pair<rclcpp::Time, Eigen::Isometry3d>* best = nullptr;
    double best_dt = std::numeric_limits<double>::infinity();
    for (const auto& entry : pose_buffer_) {
        const double dt = std::abs((entry.first - stamp).seconds());
        if (dt < best_dt) {
            best_dt = dt;
            best = &entry;
        }
    }
    if (best == nullptr || best_dt > params_.state.max_time_offset) {
        return std::nullopt;
    }
    return best->second;
}

void ClusteringNode::transformToWorld(std::vector<Eigen::Vector3d>& points, const Eigen::Isometry3d& T_WB) const {
    // sensor -> world: x_W = R_WS x_S + p_WS, with T_WS = T_WB T_BS
    const Eigen::Isometry3d T_WS = T_WB * T_BS_;
    const Eigen::Matrix3d R_WS = T_WS.linear();
    const Eigen::Vector3d p_WS = T_WS.translation();
    for (Eigen::Vector3d& point : points) {
        point = R_WS * point + p_WS;
    }
}

void ClusteringNode::cloudCallback(const sensor_msgs::msg::PointCloud2& cloud) {
    using namespace clustering;
    // the map is in the world frame, so the pose at scan time is needed first
    const auto T_WB = poseAt(rclcpp::Time(cloud.header.stamp));
    if (!T_WB) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No state estimate close to the scan time, scan skipped");
        return;
    }

    std::vector<Eigen::Vector3d> points = cloudToPoints(cloud);
    transformToWorld(points, *T_WB); // transform to world frame

    // ONE FUNCTION CALL TO clustering.cpp 
    // SHOULD DIRECTLY RETURN HALFPLANE CONSTRAINTS

    // not here, but in clustering.cpp
    removeKnownPoints(obstacles_, points);

    // points -> clustering
    std::vector<int> labels; 
    labels = dbscan_fast(points, params_.dbscan.eps, params_.dbscan.min_points)

    //

}

void ClusteringNode::stateCallback(const nav_msgs::msg::Odometry& msg) {
    const auto& p = msg.pose.pose.position;
    const auto& q = msg.pose.pose.orientation;

    Eigen::Isometry3d T_WB = Eigen::Isometry3d::Identity();
    T_WB.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    T_WB.translation() = Eigen::Vector3d(p.x, p.y, p.z);

    const rclcpp::Time stamp(msg.header.stamp);
    pose_buffer_.emplace_back(stamp, T_WB);
    world_frame_ = msg.header.frame_id;

    // keep only the last state.buffer_duration seconds (relative to the newest state, not to now())
    const rclcpp::Time cutoff = stamp - rclcpp::Duration::from_seconds(params_.state.buffer_duration);
    while (!pose_buffer_.empty() && pose_buffer_.front().first < cutoff) {
        pose_buffer_.pop_front();
    }
}
