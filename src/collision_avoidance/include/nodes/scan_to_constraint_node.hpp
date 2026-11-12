// Declaration of scan_to_constraint_node: LiDAR scan in -> half-plane constraints out.
// Node headers hold only the ROS plumbing: parameters, subscriptions, publishers, callbacks.
// The actual computation is done by a ClusteringAlgorithm from the library.
#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "collision_avoidance_msgs/msg/half_plane_array.hpp"
#include "collision_avoidance/cbf_parameters.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "collision_avoidance/scan_to_constraint_parameters.hpp"

/// @brief Computes half-plane constraints from every LiDAR scan with the configured algorithm.
class ScanToConstraintNode : public rclcpp::Node {
   public:
    ScanToConstraintNode();

   private:
    /// @brief Convert the cloud, run the algorithm, publish the constraints.
    void cloudCallback(const sensor_msgs::msg::PointCloud2& cloud);

    std::unique_ptr<scan_to_constraint::ParamListener> param_listener_;
    scan_to_constraint::Params params_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<collision_avoidance_msgs::msg::HalfPlaneArray>::SharedPtr constraints_pub_;
};
