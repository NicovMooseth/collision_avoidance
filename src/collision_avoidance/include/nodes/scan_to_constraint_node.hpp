// Declaration of scan_to_constraint_node: LiDAR scan in -> half-plane constraints out.
// Node headers hold only the ROS plumbing: parameters, subscriptions, publishers, callbacks.
// The actual computation is done by a ClusteringAlgorithm from the library.
#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "collision_avoidance_msgs/msg/half_plane_array.hpp"
#include "collision_avoidance/constraints.hpp"
#include "collision_avoidance/scan_to_constraint_parameters.hpp"

/// @brief Computes half-plane constraints from every LiDAR scan with the configured algorithm
///        and publishes them in the world frame of the state estimate (map).
class ScanToConstraintNode : public rclcpp::Node {

   public:
    ScanToConstraintNode();

   private:
    /// @brief Convert the cloud, run the algorithm, transform to the world frame, publish the constraints.
    void cloudCallback(const sensor_msgs::msg::PointCloud2& cloud);

    /// @brief Store the drone pose with its stamp, so the pose at scan time can be looked up later.
    void stateCallback(const nav_msgs::msg::Odometry& msg);

    /// @brief Publish the static sensor -> body transform once both frame names are known.
    void publishStaticSensorTf(const std::string& sensor_frame);

    /// @brief Pose of the drone (body -> world) closest in time to stamp, if one is close enough.
    std::optional<Eigen::Isometry3d> poseAt(const rclcpp::Time& stamp) const;

    /// @brief Transform half-planes from the sensor frame into the world frame.
    /// @param planes constraints in the sensor frame (n·x <= offset)
    /// @param T_WB drone pose at scan time (body -> world)
    /// @return the same constraints in the world frame
    std::vector<collision_avoidance::HalfPlane> transformToMap(const std::vector<collision_avoidance::HalfPlane>& planes,
                                                               const Eigen::Isometry3d& T_WB) const;

    std::unique_ptr<scan_to_constraint::ParamListener> param_listener_;
    scan_to_constraint::Params params_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
    rclcpp::Publisher<collision_avoidance_msgs::msg::HalfPlaneArray>::SharedPtr constraints_pub_;

    /// Recent drone poses (stamp, body -> world), oldest first.
    std::deque<std::pair<rclcpp::Time, Eigen::Isometry3d>> pose_buffer_;
    /// frame_id of the state estimate (the world frame, "map" in the bag).
    std::string world_frame_;
    /// child_frame_id of the state estimate (the body frame, "base_link" in the bag).
    std::string body_frame_;

    /// TF for visualisation: world -> body (dynamic) and body -> sensor (static); only if tf.publish is true.
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    bool static_tf_sent_ = false;

    Eigen::Matrix3d R_BS_;  ///< rotation sensor -> body, from extrinsics.fast_lio_to_body.rotation
    Eigen::Vector3d t_BS_;  ///< sensor position in the body frame [m], from extrinsics.fast_lio_to_body.translation
};
