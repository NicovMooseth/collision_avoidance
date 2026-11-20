#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "clustering/clustering.hpp"
#include "clustering/clustering_parameters.hpp"
#include "clustering/map_parameters.hpp"
#include "constants.h"

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

class ClusteringNode : public rclcpp::Node {
    public:
        ClusteringNode();
    
    private:
        /// @brief Convert the cloud, transform it into the world frame, remove the points of known obstacles.
        void cloudCallback(const sensor_msgs::msg::PointCloud2& cloud);

        /// @brief Store the drone pose with its stamp, so the pose at scan time can be looked up later.
        void stateCallback(const nav_msgs::msg::Odometry& msg);

        /// @brief Pose of the drone (body -> world) closest in time to stamp, if one is close enough.
        std::optional<Eigen::Isometry3d> poseAt(const rclcpp::Time& stamp) const;

        /// @brief Transform points from the sensor frame into the world frame (in place).
        /// @param points in the sensor frame on input, in the world frame on output
        /// @param T_WB drone pose at scan time (body -> world)
        void transformToWorld(std::vector<Eigen::Vector3d>& points, const Eigen::Isometry3d& T_WB) const;

        std::unique_ptr<clustering::ParamListener> param_listener_;
        clustering::Params params_;

        std::unique_ptr<clustering_map::ParamListener> map_loader_;  ///< parameters of the offline map (map.yaml)
        clustering_map::Params map_params_;
        map_obstacles::MapObstacles obstacles_;  ///< of the offline map (map_params_.map.path), loaded once

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;

        /// Recent drone poses (stamp, body -> world), oldest first.
        std::deque<std::pair<rclcpp::Time, Eigen::Isometry3d>> pose_buffer_;
        /// frame_id of the state estimate (the world frame, "map" in the bag); the frame of the map and of the
        /// transformed points.
        std::string world_frame_;

        Eigen::Isometry3d T_BS_;  ///< sensor -> body, from extrinsics.fast_lio_to_body

        // publish what?
        // obstacles, with motion estimate
        // rclcpp::Publisher<>;
};