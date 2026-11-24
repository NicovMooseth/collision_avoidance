#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <collision_avoidance_msgs/msg/lidar_obstacle_array.hpp>
#include <collision_avoidance_msgs/msg/tracking_stats.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "clustering/clustering.hpp"
#include "clustering/clustering_parameters.hpp"
#include "clustering/map_parameters.hpp"

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

        /// @brief Add the points of this scan to scan_buffer_, drop old scans, return the points of all scans in it.
        ///        Keeps at most accumulation.num_scans scans, none older than (num_scans - 0.5) / lidar_frequency
        ///        before this scan (skipped scans do not stretch the window), and clears the buffer if time jumps
        ///        back (bag restarted).
        /// @param scan_time stamp of this scan [s]
        /// @param points of this scan, world frame, map obstacles removed
        std::vector<Eigen::Vector3d> integrateScans(double scan_time, std::vector<Eigen::Vector3d> points);

        /// @brief Pose of the drone (body -> world) closest in time to stamp, if one is close enough.
        std::optional<Eigen::Isometry3d> poseAt(const rclcpp::Time& stamp) const;

        /// @brief Publish the points left after removing the known (map) obstacles, world frame.
        void publishPoints(const std_msgs::msg::Header& header, const std::vector<Eigen::Vector3d>& points);

        /// @brief Publish the clusters of this scan (colored by track ID), the tracks and their RViz markers.
        /// @param header stamp = scan time, frame_id = world frame
        /// @param clusters points of each cluster of the scan, world frame
        /// @param cluster_track_ids track ID of each cluster (associateClusters)
        void publishResults(const std_msgs::msg::Header& header,
                            const std::vector<std::vector<Eigen::Vector3d>>& clusters,
                            const std::vector<int>& cluster_track_ids);

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

        Eigen::Isometry3d T_BS_;

        /// Tracked LiDAR obstacles, each with its own measurement history (LidarObstacle::history). Tracks not
        /// seen for history.track_timeout seconds are deleted.
        clustering::LidarObstacles tracks_;
        int next_track_id_ = 0;  ///< ID of the next new track
        int num_confirmed_total_ = 0;  ///< tracks confirmed since the start (statistics)
        clustering::ClusteringParams clustering_params_;  ///< library parameters, filled from params_ once

        /// Points of the last scans after removing the map obstacles (scan stamp [s], points in the world frame),
        /// oldest first: at most accumulation.num_scans scans. Points stay where they were measured (no motion
        /// compensation of moving obstacles).
        std::deque<std::pair<double, std::vector<Eigen::Vector3d>>> scan_buffer_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
        rclcpp::Publisher<collision_avoidance_msgs::msg::LidarObstacleArray>::SharedPtr obstacle_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Publisher<collision_avoidance_msgs::msg::TrackingStats>::SharedPtr stats_pub_;
};