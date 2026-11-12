// Declaration of cbf_node: the safety filter.
// Subscribes to constraints, state estimate and nominal command; publishes the safe command.
// Node headers hold only the ROS plumbing; the filter itself is collision_avoidance::CbfFilter.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "collision_avoidance_msgs/msg/half_plane_array.hpp"

#include "collision_avoidance/cbf_parameters.hpp"

#include "collision_avoidance/constraints.hpp"

#include "ekf_estimator/types.hpp"

class CbfNode : public rclcpp::Node {
    public: 
        CbfNode();

    private:
        void controllerCallback(const geometry_msgs::msg::TwistStamped& msg, 
                                const std::vector<collision_avoidance::HalfPlane>& constraints, 
                                const ekf_estimator::StateVec& state);

        void constraintCallback(const collision_avoidance_msgs::msg::HalfPlaneArray& msg);

        void stateCallback(const nav_msgs::msg::Odometry& msg);

        std::unique_ptr<cbf::ParamListener> param_listener_;
        cbf::Params params_;

        rclcpp::Subscription<collision_avoidance_msgs::msg::HalfPlaneArray>::SharedPtr constraint_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr input_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;

        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr input_pub_;

        std::vector<collision_avoidance::HalfPlane> constraints_;
        ekf_estimator::StateVec state_;
};