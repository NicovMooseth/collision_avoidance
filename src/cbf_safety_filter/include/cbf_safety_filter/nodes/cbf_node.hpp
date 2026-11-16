// Declaration of cbf_node: the safety filter.
// Subscribes to constraints, state estimate and nominal command; publishes the safe command.
// Node headers hold only the ROS plumbing; the filter itself is cbf_safety_filter::CbfFilter.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <memory>
#include <optional>
#include <utility>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "collision_avoidance_msgs/msg/half_plane_array.hpp"

#include "cbf_safety_filter/cbf_parameters.hpp"

#include "pcd_constraints/constraints.hpp"

#include "cbf_safety_filter/cbf_filter.hpp"
#include "cbf_safety_filter/ekf/drone_model.hpp"
#include "cbf_safety_filter/ekf/types.hpp"

class CbfNode : public rclcpp::Node {
    public: 
        CbfNode();

    private:
        /// @brief Filter the nominal command with the CBF and publish the safe command.
        void controllerCallback(const geometry_msgs::msg::TwistStamped& msg);

        void constraintCallback(const collision_avoidance_msgs::msg::HalfPlaneArray& msg);

        void stateCallback(const nav_msgs::msg::Odometry& msg);

        std::unique_ptr<cbf::ParamListener> param_listener_;
        cbf::Params params_;

        rclcpp::Subscription<collision_avoidance_msgs::msg::HalfPlaneArray>::SharedPtr constraint_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr input_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;

        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr input_pub_;

        std::vector<pcd_constraints::HalfPlane> constraints_;
        std::optional<ekf_estimator::StateVec> state_;

        cbf_safety_filter::BarrierFunctionParams bf_params_;
        std::optional<ekf_estimator::DroneModel> model_;
        /// Commands sent but not yet applied by the drone (send time, command), oldest first.
        std::deque<std::pair<rclcpp::Time, ekf_estimator::InputVec>> pending_;

};