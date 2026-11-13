// Declaration of cbf_node: the safety filter.
// Subscribes to constraints, state estimate and nominal command; publishes the safe command.
// Node headers hold only the ROS plumbing; the filter itself is collision_avoidance::CbfFilter.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <memory>
#include <optional>
#include <utility>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "collision_avoidance_msgs/msg/half_plane_array.hpp"

#include "collision_avoidance/cbf_parameters.hpp"

#include "collision_avoidance/constraints.hpp"

#include "collision_avoidance/cbf/cbf_filter.hpp"
#include "ekf_estimator/drone_model.hpp"
#include "ekf_estimator/types.hpp"

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

        std::vector<collision_avoidance::HalfPlane> constraints_;
        std::optional<ekf_estimator::StateVec> state_;

        collision_avoidance::BarrierFunctionParams bf_params_;
        std::optional<ekf_estimator::DroneModel> model_;
        /// Commands sent but not yet applied by the drone (send time, command), oldest first.
        std::deque<std::pair<rclcpp::Time, ekf_estimator::InputVec>> pending_;

};