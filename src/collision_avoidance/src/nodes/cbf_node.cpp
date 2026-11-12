// Implementation of cbf_node: read parameters, keep the latest state and constraints,
// convert ROS messages <-> library types and publish. No CBF math belongs here.
#include "nodes/cbf_node.hpp"

#include "collision_avoidance/cbf/cbf_filter.hpp"

#include <algorithm>
#include <cmath>

namespace {

/// @brief Convert the EKF state estimate (nav_msgs/Odometry) into the 13-entry model state.
///        In the bag, frame_id = map and the twist.linear velocity is in the world frame (map),
///        matching VX, VY, VZ of the model.
///        UF_ROLL / UF_PITCH (lagged command) are not in Odometry; they are set to the current tilt,
///        which is where the tilt servo settles (d_roll -> uf_roll), so the prediction starts steady.
/// @param msg state estimate from /filter/state_estimate
/// @return state vector, layout see ekf_estimator/types.hpp
ekf_estimator::StateVec toStateVec(const nav_msgs::msg::Odometry& msg) {
    using namespace ekf_estimator;
    const auto& p = msg.pose.pose.position;
    const auto& q = msg.pose.pose.orientation;
    const auto& v = msg.twist.twist.linear;
    const auto& w = msg.twist.twist.angular;

    // quaternion -> roll, pitch, yaw (ZYX convention)
    const double roll  = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    const double pitch = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
    const double yaw   = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    StateVec x = StateVec::Zero();
    x(PX) = p.x;  x(PY) = p.y;  x(PZ) = p.z;
    x(VX) = v.x;  x(VY) = v.y;  x(VZ) = v.z;
    x(DROLL) = roll;   x(DPITCH) = pitch;  // tilt angles
    x(WROLL) = w.x;    x(WPITCH) = w.y;    // tilt rates (body rates, close to angle rates for small tilts)
    x(YAW) = yaw;
    x(UF_ROLL)  = roll;                    // TODO: Is this needed? How much is this affecting the dynamics
    x(UF_PITCH) = pitch;
    return x;
}


// TODO: ADD TRANSFORMATION TO WORLD FRAME
std::vector<collision_avoidance::HalfPlane> toHalfPlane(const collision_avoidance_msgs::msg::HalfPlaneArray& msg) {
    std::vector<collision_avoidance::HalfPlane> x;
    x.clear();
    x.reserve(msg.planes.size());
    for (const auto& p : msg.planes) {
        collision_avoidance::HalfPlane hp;
        hp.normal = Eigen::Vector3d(p.coef[0], p.coef[1], p.coef[2]);
        hp.offset = -p.coef[3];
        x.push_back(hp);
    }
    return x;
}

}  // namespace

CbfNode::CbfNode() : Node("cbf_node") {
    param_listener_ = 
        std::make_unique<cbf::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    // PUBLISHER
    input_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(params_.topics.output.safe_command, rclcpp::QoS(10));

    // SUBSCRIBERS
    input_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        params_.topics.input.nominal_command, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::TwistStamped& msg) {controllerCallback(msg, constraints_, state_); }
    );

    state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        params_.topics.input.state, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry& msg) {stateCallback(msg); }
    );
    
    constraint_sub_ = create_subscription<collision_avoidance_msgs::msg::HalfPlaneArray>(
        params_.topics.input.constraints, rclcpp::SensorDataQoS(),
        [this](const collision_avoidance_msgs::msg::HalfPlaneArray& msg) {constraintCallback(msg); });
}

void CbfNode::controllerCallback(const geometry_msgs::msg::TwistStamped& u_ref, 
                                const std::vector<collision_avoidance::HalfPlane>& constraints, 
                                const ekf_estimator::StateVec& state) {
    ekf_estimator::InputVec u;
    double slack;

    collision_avoidance::safety_filter(state, constraints, u_sent, u_ref, u, ekf_estimator::DroneModel, params_.barrier_function,)

    return; // TODO: CBF optimization
}

// convert constraint into HalfPlane (SHOULD ALSO TRANSFORM INTO WORLD FRAME)
void CbfNode::constraintCallback(const collision_avoidance_msgs::msg::HalfPlaneArray& msg) {
    constraints_ = toHalfPlane(msg);
}

void CbfNode::stateCallback(const nav_msgs::msg::Odometry& msg) {
    state_ = toStateVec(msg);
}
