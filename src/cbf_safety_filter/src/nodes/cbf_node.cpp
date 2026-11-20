// Implementation of cbf_node: read parameters, keep the latest state and obstacles,
// convert ROS messages <-> library types and publish. No CBF math belongs here.
#include "cbf_safety_filter/nodes/cbf_node.hpp"

#include "cbf_safety_filter/cbf_filter.hpp"

#include "clustering.hpp"  // MapObstacle, LidarObstacle, loadMapObstacles

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <string>
#include <vector>

namespace {

/// @brief path_map -> absolute path: a relative path is relative to the share directory of this package
///        (config/ is installed there), an absolute path is returned unchanged.
std::string resolveMapPath(const std::string& path) {
    const std::filesystem::path p(path);
    if (p.is_absolute()) {
        return path;
    }
    return (std::filesystem::path(ament_index_cpp::get_package_share_directory("cbf_safety_filter")) / p).string();
}

/// @brief Obstacle of the offline map -> obstacle of the filter (static: velocity 0).
///        The planes are pcd_constraints::HalfPlane in both, so they are copied as they are.
cbf_safety_filter::Obstacle toFilterObstacle(const clustering::MapObstacle& obs) {
    return {obs.ID, obs.planes, Eigen::Vector3d::Zero()};
}

/// @brief Obstacle of the clustering node -> obstacle of the filter (with its velocity estimate).
///        Whether inactive obstacles (active == false) are passed on is up to the caller.
[[maybe_unused]] cbf_safety_filter::Obstacle toFilterObstacle(const clustering::LidarObstacle& obs) {
    return {obs.ID, obs.planes, obs.v};
}

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
    x(UF_ROLL)  = roll;                    // TODO: Is this needed? How much is this affecting estimation of state ~0.1 s ahead
    x(UF_PITCH) = pitch;
    return x;
}


/// @brief Half-planes of scan_to_constraint_node -> one obstacle per plane (a single half-plane is an obstacle
///        with one plane). coef = [normal, offset], free side coef·[p, 1] >= 0. The ID is the index in the message.
std::vector<cbf_safety_filter::Obstacle> toHalfPlaneObstacles(const collision_avoidance_msgs::msg::HalfPlaneArray& msg) {
    std::vector<cbf_safety_filter::Obstacle> obstacles;
    obstacles.reserve(msg.planes.size());
    for (std::size_t i = 0; i < msg.planes.size(); ++i) {
        const auto& c = msg.planes[i].coef;
        pcd_constraints::HalfPlane hp;
        hp.normal = Eigen::Vector3d(c[0], c[1], c[2]);
        hp.offset = c[3];
        obstacles.push_back({static_cast<int>(i), {hp}, Eigen::Vector3d::Zero()});
    }
    return obstacles;
}

/// @brief Command message -> model input. Convention (see ekf_estimator/types.hpp):
///        roll = linear.y, pitch = linear.x, yaw_rate = angular.z, v_z = linear.z
ekf_estimator::InputVec toInputVec(const geometry_msgs::msg::TwistStamped& msg) {
    using namespace ekf_estimator;
    InputVec u = InputVec::Zero();
    u(U_ROLL)   = msg.twist.linear.y;
    u(U_PITCH)  = msg.twist.linear.x;
    u(YAW_RATE) = msg.twist.angular.z;
    u(U_VEL_Z)  = msg.twist.linear.z;
    return u;
}

/// @brief Model input -> command message, inverse of toInputVec(). The header is set by the caller.
geometry_msgs::msg::TwistStamped toInputMsg(const ekf_estimator::InputVec& input) {
    using namespace ekf_estimator;
    geometry_msgs::msg::TwistStamped msg;
    msg.twist.linear.y  = input(U_ROLL);
    msg.twist.linear.x  = input(U_PITCH);
    msg.twist.angular.z = input(YAW_RATE);
    msg.twist.linear.z  = input(U_VEL_Z);
    return msg;
}

/// @brief Generated ROS parameters -> barrier function parameters of the core library.
cbf_safety_filter::BarrierFunctionParams toBarrierFunctionParams(const cbf::Params& p) {
    cbf_safety_filter::BarrierFunctionParams bf;
    bf.r_safety   = p.barrier_function.R_SAFETY;
    bf.t_reaction = p.barrier_function.T_REACTION;
    bf.alpha      = p.barrier_function.ALPHA;
    bf.max_tilt   = p.barrier_function.MAX_TILT;
    bf.max_v_z    = p.barrier_function.MAX_V_Z;
    bf.dt         = p.barrier_function.DT;
    bf.rho        = p.optimization.RHO;
    return bf;
}

/// @brief Generated ROS parameters -> drone model parameters (g keeps its default).
ekf_estimator::ModelParams toModelParams(const cbf::Params& p) {
    ekf_estimator::ModelParams m;
    m.C               = p.model.C;
    m.OMEGA_0         = p.model.omega_0;
    m.ETA             = p.model.ETA;
    m.TAUZ_INV        = p.model.TAUZ_INV;
    m.TAU_CMD         = p.model.TAU_CMD;
    m.INPUT_DELAY_SEC = p.model.INPUT_DELAY_SEC;
    return m;
}


}  // namespace

CbfNode::CbfNode() : Node("cbf_node") {
    param_listener_ = 
        std::make_unique<cbf::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    // convert the ROS parameters once into the types of the core library
    bf_params_ = toBarrierFunctionParams(params_);
    model_.emplace(toModelParams(params_));

    // obstacles of the offline map (static, loaded once), used together with the LiDAR obstacles of every scan
    const std::string map_path = resolveMapPath(params_.path_map);
    // TODO: check for: 2 vertical planes (top and bottom) and at least 3 non-parallel, non-vertical planes -> to be a valid obstacle
    for (const auto& obs : clustering::loadMapObstacles(map_path).map_obstacle) {
        map_obstacles_.push_back(toFilterObstacle(obs));
    }
    RCLCPP_INFO(get_logger(), "loaded %zu map obstacles from %s", map_obstacles_.size(), map_path.c_str());

    // PUBLISHER
    input_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(params_.topics.output.safe_command, rclcpp::QoS(10));

    // SUBSCRIBERS
    input_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        params_.topics.input.nominal_command, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::TwistStamped& msg) {controllerCallback(msg); }
    );

    state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        params_.topics.input.state, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry& msg) {stateCallback(msg); }
    );
    
    constraint_sub_ = create_subscription<collision_avoidance_msgs::msg::HalfPlaneArray>(
        params_.topics.input.constraints, rclcpp::SensorDataQoS(),
        [this](const collision_avoidance_msgs::msg::HalfPlaneArray& msg) {constraintCallback(msg); });

    // TODO: subscribe to the obstacles of the clustering node (message type still to be defined), e.g.
    //   lidar_obstacle_sub_ = create_subscription<...>(params_.topics.input.lidar_obstacles, rclcpp::SensorDataQoS(),
    //       [this](const ...& msg) { lidarObstacleCallback(msg); });
    // and in the callback: msg -> clustering::LidarObstacle -> toFilterObstacle() -> lidar_obstacles_

}

void CbfNode::controllerCallback(const geometry_msgs::msg::TwistStamped& msg) {
    if (!state_) {
        return;  // no state estimate yet: nothing to filter with
    }
    const rclcpp::Time t = now();

    // drop commands the drone already applies (sent more than INPUT_DELAY_SEC ago)
    const rclcpp::Time cutoff = t - rclcpp::Duration::from_seconds(model_->params().INPUT_DELAY_SEC);
    while (!pending_.empty() && pending_.front().first < cutoff) {
        pending_.pop_front();
    }
    std::deque<ekf_estimator::InputVec> u_sent;
    for (const auto& [stamp, u_pending] : pending_) {
        u_sent.push_back(u_pending);
    }

    const ekf_estimator::InputVec u_ref = toInputVec(msg);

    // all obstacles: offline map, clustering node (latest scan), single half-planes (latest scan)
    std::vector<cbf_safety_filter::Obstacle> obstacles;
    obstacles.reserve(map_obstacles_.size() + lidar_obstacles_.size() + halfplane_obstacles_.size());
    obstacles.insert(obstacles.end(), map_obstacles_.begin(), map_obstacles_.end());
    obstacles.insert(obstacles.end(), lidar_obstacles_.begin(), lidar_obstacles_.end());
    obstacles.insert(obstacles.end(), halfplane_obstacles_.begin(), halfplane_obstacles_.end());

    ekf_estimator::InputVec u = u_ref;
    double slack = 0.0;
    int num_rows = 0;
    if (!cbf_safety_filter::safety_filter(*state_, obstacles, u_sent, u_ref, u,
                                            *model_, bf_params_, slack, num_rows)) {
        // TODO: decide on a fallback (u_ref unchanged is sent for now)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "CBF QP failed, sending the nominal command");
    }

    // keep yaw rate
    u(ekf_estimator::YAW_RATE) = u_ref(ekf_estimator::YAW_RATE);

    auto out = toInputMsg(u);
    out.header.stamp = t;
    out.header.frame_id = msg.header.frame_id;

    // pushlish slack on out.twist.angular.x
    out.twist.angular.x = slack;
    // publish num_rows on out.twist.angular.y
    out.twist.angular.y = num_rows;

    input_pub_->publish(out);

    // remember what was sent, for the delay prediction
    // (bag replay: the drone flew the nominal commands, so store u_ref instead of u there)
    pending_.emplace_back(t, u);
}

void CbfNode::constraintCallback(const collision_avoidance_msgs::msg::HalfPlaneArray& msg) {
    halfplane_obstacles_ = toHalfPlaneObstacles(msg);
}

// void CbfNode::clusteringCallback
// void CbfNode::halfplaneCallback

void CbfNode::stateCallback(const nav_msgs::msg::Odometry& msg) {
    state_ = toStateVec(msg);
}
