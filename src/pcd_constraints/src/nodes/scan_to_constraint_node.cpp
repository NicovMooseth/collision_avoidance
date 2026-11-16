// Implementation of scan_to_constraint_node: read parameters, pick the algorithm,
// convert ROS messages <-> library types and publish. No algorithm logic belongs here.
#include "pcd_constraints/nodes/scan_to_constraint_node.hpp"

#include <cmath>
#include <limits>

#include "pcd_constraints/clustering/halfplane_algorithm.hpp"

ScanToConstraintNode::ScanToConstraintNode() : Node("scan_to_constraint_node") {
    param_listener_ =
        std::make_unique<scan_to_constraint::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    // sensor -> body extrinsic; fixed_size validation guarantees 9 and 3 entries
    const auto& r = params_.extrinsics.fast_lio_to_body.rotation;
    const auto& t = params_.extrinsics.fast_lio_to_body.translation;
    R_BS_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(r.data());  // file is row-major
    t_BS_ = Eigen::Vector3d(t[0], t[1], t[2]);

    constraints_pub_ = create_publisher<collision_avoidance_msgs::msg::HalfPlaneArray>(params_.topics.output.constraints, rclcpp::QoS(10));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        params_.topics.input.points, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2& cloud) { cloudCallback(cloud); }
    );

    if (params_.tf.publish) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    }

    state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        params_.topics.input.state, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry& msg) { stateCallback(msg); }
    );
}

void ScanToConstraintNode::stateCallback(const nav_msgs::msg::Odometry& msg) {
    const auto& p = msg.pose.pose.position;
    const auto& q = msg.pose.pose.orientation;

    Eigen::Isometry3d T_WB = Eigen::Isometry3d::Identity();
    T_WB.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    T_WB.translation() = Eigen::Vector3d(p.x, p.y, p.z);

    const rclcpp::Time stamp(msg.header.stamp);
    pose_buffer_.emplace_back(stamp, T_WB);
    world_frame_ = msg.header.frame_id;
    body_frame_ = msg.child_frame_id;

    // world -> body for TF (e.g. RViz): the odometry pose as it is
    if (tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header = msg.header;
        tf.child_frame_id = msg.child_frame_id;
        tf.transform.translation.x = p.x;
        tf.transform.translation.y = p.y;
        tf.transform.translation.z = p.z;
        tf.transform.rotation = q;
        tf_broadcaster_->sendTransform(tf);
    }

    // keep only the last state.buffer_duration seconds (relative to the newest state, not to now())
    const rclcpp::Time cutoff = stamp - rclcpp::Duration::from_seconds(params_.state.buffer_duration);
    while (!pose_buffer_.empty() && pose_buffer_.front().first < cutoff) {
        pose_buffer_.pop_front();
    }
}

void ScanToConstraintNode::publishStaticSensorTf(const std::string& sensor_frame) {
    if (!static_tf_broadcaster_ || static_tf_sent_ || body_frame_.empty()) {
        return;
    }
    // body -> sensor: the extrinsic T_BS (sensor pose in the body frame)
    const Eigen::Quaterniond q_BS(R_BS_);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = body_frame_;
    tf.child_frame_id = sensor_frame;
    tf.transform.translation.x = t_BS_.x();
    tf.transform.translation.y = t_BS_.y();
    tf.transform.translation.z = t_BS_.z();
    tf.transform.rotation.w = q_BS.w();
    tf.transform.rotation.x = q_BS.x();
    tf.transform.rotation.y = q_BS.y();
    tf.transform.rotation.z = q_BS.z();
    static_tf_broadcaster_->sendTransform(tf);
    static_tf_sent_ = true;
}

std::optional<Eigen::Isometry3d> ScanToConstraintNode::poseAt(const rclcpp::Time& stamp) const {
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

std::vector<pcd_constraints::HalfPlane> ScanToConstraintNode::transformToMap(
        const std::vector<pcd_constraints::HalfPlane>& planes, const Eigen::Isometry3d& T_WB) const {
    // sensor -> world: x_W = R_WS x_S + p_WS
    const Eigen::Matrix3d R_WS = T_WB.linear() * R_BS_;
    const Eigen::Vector3d p_WS = T_WB.linear() * t_BS_ + T_WB.translation();

    // plane n_S·x_S <= d_S  ->  n_W·x_W <= d_W  with  n_W = R_WS n_S,  d_W = d_S + n_W·p_WS
    std::vector<pcd_constraints::HalfPlane> planes_W;
    planes_W.reserve(planes.size());
    for (const auto& plane : planes) {
        pcd_constraints::HalfPlane plane_W;
        plane_W.normal = R_WS * plane.normal;   // rotation keeps it unit length
        plane_W.offset = plane.offset + plane_W.normal.dot(p_WS);
        planes_W.push_back(plane_W);
    }
    return planes_W;
}

void ScanToConstraintNode::cloudCallback(const sensor_msgs::msg::PointCloud2& cloud) {
    publishStaticSensorTf(cloud.header.frame_id);  // once, the sensor frame name comes from the cloud

    // the constraints are published in the world frame, so the pose at scan time is needed first
    const auto T_WB = poseAt(rclcpp::Time(cloud.header.stamp));
    if (!T_WB) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No state estimate close to the scan time, scan skipped");
        return;
    }

    std::vector<Eigen::Vector3f> points;
    points.reserve(static_cast<size_t>(cloud.width) * cloud.height);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)){
            points.emplace_back(*x, *y, *z);
        }
    }

    const auto planes_S = pcd_constraints::computeConstraints(
        points, static_cast<float>(params_.range.min), static_cast<float>(params_.range.max)
    );

    const auto planes_W = transformToMap(planes_S, *T_WB);

    collision_avoidance_msgs::msg::HalfPlaneArray result;
    result.header.stamp = cloud.header.stamp;
    result.header.frame_id = world_frame_;
    result.planes.reserve(planes_W.size());
    for (const auto& p : planes_W) {
        shape_msgs::msg::Plane plane;
        plane.coef[0] = p.normal.x();
        plane.coef[1] = p.normal.y();
        plane.coef[2] = p.normal.z();
        plane.coef[3] = -p.offset;
        result.planes.push_back(plane);
    }
    constraints_pub_->publish(result);
}
