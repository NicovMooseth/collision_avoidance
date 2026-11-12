// Implementation of scan_to_constraint_node: read parameters, pick the algorithm,
// convert ROS messages <-> library types and publish. No algorithm logic belongs here.
#include "nodes/scan_to_constraint_node.hpp"

#include "collision_avoidance/clustering/halfplane_algorithm.hpp"


ScanToConstraintNode::ScanToConstraintNode() : Node("scan_to_constraint_node") {
    param_listener_ = 
        std::make_unique<scan_to_constraint::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    constraints_pub_ = create_publisher<collision_avoidance_msgs::msg::HalfPlaneArray>(params_.topics.output.constraints, rclcpp::QoS(10));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        params_.topics.input.points, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2& cloud) { cloudCallback(cloud); }
    );
}

void ScanToConstraintNode::cloudCallback(const sensor_msgs::msg::PointCloud2& cloud) {
    std::vector<Eigen::Vector3f> points;
    points.reserve(static_cast<size_t>(cloud.width) * cloud.height);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)){
            points.emplace_back(*x, *y, *z);
        }
    }

    const auto planes = collision_avoidance::computeConstraints(
        points, static_cast<float>(params_.range.min), static_cast<float>(params_.range.max)
    );

    collision_avoidance_msgs::msg::HalfPlaneArray result;
    result.header = cloud.header;
    result.planes.reserve(planes.size());
    for (const auto& p : planes) {
        shape_msgs::msg::Plane plane;
        plane.coef[0] = p.normal.x();
        plane.coef[1] = p.normal.y();
        plane.coef[2] = p.normal.z();
        plane.coef[3] = -p.offset;
        result.planes.push_back(plane);
    }
    constraints_pub_->publish(result);

}