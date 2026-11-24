#include "nodes/clustering_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <shape_msgs/msg/plane.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <string>

namespace {

/// @brief map.path -> absolute path: a relative path is relative to the share directory of map_obstacles
///        (its maps/ folder is installed there), an absolute path is returned unchanged.
std::string resolveMapPath(const std::string& path) {
    const std::filesystem::path p(path);
    if (p.is_absolute()) {
        return path;
    }
    return (std::filesystem::path(ament_index_cpp::get_package_share_directory("map_obstacles")) / p).string();
}

/// @brief The finite points of the cloud with a range in [min_range, max_range], in the frame of the cloud
///        (sensor frame, so the range is the distance to the LiDAR). The minimum range drops the returns without
///        echo, which the driver sends as (0, 0, 0) (about 2 % of each scan in the office bag), and hits on the
///        drone itself.
std::vector<Eigen::Vector3d> cloudToPoints(const sensor_msgs::msg::PointCloud2& cloud, const double min_range,
                                          const double max_range) {
    const double min_squared = min_range * min_range;
    const double max_squared = max_range * max_range;
    std::vector<Eigen::Vector3d> points;
    points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
            continue;
        }
        const Eigen::Vector3d p(*x, *y, *z);
        const double range_squared = p.squaredNorm();
        if (range_squared >= min_squared && range_squared <= max_squared) {
            points.push_back(p);
        }
    }
    return points;
}

/// @brief Stamp in seconds (as used by the clustering library) -> ROS time.
builtin_interfaces::msg::Time toTimeMsg(const double seconds) {
    return rclcpp::Time(static_cast<std::int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME);
}

/// @brief Points as a cloud with fields x, y, z (float32).
sensor_msgs::msg::PointCloud2 toCloud(const std_msgs::msg::Header& header, const std::vector<Eigen::Vector3d>& points) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    // the first argument is the number of fields that follow (name, count, type each)
    modifier.setPointCloud2Fields(3,
                                  "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const Eigen::Vector3d& p : points) {
        *x = static_cast<float>(p.x());
        *y = static_cast<float>(p.y());
        *z = static_cast<float>(p.z());
        ++x; ++y; ++z;
    }
    return cloud;
}

/// @brief All cluster points in one cloud with fields x, y, z, track_id (float32), so RViz can color them by
///        track ID (PointCloud2 display, color transformer "Intensity", channel "track_id").
sensor_msgs::msg::PointCloud2 toClusterCloud(const std_msgs::msg::Header& header,
                                             const std::vector<std::vector<Eigen::Vector3d>>& clusters,
                                             const std::vector<int>& cluster_track_ids) {
    std::size_t num_points = 0;
    for (const auto& cluster : clusters) {
        num_points += cluster.size();
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                  "track_id", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(num_points);

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"), id(cloud, "track_id");
    for (std::size_t c = 0; c < clusters.size(); ++c) {
        for (const Eigen::Vector3d& p : clusters[c]) {
            *x = static_cast<float>(p.x());
            *y = static_cast<float>(p.y());
            *z = static_cast<float>(p.z());
            *id = static_cast<float>(cluster_track_ids[c]);
            ++x; ++y; ++z; ++id;
        }
    }
    return cloud;
}

collision_avoidance_msgs::msg::LidarObstacleArray toObstacleArray(const std_msgs::msg::Header& header,
                                                                   const clustering::LidarObstacles& tracks) {
    collision_avoidance_msgs::msg::LidarObstacleArray msg;
    msg.header = header;
    msg.obstacles.reserve(tracks.lidar_obstacles.size());
    for (const clustering::LidarObstacle& track : tracks.lidar_obstacles) {
        if (track.history.empty()) {
            continue;
        }
        collision_avoidance_msgs::msg::LidarObstacle obstacle;
        obstacle.id = track.ID;
        obstacle.motion = track.motion == 1 ? collision_avoidance_msgs::msg::LidarObstacle::LINEAR
                                            : collision_avoidance_msgs::msg::LidarObstacle::STATIC;
        obstacle.last_seen = toTimeMsg(track.last_seen);
        const Eigen::Vector3d& center = track.history.back().center;
        obstacle.center.x = center.x();
        obstacle.center.y = center.y();
        obstacle.center.z = center.z();
        obstacle.velocity.x = track.v.x();
        obstacle.velocity.y = track.v.y();
        obstacle.velocity.z = track.v.z();
        obstacle.planes.reserve(track.planes.size());
        for (const clustering::HalfPlane& plane : track.planes) {
            shape_msgs::msg::Plane plane_msg;
            plane_msg.coef = {plane.normal.x(), plane.normal.y(), plane.normal.z(), plane.offset};
            obstacle.planes.push_back(plane_msg);
        }
        msg.obstacles.push_back(std::move(obstacle));
    }
    return msg;
}

/// @brief CUBE marker of a box from fitAlignedBox: 6 planes, normals out of the box, in the order
///        +u, -u, +w, -w (vertical sides), +z, -z. Returns false for any other shape.
bool toBoxMarker(const std::vector<clustering::HalfPlane>& planes, visualization_msgs::msg::Marker& marker) {
    if (planes.size() != 6) {
        return false;
    }
    // free side n·p + offset >= 0: the face at s_max has n = +axis, offset = -s_max; at s_min n = -axis, offset = s_min
    const Eigen::Vector3d u = planes[0].normal;
    const Eigen::Vector3d w = planes[2].normal;
    const double u_max = -planes[0].offset, u_min = planes[1].offset;
    const double w_max = -planes[2].offset, w_min = planes[3].offset;
    const double z_max = -planes[4].offset, z_min = planes[5].offset;

    const Eigen::Vector3d center = u * 0.5 * (u_min + u_max) + w * 0.5 * (w_min + w_max) +
                                   Eigen::Vector3d::UnitZ() * 0.5 * (z_min + z_max);
    const Eigen::Quaterniond orientation(Eigen::AngleAxisd(std::atan2(u.y(), u.x()), Eigen::Vector3d::UnitZ()));

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();
    marker.pose.orientation.w = orientation.w();
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.scale.x = u_max - u_min;
    marker.scale.y = w_max - w_min;
    marker.scale.z = z_max - z_min;
    return true;
}

/// @brief Per track: box (static blue, moving red; faded if not matched in this scan), velocity arrow (1 s ahead,
///        moving tracks only) and ID text. Starts with DELETEALL, so markers of deleted tracks disappear.
visualization_msgs::msg::MarkerArray toMarkers(const std_msgs::msg::Header& header,
                                               const clustering::LidarObstacles& tracks, const double scan_time) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    for (const clustering::LidarObstacle& track : tracks.lidar_obstacles) {
        if (track.history.empty()) {
            continue;
        }
        const bool moving = track.motion == 1;
        const bool seen_now = std::abs(track.last_seen - scan_time) < 1e-6;
        const Eigen::Vector3d& center = track.history.back().center;

        visualization_msgs::msg::Marker box;
        box.header = header;
        box.ns = "boxes";
        box.id = track.ID;
        box.action = visualization_msgs::msg::Marker::ADD;
        if (toBoxMarker(track.planes, box)) {
            box.color.r = moving ? 0.9f : 0.2f;
            box.color.g = 0.3f;
            box.color.b = moving ? 0.2f : 0.9f;
            box.color.a = seen_now ? 0.5f : 0.15f;
            markers.markers.push_back(box);
        }

        if (moving) {
            visualization_msgs::msg::Marker arrow;
            arrow.header = header;
            arrow.ns = "velocity";
            arrow.id = track.ID;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.pose.orientation.w = 1.0;
            geometry_msgs::msg::Point start, end;
            start.x = center.x(); start.y = center.y(); start.z = center.z();
            end.x = center.x() + track.v.x(); end.y = center.y() + track.v.y(); end.z = center.z() + track.v.z();
            arrow.points = {start, end};
            arrow.scale.x = 0.05;  // shaft diameter [m]
            arrow.scale.y = 0.1;   // head diameter [m]
            arrow.scale.z = 0.1;   // head length [m]
            arrow.color.r = 0.9f; arrow.color.g = 0.3f; arrow.color.b = 0.2f; arrow.color.a = 1.0f;
            markers.markers.push_back(arrow);
        }

        visualization_msgs::msg::Marker text;
        text.header = header;
        text.ns = "ids";
        text.id = track.ID;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = center.x();
        text.pose.position.y = center.y();
        text.pose.position.z = center.z() + (box.scale.z > 0.0 ? 0.5 * box.scale.z : 0.0) + 0.2;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.2;  // text height [m]
        text.color.r = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f; text.color.a = 1.0f;
        char label[64];
        std::snprintf(label, sizeof(label), moving ? "%d  %.2f m/s" : "%d", track.ID, track.v.norm());
        text.text = label;
        markers.markers.push_back(text);
    }
    return markers;
}

}  // namespace

ClusteringNode::ClusteringNode() : Node("clustering_node") {
    param_listener_ =
        std::make_unique<clustering::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    // parameters of the clustering library (plain C++); validation guarantees positive values
    clustering_params_.eps = params_.dbscan.eps;
    clustering_params_.min_points = static_cast<std::size_t>(params_.dbscan.min_points);
    clustering_params_.num_measurements = static_cast<std::size_t>(params_.history.num_measurements);
    clustering_params_.min_measurements = static_cast<std::size_t>(params_.history.min_measurements);
    clustering_params_.max_static_speed = params_.history.max_static_speed;
    clustering_params_.max_association_distance = params_.history.max_association_distance;
    clustering_params_.known_obstacle_margin = params_.map_removal.margin;

    // sensor -> body extrinsic; fixed_size validation guarantees 9 and 3 entries
    const auto& r = params_.extrinsics.fast_lio_to_body.rotation;
    const auto& t = params_.extrinsics.fast_lio_to_body.translation;
    T_BS_ = Eigen::Isometry3d::Identity();
    T_BS_.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(r.data());  // file is row-major
    T_BS_.translation() = Eigen::Vector3d(t[0], t[1], t[2]);

    // obstacles of the offline map: map.yaml gives the parameters, the CSV the half-planes
    map_loader_ =
        std::make_unique<clustering_map::ParamListener>(get_node_parameters_interface());
    map_params_ = map_loader_->get_params();
    const std::string map_path = resolveMapPath(map_params_.map.path);
    obstacles_ = map_obstacles::loadMapObstacles(map_path);
    RCLCPP_INFO(get_logger(), "Loaded %zu obstacles of the offline map from %s",
                obstacles_.map_obstacle.size(), map_path.c_str());

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        params_.topics.input.points, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2& cloud) { cloudCallback(cloud); }
    );

    state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        params_.topics.input.state, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry& msg) { stateCallback(msg); }
    );

    // PUBLISHING
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(params_.topics.output.points, rclcpp::QoS(10));
    cluster_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(params_.topics.output.clusters, rclcpp::QoS(10));
    obstacle_pub_ = create_publisher<collision_avoidance_msgs::msg::LidarObstacleArray>(
        params_.topics.output.obstacles, rclcpp::QoS(10));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(params_.topics.output.markers, rclcpp::QoS(10));
}

std::optional<Eigen::Isometry3d> ClusteringNode::poseAt(const rclcpp::Time& stamp) const {
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

void ClusteringNode::transformToWorld(std::vector<Eigen::Vector3d>& points, const Eigen::Isometry3d& T_WB) const {
    // sensor -> world: x_W = R_WS x_S + p_WS, with T_WS = T_WB T_BS
    const Eigen::Isometry3d T_WS = T_WB * T_BS_;
    const Eigen::Matrix3d R_WS = T_WS.linear();
    const Eigen::Vector3d p_WS = T_WS.translation();
    for (Eigen::Vector3d& point : points) {
        point = R_WS * point + p_WS;
    }
}

void ClusteringNode::cloudCallback(const sensor_msgs::msg::PointCloud2& cloud) {
    using namespace clustering;
    // the map is in the world frame, so the pose at scan time is needed first
    const auto T_WB = poseAt(rclcpp::Time(cloud.header.stamp));
    if (!T_WB) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No state estimate close to the scan time, scan skipped");
        return;
    }

    std::vector<Eigen::Vector3d> points = cloudToPoints(cloud, params_.range.min, params_.range.max);
    transformToWorld(points, *T_WB); // transform to world frame

    // scan time in seconds
    const double scan_time = rclcpp::Time(cloud.header.stamp).seconds();

    // delete tracks not seen for track_timeout (relative to the scan stamp)
    auto& tracks = tracks_.lidar_obstacles;
    const double timeout = params_.history.track_timeout;
    tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
                                [&](const LidarObstacle& track) { return scan_time - track.last_seen > timeout; }),
                 tracks.end());
                 
    // points of known obstacles out, cluster the rest, match the clusters to the tracks
    removeKnownPoints(obstacles_, points, clustering_params_.known_obstacle_margin);
    const std::vector<int> labels = dbscan_fast(points, clustering_params_);
    const std::vector<std::vector<Eigen::Vector3d>> clusters = splitClusters(points, labels);
    const std::vector<int> cluster_track_ids =
        associateClusters(tracks_, clusters, scan_time, clustering_params_, next_track_id_);

    std_msgs::msg::Header header;
    header.stamp = cloud.header.stamp;
    header.frame_id = world_frame_;
    publishPoints(header, points);  // all points not removed by the map (input of DBSCAN)
    publishResults(header, clusters, cluster_track_ids);
}

void ClusteringNode::publishPoints(const std_msgs::msg::Header& header, const std::vector<Eigen::Vector3d>& points) {
    points_pub_->publish(toCloud(header, points));
}

void ClusteringNode::publishResults(const std_msgs::msg::Header& header,
                                    const std::vector<std::vector<Eigen::Vector3d>>& clusters,
                                    const std::vector<int>& cluster_track_ids) {
    cluster_pub_->publish(toClusterCloud(header, clusters, cluster_track_ids));
    obstacle_pub_->publish(toObstacleArray(header, tracks_));
    marker_pub_->publish(toMarkers(header, tracks_, rclcpp::Time(header.stamp).seconds()));
}

void ClusteringNode::stateCallback(const nav_msgs::msg::Odometry& msg) {
    const auto& p = msg.pose.pose.position;
    const auto& q = msg.pose.pose.orientation;

    Eigen::Isometry3d T_WB = Eigen::Isometry3d::Identity();
    T_WB.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    T_WB.translation() = Eigen::Vector3d(p.x, p.y, p.z);

    const rclcpp::Time stamp(msg.header.stamp);
    pose_buffer_.emplace_back(stamp, T_WB);
    world_frame_ = msg.header.frame_id;

    // keep only the last state.buffer_duration seconds (relative to the newest state, not to now())
    const rclcpp::Time cutoff = stamp - rclcpp::Duration::from_seconds(params_.state.buffer_duration);
    while (!pose_buffer_.empty() && pose_buffer_.front().first < cutoff) {
        pose_buffer_.pop_front();
    }
}
