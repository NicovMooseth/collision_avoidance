# Pointcloud Constraints for CBF Filter

## Description

ROS 2 node (`scan_to_constraint_node`, executable `scan_to_constraint_app`) that computes half-plane constraints from
every LiDAR scan and publishes them, in the world frame of the state estimate, on `/collision_avoidance/constraints`.
`cbf_node` (package `cbf_safety_filter`) uses them directly as its obstacle constraints.

The algorithm is plain C++ without ROS (library `pcd_constraints_core`: `computeConstraints()` in
`src/clustering/halfplane_algorithm.cpp`); the `HalfPlane` type in `include/pcd_constraints/constraints.hpp` is shared with
`cbf_safety_filter`. The node (`src/nodes/scan_to_constraint_node.cpp`) only converts messages, looks up the pose and
transforms the planes.

For every scan, in the sensor frame (LiDAR at the origin):

1. Find the remaining point $q$ closest to the LiDAR, among the points within `range.min` … `range.max`.
2. Put a plane through $q$, perpendicular to the ray from the LiDAR to $q$: the half-plane $n^\top p \le b$ with
   $n = q / \lVert q \rVert$ (unit, pointing away from the LiDAR, towards the obstacle) and $b = \lVert q \rVert$.
3. Remove all points on or behind this plane ($n^\top p \ge b - 10^{-4}$); only the points between the LiDAR and the
   plane remain.
4. Repeat from 1 until no point within the range is left.
5. Transform the planes into the world frame with the drone pose at the scan time and the sensor → body extrinsic:
   $n_W = R_{WS}\, n_S$, $b_W = b_S + n_W^\top p_{WS}$ (with $R_{WS} = R_{WB} R_{BS}$ and
   $p_{WS} = R_{WB}\, t_{BS} + p_{WB}$).

The intersection of all half-planes is a convex region around the LiDAR that contains none of the scan points in range.

## Interfaces

### Subscribed topics

| Parameter | Default topic | Type | Content |
|---|---|---|---|
| `topics.input.points` | `/rms2/lidar/points` | `sensor_msgs/PointCloud2` | LiDAR scan; every message triggers one constraint computation |
| `topics.input.state` | `/filter/state_estimate` | `nav_msgs/Odometry` | drone pose (body → world), used to transform the constraints into the world frame |

Both subscriptions use the sensor-data QoS (best effort).

- **Point cloud:** only the `x`, `y`, `z` fields (`float32`) are read, in the sensor frame (`header.frame_id`). Non-finite
  points are skipped; points closer than `range.min` or farther than `range.max` are not used. `header.stamp` is the
  scan time used to look up the pose.
- **State:** only the pose is used (position and orientation). `header.frame_id` is the world frame (`map`),
  `child_frame_id` the body frame (`base_link`). The poses of the last `state.buffer_duration` seconds are kept; each
  scan uses the pose nearest in time to its stamp (not interpolated). If none lies within `state.max_time_offset`, the
  scan is skipped (warning, throttled to 1 s). The sensor → body transform comes from the parameter
  `extrinsics.fast_lio_to_body`.

### Published topics

| Parameter | Default topic | Type | Content |
|---|---|---|---|
| `topics.output.constraints` | `/collision_avoidance/constraints`* | `collision_avoidance_msgs/HalfPlaneArray` | half-plane constraints of one scan, in the world frame |

\* value in `config/scan_to_constraint.yaml`; the schema default in `src/scan_to_constraint_params.yaml` is the relative
name `collision_avoidance/constraints`, which only resolves to the same topic when the node runs without a namespace.

Published with QoS depth 10 (reliable), one message per processed scan.

- **`HalfPlaneArray`:** `std_msgs/Header header`, `shape_msgs/Plane[] planes`.
- **Header:** `stamp` = stamp of the point cloud, `frame_id` = world frame (the `frame_id` of the state estimate).
- **Planes:** `coef = [n_x, n_y, n_z, -b]` is the half-plane $n^\top p \le b$ with $n$ a unit vector pointing towards
  the obstacle (away from the LiDAR) and $b$ the plane's offset in the world frame; the safe side is
  `coef[0]*x + coef[1]*y + coef[2]*z + coef[3] <= 0`. This is the convention `cbf_node` expects.

### TF (only if `tf.publish` is true)

| Transform | Topic | Source |
|---|---|---|
| world → body (dynamic) | `/tf` | the pose of every state message, with its stamp |
| body → sensor (static) | `/tf_static` | `extrinsics.fast_lio_to_body`, sent once at the first scan after the first state (both frame names known) |

## Parameters

Values in `config/scan_to_constraint.yaml` (loaded by the launch file), schema with defaults and validation in
`src/scan_to_constraint_params.yaml` (generate_parameter_library). All parameters are read once at startup; changing
them at runtime has no effect.

| Parameter | Default | Config value | Unit | Meaning |
|---|---|---|---|---|
| `topics.input.points` | `/rms2/lidar/points` | same | – | point cloud topic (read-only) |
| `topics.input.state` | `/filter/state_estimate` | same | – | state estimate topic (read-only) |
| `topics.output.constraints` | `collision_avoidance/constraints` | `/collision_avoidance/constraints` | – | constraint topic (read-only) |
| `range.min` | 0.3 | 0.5 | m | closer points are not used |
| `range.max` | 30.0 | 30.0 | m | farther points are not used |
| `tf.publish` | true | true | – | publish world → body and body → sensor on TF (read-only); turn off if another node already publishes them |
| `state.buffer_duration` | 1.0 | 1.0 | s | how long past poses are kept (clouds arrive ~0.12 s after their stamp) |
| `state.max_time_offset` | 0.05 | 0.05 | s | maximum time between the scan stamp and the nearest pose; otherwise the scan is skipped |
| `extrinsics.fast_lio_to_body.translation` | `[0, 0, 0]` | `[0, 0, 0]` | m | sensor position in the body frame $t_{BS}$ (read-only) |
| `extrinsics.fast_lio_to_body.rotation` | identity | JT128 extrinsic | – | rotation sensor → body $R_{BS}$, row-major 3×3 (read-only) |

## Usage

Build (from the workspace root):

```bash
colcon build --packages-up-to pcd_constraints
source install/setup.bash
```

The launch file for the whole pipeline (`scan_to_constraint_node` + `cbf_node`) is in `cbf_safety_filter`; by default
it runs on simulated time from a bag played with `--clock`:

```bash
ros2 launch cbf_safety_filter collision_avoidance.launch.py
# arguments: use_sim_time (default true), scan_to_constraint_config (default scan_to_constraint.yaml), cbf_config
```

Run only this node:

```bash
ros2 run pcd_constraints scan_to_constraint_app --ros-args \
  --params-file $(ros2 pkg prefix pcd_constraints)/share/pcd_constraints/config/scan_to_constraint.yaml \
  -p use_sim_time:=true   # when playing a bag with --clock
```

## Current limitations

- **One pose per scan:** the whole scan is transformed with the single pose nearest to its stamp (not interpolated);
  the motion of the drone during the scan is not compensated.
- **Number of planes:** there is no limit; every iteration searches all remaining points, so the cost grows with
  (points × planes). Isolated points (noise, small objects) each give their own plane.
- **No filtering of the points:** every point in range can create a plane, including the floor, the ceiling and noise;
  only `range.min` keeps the drone's own body out.
- **No memory:** each message holds only the constraints of the latest scan, and `cbf_node` replaces its constraints on
  every message, so obstacles outside the LiDAR's current field of view are not constrained.
