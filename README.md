# collision_avoidance

ROS 2 Jazzy workspace for LiDAR-based drone collision avoidance.

- `src/collision_avoidance`: `scan_to_constraint_node` (point cloud → half-plane constraints) and `cbf_node` (CBF safety filter)
- `src/collision_avoidance_msgs`: `HalfPlaneArray` message
- `src/qpmad`: QP solver, git submodule

## Build

```bash
git clone --recurse-submodules <repo-url> ros2_ws
cd ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build
```
