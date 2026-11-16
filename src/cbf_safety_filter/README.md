# CBF Safety Filter

## Description

ROS 2 node (`cbf_node`, executable `cbf_app`) implementing a Control Barrier Function (CBF) safety filter for a quadrotor
(Parrot Anafi). It takes the nominal command of the controller and changes it as little as possible so that the drone
stays on the safe side of all half-plane constraints (obstacles) published by `scan_to_constraint_node`
(package `pcd_constraints`).

The filter itself is plain C++ without ROS (`cbf_safety_filter_core`: `src/cbf_filter.cpp`, drone model in
`src/ekf/drone_model.cpp`); the node (`src/nodes/cbf_node.cpp`) only converts messages and keeps the latest state,
constraints and sent commands.

Every nominal command runs through these steps:

1. predict the state $\hat x$ at which the new command takes effect (input delay, see [State prediction](#state-prediction))
2. build one linear CBF row per half-plane, in the acceleration $a$
3. solve a QP for the acceleration closest to the one of the nominal command
4. convert the safe acceleration back to a command and publish it

## Barrier function

A half-plane is given by its unit normal $n$ (pointing **towards the obstacle**) and offset $b$; the safe side is
$n^\top p \le b$. With position $p$ and velocity $v$ of the predicted state $\hat x$ (map frame), the barrier function of
one half-plane is

$$
h(x) = b - n^\top p - r_{safe} - t_r \cdot \bar v_n - \frac{\bar v_n^2}{2\,a_{max}},
\qquad v_n = n^\top v, \qquad \bar v_n = \max(v_n, 0)
$$

- $b - n^\top p$: distance of the drone to the plane (positive on the safe side)
- $r_{safe}$ (`R_SAFETY`): safety margin
- $t_r \bar v_n$ (`T_REACTION`): distance travelled before braking starts
- $\bar v_n^2 / (2 a_{max})$: braking distance, with $a_{max} = g \tan(\theta_{max})$ the horizontal acceleration
  reachable at the tilt limit $\theta_{max}$ (`MAX_TILT`)

$h \ge 0$ is safe. Moving away from a plane ($v_n \le 0$) does not reduce $h$.

## CBF condition

The drone model gives the velocity derivative $\dot v = a - d(v)$, with the acceleration from tilt and vertical velocity
command $a$ and the linear drag $d(v) = C\,[v_x, v_y, 0]^\top$ (`C`). For an approaching plane ($v_n > 0$):

$$
\dot h = -v_n - \gamma \, n^\top (a - d(v)), \qquad \gamma = t_r + \frac{v_n}{a_{max}}
$$

The CBF condition $\dot h \ge -\alpha h$ (linear class-$\mathcal K$ function, gain `ALPHA`) is linear in $a$:

$$
-\gamma \, n^\top a \;\ge\; v_n - \gamma \, n^\top d(v) - \alpha \, h(x)
$$

- approaching ($v_n > 0$): the row above
- moving away ($v_n \le 0$) and $h \ge 0$: no row
- moving away and $h < 0$ (already inside the margin): the same row with $\gamma = t_r$, which pushes the drone back out

## Optimization problem

The decision variables are the acceleration and one slack, $z = [a_x, a_y, a_z, s]^\top$ (map frame), **not** the command
itself: the CBF rows are linear in the acceleration. The nominal command is converted to a reference acceleration
$a_{ref}$ (inverse of the conversion below), and the QP (solver: qpmad) is

$$
\begin{aligned}
\min_{a,\,s} \quad & \tfrac12 \lVert a - a_{ref} \rVert^2 + \tfrac12 s^2 + \rho\, s \\
\text{s.t.} \quad & -\gamma_i \, n_i^\top a + s \ge v_{n,i} - \gamma_i \, n_i^\top d(v) - \alpha \, h_i(x) \qquad \text{(one row per active plane } i) \\
& -a_{max} \le a_x, a_y \le a_{max} \\
& (-v_{z,max} - v_z)\,\tau_z^{-1} \le a_z \le (v_{z,max} - v_z)\,\tau_z^{-1} \\
& s \ge 0
\end{aligned}
$$

i.e. $\min_z \tfrac12 z^\top H z + f^\top z$ with $H = I_4$ and $f = [-a_{ref}^\top, \rho]^\top$. The slack $s$ relaxes
all rows by the same amount and keeps the QP feasible when the constraints conflict with the input limits; $\rho$
(`RHO`) sets how strongly that is penalised. $s > 0$ means the constraints could not all be met.

The safe acceleration is converted back to a command (and clamped to the limits):

$$
\theta_{pitch} = \arctan\frac{a_x}{g}, \qquad \phi_{roll} = -\arctan\frac{a_y}{g}, \qquad
v_{z,cmd} = v_z + \frac{a_z}{\tau_z^{-1}}
$$

with $\tau_z^{-1}$ = `TAUZ_INV` (first-order vertical velocity response) and $v_z$ from the predicted state.

## State prediction

Commands take `INPUT_DELAY_SEC` to reach the drone, so the command computed now acts on a future state. The node keeps
the commands it sent during the last `INPUT_DELAY_SEC` (`pending_`, oldest first). Before each solve, the current
state estimate is integrated over them with the full drone model, one `DroneModel::rk4Step()` of length `DT` per
pending command; the barrier functions are evaluated on this predicted state $\hat x$.

## Interfaces

### Subscribed topics

| Parameter | Default topic | Type | Content |
|---|---|---|---|
| `topics.input.constraints` | `/collision_avoidance/constraints` | `collision_avoidance_msgs/HalfPlaneArray` | half-plane constraints (`shape_msgs/Plane[]`), replaced on every message |
| `topics.input.state` | `/filter/state_estimate` | `nav_msgs/Odometry` | EKF state estimate |
| `topics.input.nominal_command` | `/control_input/mux` | `geometry_msgs/TwistStamped` | nominal command of the controller; every message triggers one filter step |

All subscriptions use the sensor-data QoS (best effort).

- **Constraints:** a plane `coef = [n_x, n_y, n_z, d]` is the half-plane $n^\top p \le -d$, i.e. $b = -d$, with $n$ a unit
  vector pointing towards the obstacle (as `scan_to_constraint_node` publishes them). The planes must be in the same
  frame as the state estimate (`map`); the message's `frame_id` is not checked.
- **State:** converted to the 13-entry `ekf_estimator::StateVec` (`Eigen::Matrix<double, 13, 1>`, layout in
  `include/cbf_safety_filter/ekf/types.hpp`): position and linear velocity (both in the map frame), roll and pitch
  from the orientation, their rates from the angular velocity, yaw. The lagged commands `uf_roll` / `uf_pitch` are not
  in the message and are set to the current roll / pitch.
- **Nominal command:** converted to `ekf_estimator::InputVec` (`Eigen::Matrix<double, 4, 1>`):

  | Command | Field |
  |---|---|
  | pitch [rad] | `twist.linear.x` |
  | roll [rad] | `twist.linear.y` |
  | vertical velocity [m/s] | `twist.linear.z` |
  | yaw rate [rad/s] | `twist.angular.z` |

  Commands that arrive before the first state estimate are dropped (nothing is published).

### Published topics

| Parameter | Default topic | Type | Content |
|---|---|---|---|
| `topics.output.safe_command` | `/collision_avoidance/safe_command` | `geometry_msgs/TwistStamped` | safe command, same field layout as the nominal command |

- the yaw rate is passed through unchanged (the filter only acts on roll, pitch and vertical velocity)
- `twist.angular.x` carries the QP slack $s$ (0 = all constraints met)
- `header.stamp` is the time of the filter step, `header.frame_id` is copied from the nominal command

## Parameters

Values in `config/cbf.yaml` (loaded by the launch file), schema with defaults and validation in `src/cbf_params.yaml`
(generate_parameter_library). The topic names are read-only, all parameters are read once at startup.

| Parameter | Default | Unit | Meaning |
|---|---|---|---|
| `barrier_function.R_SAFETY` | 0.25 | m | safety margin $r_{safe}$ to every plane |
| `barrier_function.T_REACTION` | 0.13 | s | reaction time $t_r$ |
| `barrier_function.ALPHA` | 1.0 | 1/s | CBF gain $\alpha$ |
| `barrier_function.MAX_TILT` | 0.4363 | rad | roll / pitch limit $\theta_{max}$ (25°), gives $a_{max}$ |
| `barrier_function.MAX_V_Z` | 6.0 | m/s | vertical velocity limit $v_{z,max}$ |
| `barrier_function.DT` | 0.0667 | s | command period (15 Hz): length of each prediction step |
| `optimization.RHO` | 1000.0 | – | slack weight $\rho$ |
| `model.C` | 0.26 | 1/s | horizontal drag coefficient |
| `model.omega_0` | 18.89 | rad/s | tilt servo natural frequency |
| `model.ETA` | 0.921 | – | tilt servo damping (= 2ζ) |
| `model.TAUZ_INV` | 2.0 | 1/s | inverse time constant of the vertical velocity response $\tau_z^{-1}$ |
| `model.TAU_CMD` | 0.08 | s | time constant of the roll / pitch command lag |
| `model.INPUT_DELAY_SEC` | 0.10 | s | delay from sending a command to the drone reacting |

## Usage

Build (from the workspace root):

```bash
colcon build --packages-up-to cbf_safety_filter
source install/setup.bash
```

Run the whole pipeline (`scan_to_constraint_node` + `cbf_node`), by default on simulated time from a bag played with
`--clock`:

```bash
ros2 launch cbf_safety_filter collision_avoidance.launch.py
# arguments: use_sim_time (default true), cbf_config (default cbf.yaml), scan_to_constraint_config
```

Run only the filter:

```bash
ros2 run cbf_safety_filter cbf_app --ros-args \
  --params-file $(ros2 pkg prefix cbf_safety_filter)/share/cbf_safety_filter/config/cbf.yaml
```

## Current limitations

- **Yaw:** the conversion between command and acceleration ignores the heading: roll / pitch produce the acceleration in
  the yaw-aligned body frame (the drone model rotates it by the yaw into the map frame), but the QP treats it as the
  map-frame acceleration of the half-planes. This is only exact at yaw ≈ 0.
- **QP failure:** if the QP cannot be solved, the nominal command is published unchanged (warning, throttled to 1 s).
- **No constraints yet:** until the first constraint message arrives, the nominal command passes unfiltered (only the
  input limits apply).
