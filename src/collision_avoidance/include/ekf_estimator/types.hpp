#pragma once

#include <Eigen/Core>

namespace ekf_estimator {

/// @brief Fixed-size types for the 13-state Anafi quadrotor EKF.
///
/// State layout (world frame for pos/vel, body-internal for servo states):
///   x = [px, py, pz, vx, vy, vz, d_roll, d_pitch, w_roll, w_pitch, yaw,
///        uf_roll, uf_pitch]
///
/// Input layout (matches geometry_msgs/Twist via the legacy convention):
///   u = [u_roll, u_pitch, yaw_rate, u_vel_z]
///        Twist.linear.y, .linear.x, .angular.z, .linear.z
using StateVec = Eigen::Matrix<double, 13, 1>;
using InputVec = Eigen::Matrix<double, 4, 1>;
using StateMat = Eigen::Matrix<double, 13, 13>;

constexpr int kStateDim = 13;
constexpr int kInputDim = 4;

/// @brief Named indices into the state vector.
///
/// New states are appended, never inserted: the values below are relied on by
/// the hand-built measurement Jacobians and the covariance mapping in the node.
enum StateIdx : int {
  PX = 0,
  PY = 1,
  PZ = 2,
  VX = 3,
  VY = 4,
  VZ = 5,
  DROLL = 6,
  DPITCH = 7,
  WROLL = 8,
  WPITCH = 9,
  YAW = 10,
  UF_ROLL = 11,
  UF_PITCH = 12,
};

/// @brief Named indices into the control-input vector.
enum InputIdx : int {
  U_ROLL = 0,
  U_PITCH = 1,
  YAW_RATE = 2,
  U_VEL_Z = 3,
};

/// @brief Parameters of the Anafi quadrotor dynamics model.
/// Drag, tilt servo and vertical-velocity filter follow
/// legacy_controller/other/quadmodelODE.m; the first-order command lag
/// (TAU_CMD, states uf_roll/uf_pitch) is the damped-spring-lag extension.
struct ModelParams {
  double g = 9.81;         ///< gravity, m/s^2
  double C = 0.26;          ///< horizontal drag coefficient, 1/s
  double OMEGA_0 = 18.89;    ///< tilt-servo natural frequency, rad/s
  double ETA = 0.921;       ///< tilt-servo damping coefficient (= 2*zeta), dimensionless
  double TAUZ_INV = 2.0;   ///< inverse time constant of vertical velocity filter, 1/s
  double TAU_CMD = 0.0800;    ///< time constant of the roll/pitch command lag, s
  double INPUT_DELAY_SEC = 0.10;  ///< transport delay from command sent to drone reacting, s
};

}  // namespace ekf_estimator
