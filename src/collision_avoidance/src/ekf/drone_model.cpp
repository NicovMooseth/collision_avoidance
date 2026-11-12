#include "ekf_estimator/drone_model.hpp"

#include <cmath>

namespace ekf_estimator {

DroneModel::DroneModel(const ModelParams& params) : p_(params) {}

StateVec DroneModel::f(const StateVec& x, const InputVec& u) const {
  const double vx = x(VX);
  const double vy = x(VY);
  const double vz = x(VZ);
  const double d_roll = x(DROLL);
  const double d_pitch = x(DPITCH);
  const double w_roll = x(WROLL);
  const double w_pitch = x(WPITCH);
  const double uf_roll = x(UF_ROLL);
  const double uf_pitch = x(UF_PITCH);

  const double yaw = x(YAW);

  const double u_roll = u(U_ROLL);
  const double u_pitch = u(U_PITCH);
  const double yaw_rate = u(YAW_RATE);
  const double u_vel_z = u(U_VEL_Z);

  // Tilt-induced horizontal acceleration is generated in the (yaw-aligned)
  // body frame; rotate it into the world frame by the heading before it drives
  // the world-frame velocity states. Drag acts on the world-frame velocity
  // directly.
  const double a_fwd = p_.g * std::tan(d_pitch);   // body +x (forward), from pitch
  const double a_lat = -p_.g * std::tan(d_roll);   // body +y (left), from roll
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  StateVec dx;
  dx(PX) = vx;
  dx(PY) = vy;
  dx(PZ) = vz;
  dx(VX) = cy * a_fwd - sy * a_lat - p_.C * vx;
  dx(VY) = sy * a_fwd + cy * a_lat - p_.C * vy;
  dx(VZ) = p_.TAUZ_INV * (u_vel_z - vz);
  dx(DROLL) = w_roll;
  dx(DPITCH) = w_pitch;
  // The tilt servo tracks the lagged command uf, not the raw command u.
  dx(WROLL) = p_.OMEGA_0 * p_.OMEGA_0 * (uf_roll - d_roll)
              - p_.ETA * p_.OMEGA_0 * w_roll;
  dx(WPITCH) = p_.OMEGA_0 * p_.OMEGA_0 * (uf_pitch - d_pitch)
               - p_.ETA * p_.OMEGA_0 * w_pitch;
  dx(YAW) = yaw_rate;
  dx(UF_ROLL) = (u_roll - uf_roll) / p_.TAU_CMD;
  dx(UF_PITCH) = (u_pitch - uf_pitch) / p_.TAU_CMD;
  return dx;
}

StateMat DroneModel::F(const StateVec& x, const InputVec& /*u*/) const {
  StateMat J = StateMat::Zero();
  const double d_roll = x(DROLL);
  const double d_pitch = x(DPITCH);
  const double yaw = x(YAW);

  // Position rows: dpos/dt = vel.
  J(PX, VX) = 1.0;
  J(PY, VY) = 1.0;
  J(PZ, VZ) = 1.0;

  // Velocity rows: drag + yaw-rotated tilt coupling. sec^2 = 1 / cos^2.
  //   dx(VX) = cy*a_fwd - sy*a_lat - C*vx
  //   dx(VY) = sy*a_fwd + cy*a_lat - C*vy
  // with a_fwd = g*tan(d_pitch), a_lat = -g*tan(d_roll).
  const double sec2_roll = 1.0 / (std::cos(d_roll) * std::cos(d_roll));
  const double sec2_pitch = 1.0 / (std::cos(d_pitch) * std::cos(d_pitch));
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double a_fwd = p_.g * std::tan(d_pitch);
  const double a_lat = -p_.g * std::tan(d_roll);

  J(VX, VX) = -p_.C;
  J(VX, DPITCH) = cy * p_.g * sec2_pitch;
  J(VX, DROLL) = sy * p_.g * sec2_roll;  // -sy * d(a_lat)/d(d_roll)
  J(VX, YAW) = -sy * a_fwd - cy * a_lat;

  J(VY, VY) = -p_.C;
  J(VY, DPITCH) = sy * p_.g * sec2_pitch;
  J(VY, DROLL) = -cy * p_.g * sec2_roll;  // cy * d(a_lat)/d(d_roll)
  J(VY, YAW) = cy * a_fwd - sy * a_lat;

  J(VZ, VZ) = -p_.TAUZ_INV;

  // Tilt-rate kinematics: ddelta/dt = omega.
  J(DROLL, WROLL) = 1.0;
  J(DPITCH, WPITCH) = 1.0;

  // Second-order tilt servo, driven by the lagged command.
  const double w0_sq = p_.OMEGA_0 * p_.OMEGA_0;
  const double damping = p_.ETA * p_.OMEGA_0;
  J(WROLL, DROLL) = -w0_sq;
  J(WROLL, WROLL) = -damping;
  J(WROLL, UF_ROLL) = w0_sq;
  J(WPITCH, DPITCH) = -w0_sq;
  J(WPITCH, WPITCH) = -damping;
  J(WPITCH, UF_PITCH) = w0_sq;

  // First-order command lag; driven by the exogenous u, so no other column.
  J(UF_ROLL, UF_ROLL) = -1.0 / p_.TAU_CMD;
  J(UF_PITCH, UF_PITCH) = -1.0 / p_.TAU_CMD;

  // Yaw is driven only by the input yaw_rate => zero df/dx row. It does,
  // however, appear as a column (it rotates the tilt acceleration above).

  return J;
}

StateVec DroneModel::rk4Step(const StateVec& x, const InputTimeFn& command,
                             double t0, double dt) const {
  const double delay = p_.INPUT_DELAY_SEC;
  const InputVec u_begin = command(t0 - delay);
  const InputVec u_mid = command(t0 + 0.5 * dt - delay);
  const InputVec u_end = command(t0 + dt - delay);

  const StateVec k1 = f(x, u_begin);
  const StateVec k2 = f(x + 0.5 * dt * k1, u_mid);
  const StateVec k3 = f(x + 0.5 * dt * k2, u_mid);
  const StateVec k4 = f(x + dt * k3, u_end);
  return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

StateVec DroneModel::rk4Step(const StateVec& x, const InputVec& u,
                             double dt) const {
  return rk4Step(x, [&u](double) { return u; }, 0.0, dt);
}

double DroneModel::wrapAngle(double a) {
  constexpr double kTwoPi = 2.0 * M_PI;
  // Map into (-pi, pi]. fmod can return a value in [-2pi, 2pi].
  double w = std::fmod(a + M_PI, kTwoPi);
  if (w <= 0.0) {
    w += kTwoPi;
  }
  return w - M_PI;
}

}  // namespace ekf_estimator
