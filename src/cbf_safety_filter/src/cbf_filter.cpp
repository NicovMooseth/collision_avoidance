// Implementation of the CBF safety filter: the barrier functions h_i(x) for each half-plane,
// the CBF condition dh_i/dt >= -alpha * h_i, and the (QP) solve for the safe command.
// File-local helpers (dynamics model, QP setup) go in the anonymous namespace.
//
// The decision variable is the acceleration a = [a_fwd, a_lat, az] of the drone in the yaw-aligned body frame
// (x forward, y left, z up), because the CBF condition is linear in it and the tilt commands act in that frame.
// The CBF rows are derived in the world frame (normals, velocity) and rotated into the body frame by the yaw.
// The reference command is converted to a reference acceleration, the QP finds the closest safe acceleration,
// and that is converted back to a command.

#include "cbf_safety_filter/cbf_filter.hpp"

#include <qpmad/solver.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <exception>

using namespace ekf_estimator;

namespace cbf_safety_filter {

namespace {

    /// @brief Predict the state at which a newly computed command starts to act.
    ///        Commands take input_delay_sec to reach the drone, so the ones already sent (u_sent) will be
    ///        applied first. Integrating the model over them gives the state the new command acts on.
    /// @param x current state of the drone (copied, then integrated forward)
    /// @param model contains the system dynamics and describes the drone input/output behaviour
    /// @param u_sent inputs that were sent but are not applied to the drone yet, oldest first
    /// @param dt [s] time each pending input is applied (command period, bf_params.dt)
    /// @return the predicted state x_hat
    StateVec estimate_state(StateVec x, const DroneModel& model, const std::deque<InputVec>& u_sent, double dt) {
        // one RK4 step per pending input
        for (const InputVec& u : u_sent) {
            x = model.rk4Step(x, u, dt);
        }
        return x;
    }

    /// @brief Maximum horizontal acceleration the drone can produce, derived from the tilt limit.
    ///        Holding altitude, the vertical thrust carries the weight (T cos θ = m g) and the horizontal
    ///        part accelerates the drone (T sin θ = m a), so a = g * tan(θ).
    ///        Roll and pitch are limited separately, so this is the value reachable in every horizontal direction.
    /// @param bf_params contains the tilt limit max_tilt [rad]
    /// @param model_params contains g
    /// @return a_max [m/s^2]
    double max_acceleration(const BarrierFunctionParams& bf_params, const ModelParams& model_params) {
        return model_params.g * std::tan(bf_params.max_tilt);
    }

    /// @brief Barrier function h(x) for one half-plane: h >= 0 means safe, h < 0 means too close.
    ///
    ///        Convention of HalfPlane (constraints.hpp): n faces the free side (away from the obstacle),
    ///        free side n·p + offset >= 0.
    ///
    ///        h = n·p + offset - r_safety - t_reaction * v_bar - v_bar^2 / (2 * a_max)
    ///
    ///        n·p + offset           distance of the drone to the plane (> 0 on the free side)
    ///        - r_safety             safety margin
    ///        - t_reaction * v_bar   distance travelled during the reaction time
    ///        - v_bar^2 / (2 a_max)  braking distance at maximum deceleration
    ///
    ///        v_bar = max(0, -n·v) is the speed towards the plane; moving away does not reduce h.
    /// @param x current state of the drone
    /// @param constraint half-plane representing an obstacle
    /// @param bf_params barrier function parameters (r_safety, t_reaction)
    /// @param a_max [m/s^2] maximum deceleration, from max_acceleration()
    /// @return h(x) in [m]
    double compute_h(const StateVec& x, const HalfPlane& constraint, const BarrierFunctionParams& bf_params, double a_max) {
        const Eigen::Vector3d& n = constraint.normal;                    // faces the free side
        const double v_bar = std::max(0.0, -n.dot(x.segment<3>(VX)));   // approach speed, 0 when moving away

        return n.dot(x.segment<3>(PX)) + constraint.offset - bf_params.r_safety - bf_params.t_reaction * v_bar -
           v_bar * v_bar / (2 * a_max);
    }

    /// @brief Linear drag of the drone: c*vx in x, c*vy in y and 0 in z (z is handled by the
    ///        vertical velocity loop).
    /// @param v velocity vector of the current state
    /// @param model_params contains the drag coefficient C
    /// @return drag acceleration [m/s^2] that reduces the velocity in each direction
    Eigen::Vector3d drag_term(const Eigen::Vector3d& v, const ModelParams& model_params) {
        return {model_params.C * v.x(), model_params.C * v.y(), 0.0};
    }

    /// @brief One linear row per half-plane of the CBF condition h_dot >= -alpha * h, in the acceleration a.
    ///
    ///        n faces the free side. With v_n = -n·v (approach speed) and the total acceleration a - drag:
    ///          h_dot = -v_n + gain * n·(a - drag),   gain = t_reaction + v_n / a_max
    ///        so  h_dot >= -alpha * h  becomes
    ///          gain * n^T a  >=  v_n + gain * n·drag - alpha * h
    ///          \_ A_i __/       \__________ b_i ______________/
    ///
    ///        Moving away (v_n <= 0): the plane is skipped if h >= 0. If h < 0 (already inside the
    ///        margin), a row with gain = t_reaction is added so the drone is pushed back out.
    /// @param x predicted state of the drone (x_hat)
    /// @param constraints all half-plane constraints of the current scan
    /// @param model_params drone model parameters (drag coefficient)
    /// @param bf_params barrier function parameters (alpha, t_reaction, r_safety)
    /// @param a_max [m/s^2] maximum deceleration, from max_acceleration()
    /// @return rows A (k x 3) and b (k), with k <= constraints.size() active rows
    CbfRows buildCbfRows(const StateVec& x, const std::vector<HalfPlane>& constraints,
                         const ModelParams& model_params, const BarrierFunctionParams& bf_params, double a_max) {

        // allocate for the worst case (all planes active), shrink to the used rows at the end
        CbfRows rows;
        rows.A.resize(static_cast<Eigen::Index>(constraints.size()), 3);
        rows.b.resize(static_cast<Eigen::Index>(constraints.size()));
        Eigen::Index count = 0;  // number of rows filled so far
        for (const HalfPlane& constraint : constraints) {
            const Eigen::Vector3d& n = constraint.normal;  // faces the free side
            const double v_n = -n.dot(x.segment<3>(VX));   // approach speed towards the plane
            const double h = compute_h(x, constraint, bf_params, a_max);
            if (v_n <= 0.0) {   // check if moving away from obstacle
                // if (h >= 0.0) { // check if within safe distance of obstacle
                //     continue;   // safe and moving away: no row needed
                // }
                // inside the margin while moving away: v_bar = 0, so use t_reaction as the gain
                rows.A.row(count) = bf_params.t_reaction * n.transpose();
                rows.b(count) = v_n + bf_params.t_reaction * n.dot(drag_term(x.segment<3>(VX), model_params)) - bf_params.alpha * h;
                ++count;
                continue;
            }
            // approaching: gain from the derivative of the reaction and braking distance terms
            const double gain = bf_params.t_reaction + v_n / a_max;
            rows.A.row(count) = gain * n.transpose();
            rows.b(count) = v_n + gain * n.dot(drag_term(x.segment<3>(VX), model_params)) - bf_params.alpha * h;
            ++count;
        }
        // drop the unused rows of the skipped planes
        rows.A.conservativeResize(count, 3);
        rows.b.conservativeResize(count);
        return rows;
    }

    /// @brief Rotate the CBF rows from the world-frame acceleration a_W to the body-frame acceleration a_B.
    ///        The tilt acceleration is generated in the yaw-aligned body frame (DroneModel::f):
    ///          a_W = R_z(yaw) a_B   ->   A a_W = (A R_z(yaw)) a_B
    ///        z is the same in both frames, so only the x/y columns change.
    /// @param rows CBF rows in a_W from buildCbfRows(), rotated in place
    /// @param yaw [rad] heading of the predicted state
    void rotateRowsToBody(CbfRows& rows, double yaw) {
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);
        Eigen::Matrix2d R_z;  // horizontal part of R_z(yaw): body -> world
        R_z << cy, -sy,
               sy,  cy;
        rows.A.leftCols<2>() = (rows.A.leftCols<2>() * R_z).eval();
    }

    /// @brief Solve the safety QP for the acceleration closest to the reference.
    ///
    ///        variables z = [a (3), s (1)]
    ///        min   1/2 |a - a_ref|^2 + 1/2 s^2 + rho * s
    ///        s.t.  A a + s >= b       (CBF rows, all relaxed by the same slack s)
    ///              lb <= a <= ub      (tilt and vertical velocity limits)
    ///              s >= 0
    ///
    ///        The slack keeps the QP feasible when the constraints conflict with the input limits;
    ///        rho weights how strongly a violation is penalised.
    /// @param a_ref reference acceleration (from the nominal command), body frame
    /// @param lb lower bound on a (body frame)
    /// @param ub upper bound on a (body frame)
    /// @param rows CBF rows in the body-frame acceleration, from buildCbfRows() and rotateRowsToBody()
    /// @param rho weight of the slack in the cost (bf_params.rho)
    /// @param[out] a safe acceleration, body frame
    /// @param[out] slack slack s of the solution (0 = no constraint relaxed)
    /// @return true if qpmad found a solution
    bool solve_qp(const Eigen::Vector3d& a_ref, const Eigen::Vector3d& lb, const Eigen::Vector3d& ub, const CbfRows& rows, double rho,
            Eigen::Vector3d& a, double& slack) {

        const double inf = std::numeric_limits<double>::infinity();
        const Eigen::Index m = rows.b.size();  // number of CBF rows

        // the variables z = [a, s], cost written as 1/2 z'Hz + h'z.
        // H = I gives 1/2|a|^2 + 1/2 s^2; h = [-a_ref, rho] adds -a_ref·a (closest to a_ref) and rho * s
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(4, 4);
        Eigen::VectorXd h(4);
        h << -a_ref, rho;

        // bounds on z: input limits for a, s >= 0
        Eigen::VectorXd lower(4), upper(4);
        lower << lb, 0.0;
        upper << ub, inf;

        // general constraints [A_rows, 1] * z >= b: the column of ones adds the slack to every row
        Eigen::MatrixXd A(m, 4);
        A << rows.A, Eigen::VectorXd::Ones(m);
        // the upper bound is needed: with a lower bound alone, qpmad treats the rows as equalities
        const Eigen::VectorXd no_limit = Eigen::VectorXd::Constant(m, inf);

        Eigen::VectorXd z(4);
        qpmad::Solver solver;
        try {
            if (solver.solve(z, H, h, lower, upper, A, rows.b, no_limit) != qpmad::Solver::OK) {
                return false;
            }
        } catch (const std::exception&) {  // for example invalid input: the slack keeps the rows feasible
            return false;
        }
        // split the solution back into acceleration and slack
        a = z.head<3>();
        slack = z(3);
        return true;
    }

    /// @brief Convert a (safe) body-frame acceleration into a command; inverse of to_acceleration().
    ///          pitch = atan(a_fwd / g),  roll = -atan(a_lat / g)   (as in DroneModel::f)
    ///          v_z   = vz_hat + az / tauz_inv   (first-order vertical velocity response, TAUZ_INV)
    ///        All outputs are clamped to the command limits.
    /// @param x_hat predicted state (for the current vertical velocity)
    /// @param a acceleration from the QP, body frame [a_fwd, a_lat, az]
    /// @param bf_params command limits (max_tilt, max_v_z)
    /// @param model_params drone model parameters (g, TAUZ_INV)
    /// @return command with pitch, roll and vertical velocity set, all other entries 0
    InputVec to_command(const StateVec& x_hat, const Eigen::Vector3d& a,
                        const BarrierFunctionParams& bf_params, const ModelParams& model_params) {
        const double g = model_params.g;

        InputVec u = InputVec::Zero();
        u(ekf_estimator::U_PITCH) = std::clamp(std::atan(a.x() / g), -bf_params.max_tilt, bf_params.max_tilt);
        u(ekf_estimator::U_ROLL) = std::clamp(-std::atan(a.y() / g), -bf_params.max_tilt, bf_params.max_tilt);
        u(ekf_estimator::U_VEL_Z) = std::clamp(x_hat(ekf_estimator::VZ) + a.z() / model_params.TAUZ_INV,
                                            -bf_params.max_v_z, bf_params.max_v_z);
        return u;
    }

    /// @brief Convert a command into the body-frame acceleration it produces; inverse of to_command().
    ///          a_fwd = g * tan(pitch),  a_lat = -g * tan(roll)   (as in DroneModel::f)
    ///          az = (v_z_cmd - vz_hat) * tauz_inv
    /// @param x_hat predicted state (for the current vertical velocity)
    /// @param u_ref reference command
    /// @param model_params drone model parameters (g, TAUZ_INV)
    /// @return reference acceleration a_ref [m/s^2], body frame
    Eigen::Vector3d to_acceleration(const StateVec& x_hat, const InputVec& u_ref, const ModelParams& model_params) {
        const double g = model_params.g;
        return {g * std::tan(u_ref(ekf_estimator::U_PITCH)), -g * std::tan(u_ref(ekf_estimator::U_ROLL)),
                (u_ref(ekf_estimator::U_VEL_Z) - x_hat(ekf_estimator::VZ)) * model_params.TAUZ_INV};
    }

}  // namespace

    // Public entry point, documented in cbf_filter.hpp.
    bool safety_filter(const StateVec& x, const std::vector<HalfPlane>& constraints, const std::deque<InputVec>& u_sent,
                       const InputVec& u_ref, InputVec& u, const DroneModel& model, const BarrierFunctionParams& bf_params,
                       double& slack, int& num_rows) {

        // 1. state at which the new command acts, 2. one CBF row per relevant half-plane
        const StateVec& x_hat = estimate_state(x, model, u_sent, bf_params.dt);
        const double a_max = max_acceleration(bf_params, model.params());  // derived from max_tilt
        CbfRows rows = buildCbfRows(x_hat, constraints, model.params(), bf_params, a_max);  // in a_W
        rotateRowsToBody(rows, x_hat(YAW));  // the QP solves for a_B, the frame of the tilt commands

        num_rows = static_cast<int>(rows.b.size());

        // input limits (body frame): the tilt limit (= a_max), and the v_z limit as an acceleration
        // (az = (v_z_cmd - vz) * TAUZ_INV, with v_z_cmd in [-max_v_z, max_v_z])
        const double tau_inv = model.params().TAUZ_INV;
        const Eigen::Vector3d lb(-a_max, -a_max, (-bf_params.max_v_z - x_hat(VZ)) * tau_inv);
        const Eigen::Vector3d ub(a_max, a_max, (bf_params.max_v_z - x_hat(VZ)) * tau_inv);

        // 3. closest safe acceleration to the reference
        Eigen::Vector3d a;
        if (!solve_qp(to_acceleration(x_hat, u_ref, model.params()), lb, ub, rows, bf_params.rho, a, slack)) {
            return false;  // u is left unchanged
        }
        // 4. back to a command
        u = to_command(x_hat, a, bf_params, model.params());
        return true;
    }

} // namespace cbf_safety_filter
