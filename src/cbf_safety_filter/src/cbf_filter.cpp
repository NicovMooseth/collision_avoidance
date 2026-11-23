// Implementation of the CBF safety filter: the barrier function h_i(x) of each plane, combined into one barrier
// function per obstacle, the CBF condition dh/dt >= -alpha * h, and the (QP) solve for the safe command.
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

    /// Number of sides of the polygon that approximates the tilt limit |a_h| <= g tan(max_tilt) in the QP.
    constexpr int N_TILT = 64;

    /// @brief Radius of the tilt disc: the horizontal acceleration at the total tilt limit.
    ///        Holding altitude, the vertical thrust carries the weight (T cos θ = m g) and the horizontal
    ///        part accelerates the drone (T sin θ = m a), so a = g * tan(θ). With the model's
    ///        a_fwd = g tan(pitch), a_lat = -g tan(roll), the total tilt θ is tan²θ = tan²(pitch) + tan²(roll),
    ///        so θ <= max_tilt  <=>  |a_h| = sqrt(a_fwd² + a_lat²) <= g tan(max_tilt).
    /// @param bf_params contains the tilt limit max_tilt [rad]
    /// @param model_params contains g
    /// @return [m/s^2]
    double tilt_disc_radius(const BarrierFunctionParams& bf_params, const ModelParams& model_params) {
        return model_params.g * std::tan(bf_params.max_tilt);
    }

    /// @brief Maximum horizontal acceleration reachable in every direction: the inradius of the N_TILT-gon
    ///        inscribed in the tilt disc, a_max = g tan(max_tilt) cos(π / N_TILT) (2 % below the disc for 16 sides).
    ///        Used as the braking deceleration in h and as the limit of the tilt rows in the QP.
    /// @param bf_params contains the tilt limit max_tilt [rad]
    /// @param model_params contains g
    /// @return a_max [m/s^2]
    double max_acceleration(const BarrierFunctionParams& bf_params, const ModelParams& model_params) {
        return tilt_disc_radius(bf_params, model_params) * std::cos(M_PI / N_TILT);
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

    /// @brief One CBF row of a single half-plane, with its h.
    struct PlaneRow {
        Eigen::RowVector3d A;  ///< row in the world-frame acceleration a_W
        double b;
        double h;              ///< [m] barrier function of the plane, compute_h()
    };

    /// @brief Linear row of the CBF condition h_dot >= -alpha * h for one half-plane, in the acceleration a.
    ///
    ///        n faces the free side. With v_n = -n·v (approach speed) and the total acceleration a - drag:
    ///          h_dot = -v_n + gain * n·(a - drag),   gain = t_reaction + v_n / a_max
    ///        so  h_dot >= -alpha * h  becomes
    ///          gain * n^T a  >=  v_n + gain * n·drag - alpha * h
    ///          \_ A_i __/       \__________ b_i ______________/
    ///
    ///        Moving away (v_n <= 0): v_bar = 0 in h, so the gain is t_reaction (pushes the drone back out if it is
    ///        inside the margin). Rows are not skipped when moving away and h >= 0 (skipping is disabled).
    /// @param x predicted state of the drone (x_hat)
    /// @param plane half-plane, world frame
    /// @param model_params drone model parameters (drag coefficient)
    /// @param bf_params barrier function parameters (alpha, t_reaction, r_safety)
    /// @param a_max [m/s^2] maximum deceleration, from max_acceleration()
    PlaneRow planeRow(const StateVec& x, const HalfPlane& plane, const ModelParams& model_params,
                      const BarrierFunctionParams& bf_params, double a_max) {
        const Eigen::Vector3d& n = plane.normal;               // faces the free side
        const Eigen::Vector3d v = x.segment<3>(VX);
        const double v_n = -n.dot(v);                          // approach speed towards the plane
        const double h = compute_h(x, plane, bf_params, a_max);
        // approaching: gain from the derivative of the reaction and braking distance terms
        const double gain = v_n <= 0.0 ? bf_params.t_reaction : bf_params.t_reaction + v_n / a_max;
        return {gain * n.transpose(), v_n + gain * n.dot(drag_term(v, model_params)) - bf_params.alpha * h, h};
    }

    /// @brief One linear row per obstacle of the CBF condition h_obs_dot >= -alpha * h_obs, in the acceleration a.
    ///
    ///        The drone is outside a convex obstacle iff it is on the free side of at least one of its planes, so
    ///        the barrier function of the obstacle is h_obs = max_i h_i (one constraint per obstacle, not per plane).
    ///
    ///        TODO(obstacle reduction): for now the row of the active plane (largest h_i) is used. It switches
    ///        between planes at the edges of the obstacle. Alternative: smooth max, e.g.
    ///          h_obs = 1/k log(sum_i exp(k h_i)) - log(m)/k   (<= max_i h_i, i.e. conservative)
    ///        with A, b the softmax(k h_i)-weighted sum of the plane rows.
    ///        TODO(moving obstacles): Obstacle::velocity is not used yet (relative velocity v - v_obs in h and h_dot).
    /// @param x predicted state of the drone (x_hat)
    /// @param obstacles convex obstacles, world frame
    /// @param model_params drone model parameters (drag coefficient)
    /// @param bf_params barrier function parameters (alpha, t_reaction, r_safety)
    /// @param a_max [m/s^2] maximum deceleration, from max_acceleration()
    /// @return rows A (k x 3) and b (k), with k <= obstacles.size() (obstacles without planes are skipped)
    CbfRows buildCbfRows(const StateVec& x, const std::vector<Obstacle>& obstacles,
                         const ModelParams& model_params, const BarrierFunctionParams& bf_params, double a_max) {

        // allocate for the worst case (one row per obstacle), shrink to the used rows at the end
        CbfRows rows;
        rows.A.resize(static_cast<Eigen::Index>(obstacles.size()), 3);
        rows.b.resize(static_cast<Eigen::Index>(obstacles.size()));
        Eigen::Index count = 0;  // number of rows filled so far
        for (const Obstacle& obstacle : obstacles) {
            if (obstacle.planes.empty()) {
                continue;
            }
            // active plane: the one with the largest h (h_obs = max_i h_i)
            PlaneRow active = planeRow(x, obstacle.planes.front(), model_params, bf_params, a_max);
            for (std::size_t i = 1; i < obstacle.planes.size(); ++i) {
                PlaneRow row = planeRow(x, obstacle.planes[i], model_params, bf_params, a_max);
                if (row.h > active.h) {
                    active = row;
                }
            }
            rows.A.row(count) = active.A;
            rows.b(count) = active.b;
            ++count;
        }
        // drop the unused rows of the skipped obstacles
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
    ///        variables z = [a (3), s (m)],  a = [a_fwd, a_lat, az] in the body frame, one slack s_i per CBF row
    ///        min   1/2 |a - a_ref|^2 + 1/2 |s|^2 + rho * sum_i s_i
    ///        s.t.  A_i a + s_i >= b_i                            (CBF rows, each relaxed by its own slack)
    ///              cos(φ_k) a_fwd + sin(φ_k) a_lat <= a_max      (tilt: N_TILT-gon, φ_k = 2πk / N_TILT, no slack)
    ///              az_min <= az <= az_max                         (vertical velocity limit)
    ///              s >= 0
    ///
    ///        The tilt rows limit the total tilt (roll and pitch combined), not each angle on its own: the polygon
    ///        is inscribed in the disc |a_h| <= g tan(max_tilt), so the tilt limit is never exceeded.
    ///        The slacks keep the QP feasible when CBF rows conflict with the input limits; rho weights how strongly
    ///        a violation is penalised. One slack per row: a row that cannot be met (e.g. a wall inside the margin,
    ///        needing more horizontal acceleration than the tilt allows) only relaxes itself, the other rows (e.g.
    ///        the ceiling, met with az) stay enforced. With a single shared slack, that row would relax all of them.
    /// @param a_ref reference acceleration (from the nominal command), body frame
    /// @param az_min lower bound on az
    /// @param az_max upper bound on az
    /// @param a_max inradius of the tilt polygon, from max_acceleration()
    /// @param rows CBF rows in the body-frame acceleration, from buildCbfRows() and rotateRowsToBody()
    /// @param rho weight of the slacks in the cost (bf_params.rho)
    /// @param[out] a safe acceleration, body frame
    /// @param[out] slack largest slack max_i s_i of the solution (0 = no constraint relaxed, also when m = 0)
    /// @return true if qpmad found a solution
    bool solve_qp(const Eigen::Vector3d& a_ref, double az_min, double az_max, double a_max, const CbfRows& rows, double rho,
            Eigen::Vector3d& a, double& slack) {

        const double inf = std::numeric_limits<double>::infinity();
        const Eigen::Index m = rows.b.size();  // number of CBF rows = number of slacks
        const Eigen::Index n = 3 + m;          // number of variables

        // the variables z = [a, s], cost written as 1/2 z'Hz + h'z.
        // H = I gives 1/2|a|^2 + 1/2|s|^2; h = [-a_ref, rho, ..., rho] adds -a_ref·a (closest to a_ref) and
        // rho * sum_i s_i
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(n, n);
        Eigen::VectorXd h(n);
        h.head<3>() = -a_ref;
        h.tail(m).setConstant(rho);

        // bounds on z: a_fwd, a_lat free (limited by the tilt rows), az by the vertical velocity limit, s >= 0
        Eigen::VectorXd lower(n), upper(n);
        lower.head<3>() << -inf, -inf, az_min;
        upper.head<3>() << inf, inf, az_max;
        lower.tail(m).setZero();
        upper.tail(m).setConstant(inf);

        // general constraints  A_lb <= A z <= A_ub
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m + N_TILT, n);
        Eigen::VectorXd A_lb(m + N_TILT), A_ub(m + N_TILT);
        // CBF rows [A_rows, I_m] z >= b: the identity block adds slack s_i to row i only. The upper bound is
        // needed: with a lower bound alone, qpmad treats the rows as equalities
        A.topLeftCorner(m, 3) = rows.A;
        A.topRightCorner(m, m).setIdentity();
        A_lb.head(m) = rows.b;
        A_ub.head(m).setConstant(inf);
        // tilt rows: cos(φ_k) a_fwd + sin(φ_k) a_lat <= a_max, without slack (hard limit)
        for (int k = 0; k < N_TILT; ++k) {
            const double phi = 2.0 * M_PI * k / N_TILT;
            A(m + k, 0) = std::cos(phi);
            A(m + k, 1) = std::sin(phi);
            A_lb(m + k) = -inf;
            A_ub(m + k) = a_max;
        }

        Eigen::VectorXd z(n);
        qpmad::Solver solver;
        try {
            if (solver.solve(z, H, h, lower, upper, A, A_lb, A_ub) != qpmad::Solver::OK) {
                return false;
            }
        } catch (const std::exception&) {  // for example invalid input: the slack keeps the rows feasible
            return false;
        }
        // split the solution back into acceleration and slacks
        a = z.head<3>();
        slack = m > 0 ? z.tail(m).maxCoeff() : 0.0;
        return true;
    }

    /// @brief Convert a (safe) body-frame acceleration into a command; inverse of to_acceleration().
    ///          pitch = atan(a_fwd / g),  roll = -atan(a_lat / g)   (as in DroneModel::f)
    ///          v_z   = vz_hat + az / tauz_inv   (first-order vertical velocity response, TAUZ_INV)
    ///        The QP keeps |a_h| inside the tilt disc already; as a numerical safety net a longer a_h is scaled
    ///        back onto the disc (keeps its direction, unlike clamping roll and pitch on their own).
    ///        All outputs are clamped to the command limits.
    /// @param x_hat predicted state (for the current vertical velocity)
    /// @param a acceleration from the QP, body frame [a_fwd, a_lat, az]
    /// @param bf_params command limits (max_tilt, max_v_z)
    /// @param model_params drone model parameters (g, TAUZ_INV)
    /// @return command with pitch, roll and vertical velocity set, all other entries 0
    InputVec to_command(const StateVec& x_hat, const Eigen::Vector3d& a,
                        const BarrierFunctionParams& bf_params, const ModelParams& model_params) {
        const double g = model_params.g;

        Eigen::Vector2d a_h = a.head<2>();  // [a_fwd, a_lat]
        const double radius = tilt_disc_radius(bf_params, model_params);
        if (a_h.norm() > radius) {
            a_h *= radius / a_h.norm();
        }

        InputVec u = InputVec::Zero();
        u(ekf_estimator::U_PITCH) = std::clamp(std::atan(a_h.x() / g), -bf_params.max_tilt, bf_params.max_tilt);
        u(ekf_estimator::U_ROLL) = std::clamp(-std::atan(a_h.y() / g), -bf_params.max_tilt, bf_params.max_tilt);
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
    bool safety_filter(const StateVec& x, const std::vector<Obstacle>& obstacles, const std::deque<InputVec>& u_sent,
                       const InputVec& u_ref, InputVec& u, const DroneModel& model, const BarrierFunctionParams& bf_params,
                       double& slack, int& num_rows) {

        // 1. state at which the new command acts, 2. one CBF row per obstacle
        const StateVec& x_hat = estimate_state(x, model, u_sent, bf_params.dt);
        const double a_max = max_acceleration(bf_params, model.params());  // reachable in every direction, from max_tilt
        CbfRows rows = buildCbfRows(x_hat, obstacles, model.params(), bf_params, a_max);  // in a_W
        rotateRowsToBody(rows, x_hat(YAW));  // the QP solves for a_B, the frame of the tilt commands

        num_rows = static_cast<int>(rows.b.size());

        // input limits (body frame): the total tilt (polygon with inradius a_max, in solve_qp), and the v_z limit
        // as an acceleration (az = (v_z_cmd - vz) * TAUZ_INV, with v_z_cmd in [-max_v_z, max_v_z])
        const double tau_inv = model.params().TAUZ_INV;
        const double az_min = (-bf_params.max_v_z - x_hat(VZ)) * tau_inv;
        const double az_max = (bf_params.max_v_z - x_hat(VZ)) * tau_inv;

        // 3. closest safe acceleration to the reference
        Eigen::Vector3d a;
        if (!solve_qp(to_acceleration(x_hat, u_ref, model.params()), az_min, az_max, a_max, rows, bf_params.rho, a,
                      slack)) {
            return false;  // u is left unchanged
        }
        // 4. back to a command
        u = to_command(x_hat, a, bf_params, model.params());
        return true;
    }

} // namespace cbf_safety_filter
