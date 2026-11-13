// Control barrier function (CBF) safety filter — plain C++, no ROS.
//
// Takes the nominal (reference) control command, the current drone state and the half-plane
// constraints, and returns the command closest to the reference that keeps the drone on the safe
// side of all half-planes.
//
// Pipeline (implemented in src/cbf/cbf_filter.cpp):
//   1. predict the state at the moment the new command takes effect (input delay)
//   2. one linear CBF row  A_i * a >= b_i  per half-plane, in the acceleration a
//   3. QP: acceleration closest to the reference acceleration that satisfies all rows (with slack)
//   4. convert the safe acceleration back to a command (pitch, roll, vertical velocity)
//
// Only safety_filter() is public; all steps above are file-local helpers in the .cpp.
#pragma once

#include <Eigen/Core>

#include <deque>
#include <vector>

#include "ekf_estimator/drone_model.hpp"  // DroneModel, ModelParams, StateVec, InputVec
#include "collision_avoidance/constraints.hpp"

using ekf_estimator::StateVec;
using ekf_estimator::InputVec;
using ekf_estimator::DroneModel;
using ekf_estimator::ModelParams;

namespace collision_avoidance {

    /// @brief Tuning of the barrier function h(x) and the limits of the safe command.
    struct BarrierFunctionParams {
        double r_safety;    ///< [m]     extra distance kept to every half-plane
        double t_reaction;  ///< [s]     reaction time: distance travelled before braking starts is t_reaction * v
        double alpha;       ///< [1/s]   CBF gain in h_dot >= -alpha * h (larger = approaches the boundary faster)
        double max_tilt;    ///< [rad]   limit for roll and pitch commands
        double max_v_z;     ///< [m/s]   limit for the vertical velocity command
        double dt;          ///< [s]     command period: each sent input is applied for dt in the state prediction
        double rho;         ///< [-]     QP weight of the slack s (larger = constraints are relaxed less)
    };

    struct CbfRows {
        Eigen::MatrixXd A;
        Eigen::VectorXd b;
    };

    /// @brief CBF safety filter: modifies the reference command as little as possible so that the drone
    ///        stays on the safe side of all half-plane constraints.
    /// @param x            current (estimated) drone state
    /// @param constraints  half-plane constraints (obstacles) from scan_to_constraint_node
    /// @param u_sent       commands already sent but not yet acting on the drone (input delay), oldest first
    /// @param u_ref        nominal command from the controller (roll, pitch, vertical velocity, ...)
    /// @param[out] u       safe command; only written if the function returns true
    /// @param model        drone dynamics model, used to predict the state over the input delay
    /// @param bf_params    barrier function tuning and command limits
    /// @param model_params drone model parameters (ekf_estimator::ModelParams)
    /// @param[out] slack   slack of the QP: 0 = all constraints satisfied, > 0 = constraints had to be relaxed
    /// @return true if the QP was solved and u is valid, false otherwise (caller decides on a fallback)
    bool safety_filter(const StateVec& x, const std::vector<HalfPlane>& constraints, const std::deque<InputVec>& u_sent,
                       const InputVec& u_ref, InputVec& u, const DroneModel& model, const BarrierFunctionParams& bf_params,
                       double& slack);

} // namespace collision_avoidance
