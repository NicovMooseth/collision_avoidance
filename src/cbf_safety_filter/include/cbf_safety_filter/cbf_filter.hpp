// Control barrier function (CBF) safety filter — plain C++, no ROS.
//
// Takes the nominal (reference) control command, the current drone state and the obstacles, and returns the
// command closest to the reference that keeps the drone outside of all obstacles.
//
// Obstacle = convex polytope, the intersection of the obstacle sides of its half-planes. The drone is outside it
// as soon as it is on the free side of ONE of its planes, so an obstacle is one constraint (not one per plane).
// Sources (converted in cbf_node): the offline map, the obstacles of the clustering node, and single half-planes
// of scan_to_constraint_node (an obstacle with one plane).
//
// Pipeline (implemented in src/cbf_filter.cpp):
//   1. predict the state at the moment the new command takes effect (input delay)
//   2. one linear CBF row  A_i * a >= b_i  per obstacle, in the acceleration a
//   3. QP: acceleration closest to the reference acceleration that satisfies all rows (with slack)
//   4. convert the safe acceleration back to a command (pitch, roll, vertical velocity)
//
// Only safety_filter() is public; all steps above are file-local helpers in the .cpp.
#pragma once

#include <Eigen/Core>

#include <deque>
#include <vector>

#include "cbf_safety_filter/ekf/drone_model.hpp"  // DroneModel, ModelParams, StateVec, InputVec
#include "pcd_constraints/constraints.hpp"

using ekf_estimator::StateVec;
using ekf_estimator::InputVec;
using ekf_estimator::DroneModel;
using ekf_estimator::ModelParams;

namespace cbf_safety_filter {

    using pcd_constraints::HalfPlane;

    /// @brief Tuning of the barrier function h(x) and the limits of the safe command.
    struct BarrierFunctionParams {
        double r_safety;    ///< [m]     extra distance kept to every half-plane
        double t_reaction;  ///< [s]     reaction time: distance travelled before braking starts is t_reaction * v
        double alpha;       ///< [1/s]   CBF gain in h_dot >= -alpha * h (larger = approaches the boundary faster)
        double max_tilt;    ///< [rad]   limit of the total tilt (roll and pitch combined: tan² roll + tan² pitch <= tan² max_tilt)
        double max_v_z;     ///< [m/s]   limit for the vertical velocity command
        double dt;          ///< [s]     command period: each sent input is applied for dt in the state prediction
        double rho;         ///< [-]     QP weight of the slack s (larger = constraints are relaxed less)
    };

    /// @brief Convex obstacle: the intersection of the obstacle sides of its half-planes (world / map frame).
    ///        HalfPlane convention: normals point out of the obstacle towards free space, free side
    ///        normal.dot(p) + offset >= 0. p is inside iff normal.dot(p) + offset < 0 for all planes.
    struct Obstacle {
        int id = -1;                                         ///< ID in its source (map CSV, clustering), for debugging
        std::vector<HalfPlane> planes;                       ///< at least one; one plane = a plain half-plane constraint
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  ///< [m/s] world frame; 0 for static obstacles (map)
    };

    struct CbfRows {
        Eigen::MatrixXd A;
        Eigen::VectorXd b;
    };

    /// @brief CBF safety filter: modifies the reference command as little as possible so that the drone
    ///        stays outside of all obstacles.
    /// @param x            current (estimated) drone state
    /// @param obstacles    convex obstacles (map, clustering, single half-planes), world frame
    /// @param u_sent       commands already sent but not yet acting on the drone (input delay), oldest first
    /// @param u_ref        nominal command from the controller (roll, pitch, vertical velocity, ...)
    /// @param[out] u       safe command; only written if the function returns true
    /// @param model        drone dynamics model, used to predict the state over the input delay
    /// @param bf_params    barrier function tuning and command limits
    /// @param model_params drone model parameters (ekf_estimator::ModelParams)
    /// @param[out] slack   slack of the QP: 0 = all constraints satisfied, > 0 = constraints had to be relaxed
    /// @param[out] num_rows number of CBF rows in the QP (one per obstacle at most)
    /// @return true if the QP was solved and u is valid, false otherwise (caller decides on a fallback)
    bool safety_filter(const StateVec& x, const std::vector<Obstacle>& obstacles, const std::deque<InputVec>& u_sent,
                       const InputVec& u_ref, InputVec& u, const DroneModel& model, const BarrierFunctionParams& bf_params,
                       double& slack, int& num_rows);

} // namespace cbf_safety_filter
