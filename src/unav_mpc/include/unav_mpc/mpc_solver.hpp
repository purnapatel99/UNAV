#pragma once

#include <Eigen/Dense>
#include <array>

namespace unav_mpc
{

/**
 * Condensed linear MPC for a quadrotor in position-control mode.
 *
 * Plant (from our command layer looking in):
 *   PX4 handles attitude/thrust — we see a double integrator in NED:
 *     x = [n, e, d, vn, ve, vd]     (6 states)
 *     u = [an, ae, ad]               (3 inputs — acceleration commands)
 *     x[k+1] = A x[k] + B u[k]
 *
 * Cost over horizon N:
 *   J = Σ_{k=1}^{N} (x[k]-x_ref[k])' Q (x[k]-x_ref[k])
 *     + Σ_{k=0}^{N-1} u[k]' R u[k]
 *     + Σ_{k=0}^{N-1} Δu[k]' Rd Δu[k]   ← jerk penalty
 *
 *   Terminal step uses Qf (larger weights for Lyapunov stability).
 *   Δu[0] = u[0] − u_prev  (continuity across control steps)
 *
 * Hard constraints:
 *   −max_acc ≤ u[k] ≤ max_acc   (per axis, enforced via projected gradient)
 *
 * Soft constraints:
 *   Velocity limit encoded through high q_vel weight rather than explicit
 *   inequalities — keeps the solver dependency-free (Eigen only).
 *
 * Precomputed at construction (constant matrices):
 *   Φ, Γ (condensed prediction),  Γᵀ Q̄  (runtime gradient multiply),  H (Hessian)
 *
 * Runtime per step: one Cholesky back-solve + O(max_iter) projected gradient steps.
 */
class MPCSolver
{
public:
  struct Params {
    int    N        = 30;   // prediction horizon steps
    double dt       = 0.02; // s — must equal 1/control_hz
    // Tracking cost (per axis)
    double q_pos    = 10.0; // position error weight
    double q_vel    = 2.0;  // velocity error weight
    double qf_pos   = 20.0; // terminal position (stabilising)
    double qf_vel   = 4.0;  // terminal velocity
    // Input cost
    double r_acc    = 0.1;  // acceleration magnitude
    double rd_jerk  = 0.5;  // Δacceleration (jerk)
    // Acceleration hard constraint (↔ MPC_ACC_HOR_MAX on 22001_gz_x500_f1follow)
    double max_acc  = 60.0;  // m/s² per axis — 6g
    // Jerk limit (↔ MPC_JERK_MAX) — max |Δu| per step = max_jerk × dt
    double max_jerk = 100.0; // m/s³
    // Solver
    int    max_iter = 50;   // projected gradient iterations (only used when constraint active)
  };

  explicit MPCSolver(const Params & p);

  int horizon() const { return p_.N; }
  double dt()   const { return p_.dt; }

  /**
   * Solve one MPC step.
   *
   * state  : [n, e, d, vn, ve, vd]
   * refs   : 6 × (N+1) matrix — column k is the reference state at step k
   *          (column 0 = current reference, columns 1..N = future horizon)
   * u_prev : acceleration commanded on the previous step (for jerk term)
   *
   * Returns [an, ae, ad] — first element of the optimal control sequence.
   */
  std::array<double, 3> solve(
    const Eigen::Matrix<double, 6, 1> & state,
    const Eigen::Matrix<double, 6, Eigen::Dynamic> & refs,
    const Eigen::Vector3d & u_prev);

private:
  static constexpr int nx_ = 6;
  static constexpr int nu_ = 3;
  Params p_;
  int    nU_;   // = nu_ * N

  // Precomputed (all constant after construction)
  Eigen::MatrixXd A_, B_;      // system matrices
  Eigen::MatrixXd Phi_;        // 6N × 6    — free-response prediction
  Eigen::MatrixXd GtQbar_;     // 3N × 6N   — Γᵀ Q̄ (gradient multiply, cached)
  Eigen::MatrixXd H_;          // 3N × 3N   — QP Hessian
  Eigen::LLT<Eigen::MatrixXd> H_chol_;  // factorised once, reused every step
  double step_size_;           // projected gradient α = 1 / λ_max_bound(H)

  void precompute();
};

}  // namespace unav_mpc
