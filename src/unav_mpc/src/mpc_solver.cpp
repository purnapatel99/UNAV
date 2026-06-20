#include "unav_mpc/mpc_solver.hpp"
#include <algorithm>
#include <cassert>
#include <stdexcept>

namespace unav_mpc
{

MPCSolver::MPCSolver(const Params & p)
: p_(p), nU_(nu_ * p.N)
{
  if (p.N < 1)  { throw std::invalid_argument("MPC horizon N must be >= 1"); }
  if (p.dt <= 0){ throw std::invalid_argument("dt must be positive"); }
  precompute();
}

void MPCSolver::precompute()
{
  const double dt = p_.dt;
  const int    N  = p_.N;

  // ── System matrices (exact ZOH for double integrator) ──────────────
  //
  //   Continuous:  ẋ = [0 I; 0 0] x + [0; I] u
  //   Discrete:    x[k+1] = A x[k] + B u[k]
  //
  A_ = Eigen::MatrixXd::Identity(nx_, nx_);
  A_.block(0, 3, 3, 3) = dt * Eigen::Matrix3d::Identity();   // pos += vel*dt

  B_ = Eigen::MatrixXd::Zero(nx_, nu_);
  B_.block(0, 0, 3, 3) = 0.5 * dt * dt * Eigen::Matrix3d::Identity(); // pos += ½ a dt²
  B_.block(3, 0, 3, 3) = dt        * Eigen::Matrix3d::Identity();      // vel += a dt

  // ── Φ (6N × 6):  block row k = A^(k+1) ────────────────────────────
  Phi_.resize(nx_ * N, nx_);
  Eigen::MatrixXd Apow = A_;
  for (int k = 0; k < N; ++k) {
    Phi_.block(k * nx_, 0, nx_, nx_) = Apow;
    Apow = A_ * Apow;
  }

  // ── Γ (6N × 3N):  lower block-triangular ───────────────────────────
  //   Γ[row,col] = A^(row-col) B   for row >= col,  else 0
  Eigen::MatrixXd Gamma(nx_ * N, nU_);
  Gamma.setZero();
  for (int col = 0; col < N; ++col) {
    Eigen::MatrixXd AB = B_;
    for (int row = col; row < N; ++row) {
      Gamma.block(row * nx_, col * nu_, nx_, nu_) = AB;
      AB = A_ * AB;
    }
  }

  // ── Q̄ block-diagonal (running cost Q, terminal cost Qf) ────────────
  Eigen::MatrixXd Qbar(nx_ * N, nx_ * N);
  Qbar.setZero();
  for (int k = 0; k < N; ++k) {
    const double wp = (k == N - 1) ? p_.qf_pos : p_.q_pos;
    const double wv = (k == N - 1) ? p_.qf_vel : p_.q_vel;
    Qbar.block(k * nx_,     k * nx_,     3, 3) = wp * Eigen::Matrix3d::Identity();
    Qbar.block(k * nx_ + 3, k * nx_ + 3, 3, 3) = wv * Eigen::Matrix3d::Identity();
  }

  // ── R̄ block-diagonal input cost ─────────────────────────────────────
  Eigen::MatrixXd Rbar = p_.r_acc * Eigen::MatrixXd::Identity(nU_, nU_);

  // ── D (3N × 3N): first-difference operator for jerk ─────────────────
  //   (D U)[0]   = u[0]         (compared against u_prev at runtime)
  //   (D U)[k>0] = u[k] - u[k-1]
  Eigen::MatrixXd D(nU_, nU_);
  D.setZero();
  D.block(0, 0, nu_, nu_) = Eigen::Matrix3d::Identity();
  for (int k = 1; k < N; ++k) {
    D.block(k * nu_, k * nu_,       nu_, nu_) =  Eigen::Matrix3d::Identity();
    D.block(k * nu_, (k - 1) * nu_, nu_, nu_) = -Eigen::Matrix3d::Identity();
  }
  Eigen::MatrixXd Rdbar = p_.rd_jerk * Eigen::MatrixXd::Identity(nU_, nU_);

  // ── H = Γᵀ Q̄ Γ + R̄ + Dᵀ R̄_d D ─────────────────────────────────────
  GtQbar_ = Gamma.transpose() * Qbar;          // 3N × 6N  — precomputed for runtime
  H_      = GtQbar_ * Gamma + Rbar + D.transpose() * Rdbar * D;
  H_      = 0.5 * (H_ + H_.transpose());       // symmetrise for numerical stability

  H_chol_ = H_.llt();
  if (H_chol_.info() != Eigen::Success) {
    throw std::runtime_error(
      "MPC Hessian is not positive definite — check cost weights");
  }

  // Projected gradient step size: α = 1 / λ_max(H)
  // Safe upper bound via infinity norm (guaranteed ≥ spectral radius)
  step_size_ = 1.0 / H_.rowwise().lpNorm<1>().maxCoeff();
}

// ────────────────────────────────────────────────────────────────────────────

std::array<double, 3> MPCSolver::solve(
  const Eigen::Matrix<double, 6, 1> & x0,
  const Eigen::Matrix<double, 6, Eigen::Dynamic> & refs,
  const Eigen::Vector3d & u_prev)
{
  const int N = p_.N;
  assert(refs.cols() >= N + 1 && "refs must have N+1 columns");

  // ── Build X_ref (6N × 1): columns 1..N of refs ─────────────────────
  Eigen::VectorXd Xref(nx_ * N);
  for (int k = 0; k < N; ++k) {
    Xref.segment(k * nx_, nx_) = refs.col(k + 1);
  }

  // ── Gradient ─────────────────────────────────────────────────────────
  //   g = Γᵀ Q̄ (Φ x₀ − X_ref) − rd_jerk * [u_prev; 0; 0; ...]
  //
  //   The second term comes from expanding the jerk cost:
  //     Jerk cost = (D U + d_prev)ᵀ R̄_d (D U + d_prev)
  //   where d_prev = [−u_prev; 0; ...].
  //   ∂/∂U [jerk cost] |_{const} = Dᵀ R̄_d d_prev = rd_jerk * [−u_prev; 0; ...]
  //   so the gradient contribution is rd_jerk * [−u_prev; 0; ...].
  Eigen::VectorXd g = GtQbar_ * (Phi_ * x0 - Xref);
  g.head(nu_) -= p_.rd_jerk * u_prev;

  // ── Unconstrained optimum: U* = −H⁻¹ g ─────────────────────────────
  Eigen::VectorXd U = H_chol_.solve(-g);

  // ── Project onto acceleration box constraints ─────────────────────────
  //   Uses projected gradient:  U ← clip(U − α ∇f(U), lb, ub)
  //   α = 1/λ_max ensures descent.  Warm-starts from the unconstrained
  //   solution (already optimal if feasible; otherwise 1–5 steps typical).
  const double lb = -p_.max_acc;
  const double ub =  p_.max_acc;

  if (U.minCoeff() < lb || U.maxCoeff() > ub) {
    U = U.cwiseMax(lb).cwiseMin(ub);   // initial feasible point
    for (int i = 0; i < p_.max_iter; ++i) {
      Eigen::VectorXd U_new =
        (U - step_size_ * (H_ * U + g)).cwiseMax(lb).cwiseMin(ub);
      const double change = (U_new - U).squaredNorm();
      U = U_new;
      if (change < 1e-10) { break; }
    }
  }

  return {U(0), U(1), U(2)};
}

}  // namespace unav_mpc
