#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <unav_msgs/msg/mpc_command.hpp>
#include <unav_msgs/msg/object_gps.hpp>

#include "unav_mpc/path_sampler.hpp"
#include "unav_mpc/mpc_solver.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;
using unav_msgs::msg::MPCCommand;
using unav_msgs::msg::ObjectGps;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace unav_mpc
{

/**
 * MPC Controller Node.
 *
 * Safety model:
 *   - Does NOT arm or switch modes autonomously.
 *   - Waits passively until PX4 reports NAVIGATION_STATE_OFFBOARD.
 *   - On entering Offboard: captures current position and loiters.
 *   - On leaving Offboard: stops sending trajectory setpoints immediately.
 *   - Any manual mode switch acts as a safety override.
 *
 * Control law:
 *   Condensed linear MPC over a double-integrator plant.
 *   Cost: position error + velocity error + acceleration magnitude + jerk.
 *   Hard constraint: |acceleration| ≤ max_acc per axis (projected gradient).
 *   Output: [an, ae, ad] sent as TrajectorySetpoint acceleration feedforward.
 */
class MPCControllerNode : public rclcpp::Node
{
public:
  MPCControllerNode()
  : Node("mpc_controller"),
    command_type_(MPCCommand::LOITER),
    s_ref_(0.0),
    u_prev_(Eigen::Vector3d::Zero()),
    have_position_(false),
    in_offboard_(false)
  {
    // ── Parameters ──────────────────────────────────────────────────
    declare_parameter("control_hz",  50.0);
    // Shared motion limits (flight_limits.yaml — keep in sync with PX4 params)
    declare_parameter("max_vel_xy",  100.0);  // MPC_XY_VEL_MAX
    declare_parameter("max_vel_z_up", 20.0);  // MPC_Z_VEL_MAX_UP
    declare_parameter("max_vel_z_dn", 20.0);  // MPC_Z_VEL_MAX_DN
    declare_parameter("max_acc",      60.0);  // MPC_ACC_HOR_MAX — 6g, enforced in MPC QP
    declare_parameter("max_jerk",    100.0);  // MPC_JERK_MAX — m/s³
    // MPC tuning
    declare_parameter("mpc_horizon",   30);
    declare_parameter("q_pos",        10.0);
    declare_parameter("q_vel",         2.0);
    declare_parameter("qf_pos",       20.0);
    declare_parameter("qf_vel",        4.0);
    declare_parameter("r_acc",         0.1);
    declare_parameter("rd_jerk",       0.5);

    const double hz = get_parameter("control_hz").as_double();
    dt_ = 1.0 / hz;

    max_vel_xy_   = get_parameter("max_vel_xy").as_double();
    max_vel_z_up_ = get_parameter("max_vel_z_up").as_double();
    max_vel_z_dn_ = get_parameter("max_vel_z_dn").as_double();

    MPCSolver::Params mp;
    mp.N        = get_parameter("mpc_horizon").as_int();
    mp.dt       = dt_;
    mp.q_pos    = get_parameter("q_pos").as_double();
    mp.q_vel    = get_parameter("q_vel").as_double();
    mp.qf_pos   = get_parameter("qf_pos").as_double();
    mp.qf_vel   = get_parameter("qf_vel").as_double();
    mp.r_acc    = get_parameter("r_acc").as_double();
    mp.rd_jerk  = get_parameter("rd_jerk").as_double();
    mp.max_acc  = get_parameter("max_acc").as_double();
    mp.max_jerk = get_parameter("max_jerk").as_double();
    max_acc_    = mp.max_acc;

    mpc_ = std::make_unique<MPCSolver>(mp);

    // ── QoS ─────────────────────────────────────────────────────────
    rclcpp::QoS px4_qos(10);
    px4_qos.best_effort().durability_volatile();

    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local().reliable();

    // ── Subscriptions ────────────────────────────────────────────────
    cmd_sub_ = create_subscription<MPCCommand>(
      "/mpc/command", latched_qos,
      [this](MPCCommand::SharedPtr msg) { on_command(msg); });

    pos_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", px4_qos,
      [this](VehicleLocalPosition::SharedPtr msg) { on_position(msg); });

    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v4", px4_qos,
      [this](VehicleStatus::SharedPtr msg) { on_vehicle_status(msg); });

    // ── Publishers ───────────────────────────────────────────────────
    traj_pub_ = create_publisher<TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", px4_qos);

    offboard_pub_ = create_publisher<OffboardControlMode>(
      "/fmu/in/offboard_control_mode", px4_qos);

    // ── Control timer ────────────────────────────────────────────────
    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      [this]() { control_loop(); });

    RCLCPP_INFO(get_logger(),
      "[MPC] Ready — horizon=%d steps (%.2f s)  dt=%.3f s  "
      "max_acc=%.1f m/s²  max_vel_xy=%.1f  z_up=%.1f  z_dn=%.1f m/s",
      mp.N, mp.N * dt_, dt_, mp.max_acc,
      max_vel_xy_, max_vel_z_up_, max_vel_z_dn_);
  }

private:
  // ── Helpers ────────────────────────────────────────────────────────

  // Project a loiter hold point forward by the kinematic braking distance so
  // the drone decelerates into it smoothly rather than trying to hold a point
  // it has already passed.
  void set_loiter_from_current_state()
  {
    const double v = std::sqrt(vel_n_*vel_n_ + vel_e_*vel_e_ + vel_d_*vel_d_);
    if (v > 0.1) {
      const double brake_dist = (v * v) / (2.0 * max_acc_);
      loiter_n_ = pos_n_ + (vel_n_ / v) * brake_dist;
      loiter_e_ = pos_e_ + (vel_e_ / v) * brake_dist;
      loiter_d_ = pos_d_ + (vel_d_ / v) * brake_dist;
    } else {
      loiter_n_ = pos_n_;
      loiter_e_ = pos_e_;
      loiter_d_ = pos_d_;
    }
  }

  // GPS (lat°, lon°, alt AMSL) → NED (m) using PX4's flat-earth local origin.
  void gps_to_ned(double lat, double lon, double alt,
                  double & n, double & e, double & d) const
  {
    constexpr double R_EARTH = 6378137.0;
    const double ref_lat_rad = ref_lat_ * M_PI / 180.0;
    n = (lat - ref_lat_) * (M_PI / 180.0) * R_EARTH;
    e = (lon - ref_lon_) * (M_PI / 180.0) * R_EARTH * std::cos(ref_lat_rad);
    d = -(alt - ref_alt_);
  }

  // Clamp velocity fields of a setpoint against the active motion limits.
  // PX4 would do this anyway, but sending already-clamped values keeps the
  // MPC's published velocity consistent with what PX4 will actually track.
  void clamp_velocity(TrajectorySetpoint & sp) const
  {
    const float vxy_max = static_cast<float>(max_vel_xy_);
    const float vz_up   = static_cast<float>(max_vel_z_up_);
    const float vz_dn   = static_cast<float>(max_vel_z_dn_);

    // Horizontal: scale down the XY vector if its magnitude exceeds the limit
    const float vxy = std::sqrt(sp.velocity[0]*sp.velocity[0] +
                                sp.velocity[1]*sp.velocity[1]);
    if (vxy > vxy_max && vxy > 1e-6f) {
      const float scale = vxy_max / vxy;
      sp.velocity[0] *= scale;
      sp.velocity[1] *= scale;
    }

    // Vertical: positive vz = down in NED
    sp.velocity[2] = std::max(-vz_up, std::min(vz_dn, sp.velocity[2]));
  }

  // ── Callbacks ──────────────────────────────────────────────────────

  void on_command(MPCCommand::SharedPtr msg)
  {
    command_type_ = msg->command_type;

    if (msg->command_type == MPCCommand::FOLLOW_TRAJECTORY) {
      path_.load(msg->trajectory);
      if (!path_.valid()) {
        RCLCPP_WARN(get_logger(), "[MPC] Empty trajectory — falling back to LOITER");
        command_type_ = MPCCommand::LOITER;
        return;
      }
      s_ref_  = 0.0;
      u_prev_ = Eigen::Vector3d::Zero();
      RCLCPP_INFO(get_logger(),
        "[MPC] LOITER → FOLLOW_TRAJECTORY  %.0f m  %zu path points",
        path_.total_length(), msg->trajectory.points.size());

    } else if (msg->command_type == MPCCommand::FOLLOW_OBJECT) {
      follow_radius_    = msg->follow_radius;
      follow_direction_ = msg->follow_direction * M_PI / 180.0;  // store as radians
      follow_alt_above_ = msg->follow_altitude_above;
      u_prev_ = Eigen::Vector3d::Zero();

      const std::string topic = "/objects/" + msg->object_id + "/gps";
      if (msg->object_id != current_object_id_) {
        current_object_id_ = msg->object_id;
        obj_have_gps_       = false;
        obj_vel_n_ = obj_vel_e_ = 0.0;
        obj_heading_ = NAN;

        rclcpp::QoS qos(10);
        qos.best_effort().durability_volatile();
        obj_gps_sub_ = create_subscription<ObjectGps>(
          topic, qos,
          [this](ObjectGps::SharedPtr m) { on_object_gps(m); });

        RCLCPP_INFO(get_logger(),
          "[MPC] → FOLLOW_OBJECT  id='%s'  topic='%s'  radius=%.1f m  "
          "direction=%.0f°  alt_above=%.1f m",
          msg->object_id.c_str(), topic.c_str(),
          follow_radius_, msg->follow_direction, follow_alt_above_);
      } else {
        RCLCPP_INFO(get_logger(),
          "[MPC] FOLLOW_OBJECT params updated  radius=%.1f m  direction=%.0f°",
          follow_radius_, msg->follow_direction);
      }

    } else if (msg->command_type == MPCCommand::FOLLOW_POINT) {
      follow_n_ = msg->follow_n;
      follow_e_ = msg->follow_e;
      follow_d_ = msg->follow_d;
      u_prev_ = Eigen::Vector3d::Zero();
      RCLCPP_INFO(get_logger(), "[MPC] → FOLLOW_POINT  N=%.1f E=%.1f D=%.1f",
        follow_n_, follow_e_, follow_d_);

    } else if (msg->command_type == MPCCommand::LOITER) {
      set_loiter_from_current_state();
      RCLCPP_INFO(get_logger(), "[MPC] → LOITER  N=%.1f E=%.1f D=%.1f",
        loiter_n_, loiter_e_, loiter_d_);

    } else {
      RCLCPP_WARN(get_logger(), "[MPC] Unknown command type %d", msg->command_type);
    }
  }

  void on_position(VehicleLocalPosition::SharedPtr msg)
  {
    // Capture NED origin for GPS→NED conversion (used by FOLLOW_OBJECT)
    if (msg->xy_global && msg->z_global) {
      ref_lat_ = msg->ref_lat;
      ref_lon_ = msg->ref_lon;
      ref_alt_ = msg->ref_alt;
      have_ned_origin_ = true;
    }

    pos_n_ = msg->x;
    pos_e_ = msg->y;
    pos_d_ = msg->z;
    vel_n_ = msg->vx;
    vel_e_ = msg->vy;
    vel_d_ = msg->vz;

    if (!have_position_) {
      RCLCPP_INFO(get_logger(),
        "[MPC] Limits active — max_vel_xy=%.1f z_up=%.1f z_dn=%.1f m/s  max_acc=%.1f m/s² "
        "(from flight_limits.yaml — set to match your PX4 MPC_XY_VEL_MAX / MPC_Z_VEL_MAX_UP/DN)",
        max_vel_xy_, max_vel_z_up_, max_vel_z_dn_, max_acc_);
    }

    have_position_ = true;
  }

  void on_object_gps(ObjectGps::SharedPtr msg)
  {
    if (!have_ned_origin_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "[MPC] No NED origin yet — object GPS ignored");
      return;
    }

    double new_n, new_e, new_d;
    gps_to_ned(msg->lat, msg->lon, msg->alt, new_n, new_e, new_d);

    const double steady_now_s = static_cast<double>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count()) * 1e-9;

    if (obj_have_gps_) {
      const double now_s = static_cast<double>(msg->header.stamp.sec)
                         + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
      const double dt_gps = now_s - obj_last_stamp_s_;

      if (dt_gps > 0.01 && dt_gps < 1.0) {
        // Raw finite-difference velocity
        const double vn_raw = (new_n - obj_pos_n_) / dt_gps;
        const double ve_raw = (new_e - obj_pos_e_) / dt_gps;

        // EMA smoothing (α=0.3 for ~10 Hz GPS)
        constexpr double kAlpha = 0.3;
        obj_vel_n_ = kAlpha * vn_raw + (1.0 - kAlpha) * obj_vel_n_;
        obj_vel_e_ = kAlpha * ve_raw + (1.0 - kAlpha) * obj_vel_e_;
      }

      obj_last_stamp_s_  = now_s;
      obj_last_time_s_   = steady_now_s;
    } else {
      obj_last_stamp_s_ = static_cast<double>(msg->header.stamp.sec)
                        + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
      obj_last_time_s_  = steady_now_s;
      obj_have_gps_     = true;
    }

    obj_pos_n_ = new_n;
    obj_pos_e_ = new_e;
    obj_pos_d_ = new_d;

    // Heading: prefer explicit field, fall back to velocity direction
    if (std::isfinite(msg->heading)) {
      obj_heading_ = static_cast<double>(msg->heading);
    } else {
      const double spd = std::sqrt(obj_vel_n_*obj_vel_n_ + obj_vel_e_*obj_vel_e_);
      obj_heading_ = (spd > 0.5) ? std::atan2(obj_vel_e_, obj_vel_n_) : obj_heading_;
    }

    // --- TEST: convert future states to NED and cache them ---
    const size_t nf = msg->future_dt_s.size();
    if (nf > 0 &&
        nf == msg->future_lat.size() &&
        nf == msg->future_lon.size() &&
        nf == msg->future_alt.size())
    {
      obj_future_dt_s_.resize(nf);
      obj_future_n_.resize(nf);
      obj_future_e_.resize(nf);
      obj_future_d_.resize(nf);
      obj_future_vn_.resize(nf);
      obj_future_ve_.resize(nf);
      for (size_t i = 0; i < nf; ++i) {
        obj_future_dt_s_[i] = msg->future_dt_s[i];
        gps_to_ned(msg->future_lat[i], msg->future_lon[i], msg->future_alt[i],
                   obj_future_n_[i], obj_future_e_[i], obj_future_d_[i]);
        const double hdg = (i < msg->future_heading.size())
                         ? static_cast<double>(msg->future_heading[i])
                         : obj_heading_;
        const double spd = (i < msg->future_speed.size())
                         ? static_cast<double>(msg->future_speed[i])
                         : 0.0;
        obj_future_vn_[i] = spd * std::cos(hdg);
        obj_future_ve_[i] = spd * std::sin(hdg);
      }
    } else {
      obj_future_dt_s_.clear();
    }
  }

  void on_vehicle_status(VehicleStatus::SharedPtr msg)
  {
    const bool now_offboard =
      (msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD);

    if (now_offboard && !in_offboard_) {
      in_offboard_ = true;
      set_loiter_from_current_state();
      u_prev_ = Eigen::Vector3d::Zero();
      RCLCPP_INFO(get_logger(),
        "[MPC] Offboard ON → LOITER hold N=%.1f E=%.1f D=%.1f",
        loiter_n_, loiter_e_, loiter_d_);

    } else if (!now_offboard && in_offboard_) {
      in_offboard_ = false;
      RCLCPP_WARN(get_logger(), "[MPC] Offboard OFF — control suspended");
    }
  }

  // ── 50 Hz control loop ─────────────────────────────────────────────

  void control_loop()
  {
    publish_offboard_control_mode();   // always stream — PX4 requires this heartbeat

    now_control_s_ = static_cast<double>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count()) * 1e-9;

    if (!in_offboard_) { return; }

    if (!have_position_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "[MPC] No position estimate");
      return;
    }

    switch (command_type_) {
      case MPCCommand::FOLLOW_TRAJECTORY:
        execute_follow_trajectory();
        break;
      case MPCCommand::FOLLOW_OBJECT:
        execute_follow_object();
        break;
      case MPCCommand::FOLLOW_POINT:
        execute_follow_point();
        break;
      case MPCCommand::LOITER:
      default:
        execute_loiter();
        break;
    }
  }

  // ── LOITER ─────────────────────────────────────────────────────────

  void execute_loiter()
  {
    TrajectorySetpoint sp{};
    sp.timestamp    = timestamp_us();
    sp.position[0]  = static_cast<float>(loiter_n_);
    sp.position[1]  = static_cast<float>(loiter_e_);
    sp.position[2]  = static_cast<float>(loiter_d_);
    sp.velocity[0]  = sp.velocity[1]     = sp.velocity[2]     = 0.0f;
    sp.acceleration[0] = sp.acceleration[1] = sp.acceleration[2] = 0.0f;
    sp.yaw = NAN;
    traj_pub_->publish(sp);
  }

  // ── FOLLOW_POINT ───────────────────────────────────────────────────

  void execute_follow_point()
  {
    const double dn = follow_n_ - pos_n_;
    const double de = follow_e_ - pos_e_;
    const double dd = follow_d_ - pos_d_;
    const double dist = std::sqrt(dn*dn + de*de + dd*dd);

    // Switch to LOITER once close and slow
    if (dist < 0.5) {
      const double speed = std::sqrt(vel_n_*vel_n_ + vel_e_*vel_e_ + vel_d_*vel_d_);
      if (speed < 0.5) {
        command_type_ = MPCCommand::LOITER;
        loiter_n_ = follow_n_; loiter_e_ = follow_e_; loiter_d_ = follow_d_;
        RCLCPP_INFO(get_logger(),
          "[MPC] FOLLOW_POINT → LOITER  N=%.1f E=%.1f D=%.1f",
          loiter_n_, loiter_e_, loiter_d_);
        execute_loiter();
        return;
      }
    }

    // Braking curve: cap approach speed so the drone can always stop at the target
    const double approach_speed = std::min(max_vel_xy_,
      std::sqrt(2.0 * max_acc_ * dist + 1e-9));

    // Unit vector toward target
    const double inv_dist = 1.0 / (dist + 1e-9);
    const double dir_n = dn * inv_dist;
    const double dir_e = de * inv_dist;
    const double dir_d = dd * inv_dist;

    // Build reference horizon: march toward target at approach_speed, clamp at target
    const int N = mpc_->horizon();
    Eigen::Matrix<double, 6, Eigen::Dynamic> refs(6, N + 1);

    for (int k = 0; k <= N; ++k) {
      const double s = std::min(approach_speed * k * dt_, dist);
      refs(0, k) = pos_n_ + dir_n * s;
      refs(1, k) = pos_e_ + dir_e * s;
      refs(2, k) = pos_d_ + dir_d * s;
      // Velocity reference winds down as reference position reaches the target
      const double ref_speed = (s < dist) ? approach_speed : 0.0;
      refs(3, k) = dir_n * ref_speed;
      refs(4, k) = dir_e * ref_speed;
      refs(5, k) = dir_d * ref_speed;
    }

    Eigen::Matrix<double, 6, 1> state;
    state << pos_n_, pos_e_, pos_d_, vel_n_, vel_e_, vel_d_;

    auto [an, ae, ad] = mpc_->solve(state, refs, u_prev_);
    u_prev_ << an, ae, ad;

    const double dt2 = 0.5 * dt_ * dt_;
    TrajectorySetpoint sp{};
    sp.timestamp       = timestamp_us();
    sp.position[0]     = static_cast<float>(pos_n_ + vel_n_ * dt_ + an * dt2);
    sp.position[1]     = static_cast<float>(pos_e_ + vel_e_ * dt_ + ae * dt2);
    sp.position[2]     = static_cast<float>(pos_d_ + vel_d_ * dt_ + ad * dt2);
    sp.velocity[0]     = static_cast<float>(vel_n_ + an * dt_);
    sp.velocity[1]     = static_cast<float>(vel_e_ + ae * dt_);
    sp.velocity[2]     = static_cast<float>(vel_d_ + ad * dt_);
    sp.acceleration[0] = static_cast<float>(an);
    sp.acceleration[1] = static_cast<float>(ae);
    sp.acceleration[2] = static_cast<float>(ad);
    const double speed = std::sqrt(vel_n_*vel_n_ + vel_e_*vel_e_ + vel_d_*vel_d_);
    sp.yaw      = (speed > 3.0)
                    ? static_cast<float>(std::atan2(vel_e_, vel_n_))
                    : NAN;
    sp.yawspeed = NAN;
    clamp_velocity(sp);
    traj_pub_->publish(sp);
  }

  // ── FOLLOW_OBJECT ──────────────────────────────────────────────────

  void execute_follow_object()
  {
    if (!obj_have_gps_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[MPC] FOLLOW_OBJECT — waiting for first GPS from '%s'",
        current_object_id_.c_str());
      execute_loiter();
      return;
    }

    // Object heading: explicit field preferred, else derive from EMA velocity
    const double spd_obj = std::sqrt(obj_vel_n_*obj_vel_n_ + obj_vel_e_*obj_vel_e_);
    const double psi = std::isfinite(obj_heading_) ? obj_heading_
                     : ((spd_obj > 0.5) ? std::atan2(obj_vel_e_, obj_vel_n_) : 0.0);

    const int N = mpc_->horizon();
    Eigen::Matrix<double, 6, Eigen::Dynamic> refs(6, N + 1);

    const bool have_lookahead = (obj_future_dt_s_.size() >= 2);

    // Seconds elapsed since the last ObjectGps message was received.
    const double elapsed_s = now_control_s_ - obj_last_time_s_;

    for (int k = 0; k <= N; ++k) {
      double ref_n, ref_e, ref_d, ref_vn, ref_ve;

      if (have_lookahead) {
        // --- TEST: interpolate the real F1 lookahead trajectory ---
        // Treat the current object state as t=0 and future_dt_s as offsets from
        // the last GPS stamp, so the time we need for step k is:
        const double t_ahead = elapsed_s + k * dt_;

        if (t_ahead >= obj_future_dt_s_.back()) {
          // Beyond last future sample — clamp
          ref_n  = obj_future_n_.back();  ref_e  = obj_future_e_.back();
          ref_d  = obj_future_d_.back();  ref_vn = obj_future_vn_.back();
          ref_ve = obj_future_ve_.back();
        } else {
          // Find the two bracketing samples.
          // future_dt_s_[0] is the first future point (e.g. 0.1 s ahead).
          // For t_ahead < future_dt_s_[0] we interpolate between current obj pos (t=0)
          // and future[0]; otherwise between future[lo] and future[hi].
          double t_lo, t_hi, n_lo, n_hi, e_lo, e_hi, d_lo, d_hi, vn_lo, vn_hi, ve_lo, ve_hi;

          if (t_ahead <= obj_future_dt_s_[0]) {
            t_lo  = 0.0;       t_hi  = obj_future_dt_s_[0];
            n_lo  = obj_pos_n_; n_hi = obj_future_n_[0];
            e_lo  = obj_pos_e_; e_hi = obj_future_e_[0];
            d_lo  = obj_pos_d_; d_hi = obj_future_d_[0];
            vn_lo = obj_vel_n_; vn_hi = obj_future_vn_[0];
            ve_lo = obj_vel_e_; ve_hi = obj_future_ve_[0];
          } else {
            size_t hi = 1;
            while (hi < obj_future_dt_s_.size() && obj_future_dt_s_[hi] < t_ahead) { ++hi; }
            const size_t lo = hi - 1;
            t_lo  = obj_future_dt_s_[lo];  t_hi  = obj_future_dt_s_[hi];
            n_lo  = obj_future_n_[lo];     n_hi  = obj_future_n_[hi];
            e_lo  = obj_future_e_[lo];     e_hi  = obj_future_e_[hi];
            d_lo  = obj_future_d_[lo];     d_hi  = obj_future_d_[hi];
            vn_lo = obj_future_vn_[lo];    vn_hi = obj_future_vn_[hi];
            ve_lo = obj_future_ve_[lo];    ve_hi = obj_future_ve_[hi];
          }

          const double span  = t_hi - t_lo;
          const double alpha = (span > 1e-9) ? (t_ahead - t_lo) / span : 0.0;
          ref_n  = n_lo  + alpha * (n_hi  - n_lo);
          ref_e  = e_lo  + alpha * (e_hi  - e_lo);
          ref_d  = d_lo  + alpha * (d_hi  - d_lo);
          ref_vn = vn_lo + alpha * (vn_hi - vn_lo);
          ref_ve = ve_lo + alpha * (ve_hi - ve_lo);
        }
      } else {
        // Fallback: constant velocity extrapolation
        ref_n  = obj_pos_n_ + k * dt_ * obj_vel_n_;
        ref_e  = obj_pos_e_ + k * dt_ * obj_vel_e_;
        ref_d  = obj_pos_d_;
        ref_vn = obj_vel_n_;
        ref_ve = obj_vel_e_;
      }

      // Per-step follow offset rotated by the reference heading at this step
      const double spd_ref = std::sqrt(ref_vn * ref_vn + ref_ve * ref_ve);
      const double psi_k = (spd_ref > 0.5) ? std::atan2(ref_ve, ref_vn) : psi;
      const double off_n_k = -follow_radius_ * std::cos(follow_direction_ - psi_k);
      const double off_e_k =  follow_radius_ * std::sin(follow_direction_ - psi_k);

      refs(0, k) = ref_n + off_n_k;
      refs(1, k) = ref_e + off_e_k;
      refs(2, k) = ref_d - follow_alt_above_;
      refs(3, k) = ref_vn;
      refs(4, k) = ref_ve;
      refs(5, k) = 0.0;
    }

    Eigen::Matrix<double, 6, 1> state;
    state << pos_n_, pos_e_, pos_d_, vel_n_, vel_e_, vel_d_;

    auto [an, ae, ad] = mpc_->solve(state, refs, u_prev_);
    u_prev_ << an, ae, ad;

    const double dt2 = 0.5 * dt_ * dt_;
    TrajectorySetpoint sp{};
    sp.timestamp       = timestamp_us();
    sp.position[0]     = static_cast<float>(pos_n_ + vel_n_ * dt_ + an * dt2);
    sp.position[1]     = static_cast<float>(pos_e_ + vel_e_ * dt_ + ae * dt2);
    sp.position[2]     = static_cast<float>(pos_d_ + vel_d_ * dt_ + ad * dt2);
    sp.velocity[0]     = static_cast<float>(vel_n_ + an * dt_);
    sp.velocity[1]     = static_cast<float>(vel_e_ + ae * dt_);
    sp.velocity[2]     = static_cast<float>(vel_d_ + ad * dt_);
    sp.acceleration[0] = static_cast<float>(an);
    sp.acceleration[1] = static_cast<float>(ae);
    sp.acceleration[2] = static_cast<float>(ad);
    // Yaw: point camera toward the object
    const double to_obj_n = obj_pos_n_ - pos_n_;
    const double to_obj_e = obj_pos_e_ - pos_e_;
    sp.yaw      = static_cast<float>(std::atan2(to_obj_e, to_obj_n));
    sp.yawspeed = NAN;
    clamp_velocity(sp);
    traj_pub_->publish(sp);
  }

  // ── FOLLOW_TRAJECTORY ──────────────────────────────────────────────

  void execute_follow_trajectory()
  {
    if (!path_.valid()) { execute_loiter(); return; }

    // 1. Advance s_ref_ via windowed projection (prevents closed-loop jump)
    double s_proj = path_.closest_arc_length(pos_n_, pos_e_, pos_d_, s_ref_);
    s_ref_ = std::max(s_ref_, s_proj);

    // Advance by commanded speed × dt (predictive progress)
    PathSample cur = path_.sample(s_ref_);
    s_ref_ = std::min(s_ref_ + cur.speed * dt_, path_.total_length());

    // 2. Check trajectory completion — wait until drone is actually slow and
    //    close to the endpoint before handing off to LOITER. Switching on
    //    s_ref_ alone causes a hard velocity=0 step while still at cruise speed.
    if (s_ref_ >= path_.total_length()) {
      PathSample ep = path_.sample(path_.total_length());
      const double dn = ep.n - pos_n_;
      const double de = ep.e - pos_e_;
      const double dd = ep.d - pos_d_;
      const double dist = std::sqrt(dn*dn + de*de + dd*dd);
      const double speed = std::sqrt(vel_n_*vel_n_ + vel_e_*vel_e_ + vel_d_*vel_d_);

      if (dist < 0.5 && speed < 0.5) {
        command_type_ = MPCCommand::LOITER;
        loiter_n_ = ep.n; loiter_e_ = ep.e; loiter_d_ = ep.d;
        RCLCPP_INFO(get_logger(),
          "[MPC] FOLLOW_TRAJECTORY → LOITER  N=%.1f E=%.1f D=%.1f",
          loiter_n_, loiter_e_, loiter_d_);
        execute_loiter();
        return;
      }
      // Still moving — keep MPC running with endpoint clamped as target
    }

    // 3. Build MPC state vector
    Eigen::Matrix<double, 6, 1> state;
    state << pos_n_, pos_e_, pos_d_, vel_n_, vel_e_, vel_d_;

    // 4. Build reference trajectory for the full horizon
    const int N = mpc_->horizon();
    Eigen::Matrix<double, 6, Eigen::Dynamic> refs(6, N + 1);

    double s_h = s_ref_;
    for (int k = 0; k <= N; ++k) {
      PathSample r = path_.sample(s_h);

      PathSample r_fwd = path_.sample(
        std::min(s_h + 0.1, path_.total_length()));
      double dn  = r_fwd.n - r.n;
      double de  = r_fwd.e - r.e;
      double dd  = r_fwd.d - r.d;
      double len = std::sqrt(dn*dn + de*de + dd*dd) + 1e-9;
      refs(0, k) = r.n;
      refs(1, k) = r.e;
      refs(2, k) = r.d;
      refs(3, k) = (dn / len) * r.speed;
      refs(4, k) = (de / len) * r.speed;
      refs(5, k) = (dd / len) * r.speed;

      s_h = std::min(s_h + r.speed * dt_, path_.total_length());
    }

    // 5. Solve MPC
    auto [an, ae, ad] = mpc_->solve(state, refs, u_prev_);
    u_prev_ << an, ae, ad;

    // 6. Propagate x*[1] = A·x0 + B·u*[0]  (double integrator, exact ZOH)
    const double dt2 = 0.5 * dt_ * dt_;
    TrajectorySetpoint sp{};
    sp.timestamp       = timestamp_us();
    sp.position[0]     = static_cast<float>(pos_n_ + vel_n_ * dt_ + an * dt2);
    sp.position[1]     = static_cast<float>(pos_e_ + vel_e_ * dt_ + ae * dt2);
    sp.position[2]     = static_cast<float>(pos_d_ + vel_d_ * dt_ + ad * dt2);
    sp.velocity[0]     = static_cast<float>(vel_n_ + an * dt_);
    sp.velocity[1]     = static_cast<float>(vel_e_ + ae * dt_);
    sp.velocity[2]     = static_cast<float>(vel_d_ + ad * dt_);
    sp.acceleration[0] = static_cast<float>(an);
    sp.acceleration[1] = static_cast<float>(ae);
    sp.acceleration[2] = static_cast<float>(ad);
    sp.yaw      = static_cast<float>(path_.sample(s_ref_).yaw);
    sp.yawspeed = NAN;
    clamp_velocity(sp);
    traj_pub_->publish(sp);
  }

  // ── Offboard heartbeat ─────────────────────────────────────────────

  void publish_offboard_control_mode()
  {
    OffboardControlMode msg{};
    msg.timestamp    = timestamp_us();
    msg.position     = true;
    msg.velocity     = true;
    msg.acceleration = true;
    msg.attitude     = false;
    msg.body_rate    = false;
    offboard_pub_->publish(msg);
  }

  // ── Helpers ────────────────────────────────────────────────────────

  static uint64_t timestamp_us()
  {
    return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  // ── Members ────────────────────────────────────────────────────────

  rclcpp::Subscription<MPCCommand>::SharedPtr           cmd_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr        status_sub_;
  rclcpp::Subscription<ObjectGps>::SharedPtr            obj_gps_sub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr      traj_pub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr     offboard_pub_;
  rclcpp::TimerBase::SharedPtr                          timer_;

  std::unique_ptr<MPCSolver> mpc_;
  PathSampler                path_;

  uint8_t         command_type_;
  double          s_ref_;
  Eigen::Vector3d u_prev_;

  // Drone state (from VehicleLocalPosition)
  double pos_n_{0.0}, pos_e_{0.0}, pos_d_{0.0};
  double vel_n_{0.0}, vel_e_{0.0}, vel_d_{0.0};
  double loiter_n_{0.0}, loiter_e_{0.0}, loiter_d_{0.0};
  double follow_n_{0.0}, follow_e_{0.0}, follow_d_{0.0};

  // NED origin (from VehicleLocalPosition, needed for GPS→NED conversion)
  double ref_lat_{0.0}, ref_lon_{0.0}, ref_alt_{0.0};
  bool   have_ned_origin_{false};

  // FOLLOW_OBJECT state
  std::string current_object_id_;
  double follow_radius_{10.0};
  double follow_direction_{0.0};   // radians
  double follow_alt_above_{5.0};   // m

  // Object GPS + EMA velocity estimator
  bool   obj_have_gps_{false};
  double obj_pos_n_{0.0}, obj_pos_e_{0.0}, obj_pos_d_{0.0};
  double obj_vel_n_{0.0}, obj_vel_e_{0.0};
  double obj_heading_{NAN};
  double obj_last_stamp_s_{0.0};  // header.stamp of last GPS (wall clock)
  double obj_last_time_s_{0.0};   // steady_clock time when last GPS arrived

  // TEST: cached future states (NED) from lookahead GPS message
  std::vector<double> obj_future_dt_s_;
  std::vector<double> obj_future_n_, obj_future_e_, obj_future_d_;
  std::vector<double> obj_future_vn_, obj_future_ve_;

  double  now_control_s_{0.0};  // steady_clock time updated each control tick

  bool    have_position_;
  bool    in_offboard_;
  uint8_t nav_state_{0};
  double  dt_;
  double  max_acc_;
  // Active velocity limits — initialised from params
  double  max_vel_xy_;
  double  max_vel_z_up_;
  double  max_vel_z_dn_;
};

}  // namespace unav_mpc

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<unav_mpc::MPCControllerNode>());
  rclcpp::shutdown();
  return 0;
}
