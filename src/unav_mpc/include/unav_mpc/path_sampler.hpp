#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include "unav_msgs/msg/ned_path.hpp"
#include "unav_msgs/msg/ned_waypoint.hpp"

namespace unav_mpc
{

struct PathSample
{
  double n, e, d;   // NED position (meters)
  double yaw;       // heading (radians, NED)
  double speed;     // desired speed (m/s)
};

/**
 * Stores a NedPath and provides arc-length interpolation.
 * The MPC samples this at each 50 Hz step for its horizon.
 */
class PathSampler
{
public:
  PathSampler() = default;

  void load(const unav_msgs::msg::NedPath & path)
  {
    points_ = path.points;
    arc_lengths_ = path.arc_lengths;
    total_length_ = path.total_length;
    valid_ = !points_.empty() && points_.size() == arc_lengths_.size();
  }

  bool valid() const { return valid_; }
  double total_length() const { return total_length_; }

  /**
   * Interpolate path at given arc-length s.
   * Clamps to [0, total_length].
   */
  PathSample sample(double s) const
  {
    s = std::clamp(s, 0.0, total_length_);

    // Binary search for the segment containing s
    auto it = std::lower_bound(arc_lengths_.begin(), arc_lengths_.end(), s);
    size_t idx = std::distance(arc_lengths_.begin(), it);

    if (idx == 0) {
      return to_sample(points_[0]);
    }
    if (idx >= points_.size()) {
      return to_sample(points_.back());
    }

    // Linear interpolation between idx-1 and idx
    double s0 = arc_lengths_[idx - 1];
    double s1 = arc_lengths_[idx];
    double alpha = (s - s0) / (s1 - s0 + 1e-9);

    const auto & p0 = points_[idx - 1];
    const auto & p1 = points_[idx];

    PathSample result;
    result.n = p0.n + alpha * (p1.n - p0.n);
    result.e = p0.e + alpha * (p1.e - p0.e);
    result.d = p0.d + alpha * (p1.d - p0.d);
    result.yaw = lerp_angle(p0.yaw, p1.yaw, alpha);
    result.speed = p0.speed + alpha * (p1.speed - p0.speed);
    return result;
  }

  /**
   * Find the arc-length of the closest point on the path within the window
   * [s_min, s_min + window].  Restricting the search prevents a closed-loop
   * path from jumping to a geometrically identical point near the end when
   * the drone is still near the beginning.
   *
   * s_min   — lower bound (typically current s_ref_)
   * window  — how far ahead to search; large enough to cover any expected lag
   *           but smaller than the loop circumference so duplicates are invisible
   */
  double closest_arc_length(double n, double e, double d,
                             double s_min = 0.0, double window = 10.0) const
  {
    if (points_.empty()) { return 0.0; }

    double s_max = s_min + window;

    // Jump to first index at or after s_min
    auto start_it = std::lower_bound(arc_lengths_.begin(), arc_lengths_.end(), s_min);
    size_t start = std::distance(arc_lengths_.begin(), start_it);
    if (start > 0) { --start; }   // include one point before s_min as a guard

    double best_s   = arc_lengths_[start];
    double best_dist = std::numeric_limits<double>::max();

    for (size_t i = start; i < points_.size(); ++i) {
      if (arc_lengths_[i] > s_max) { break; }
      double dn   = points_[i].n - n;
      double de   = points_[i].e - e;
      double dd   = points_[i].d - d;
      double dist = std::sqrt(dn * dn + de * de + dd * dd);
      if (dist < best_dist) {
        best_dist = dist;
        best_s    = arc_lengths_[i];
      }
    }
    return best_s;
  }

private:
  std::vector<unav_msgs::msg::NedWaypoint> points_;
  std::vector<double> arc_lengths_;
  double total_length_{0.0};
  bool valid_{false};

  static PathSample to_sample(const unav_msgs::msg::NedWaypoint & wp)
  {
    return {wp.n, wp.e, wp.d, wp.yaw, wp.speed};
  }

  // Shortest-path angle interpolation
  static double lerp_angle(double a0, double a1, double t)
  {
    double diff = a1 - a0;
    while (diff >  M_PI) { diff -= 2.0 * M_PI; }
    while (diff < -M_PI) { diff += 2.0 * M_PI; }
    return a0 + t * diff;
  }
};

}  // namespace unav_mpc
