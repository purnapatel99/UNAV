# Dense arc-length sampled path in NED frame.
# Points are sampled at fixed arc-length intervals from the spline.
# MPC uses arc_lengths + points to sample its horizon via interpolation.

std_msgs/Header header

NedWaypoint[] points        # densely sampled path points
float64[] arc_lengths       # cumulative arc-length at each point (meters)
float64 total_length        # total path arc length (meters)
float64 sample_spacing      # arc-length between consecutive points (meters)
