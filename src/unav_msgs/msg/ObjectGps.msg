std_msgs/Header header

string object_id   # matches the ID used in MPCCommand

float64 lat        # degrees
float64 lon        # degrees
float64 alt        # meters AMSL

# Heading in NED convention (radians, 0 = North, clockwise positive).
# Set to NaN when unknown — MPC will derive heading from velocity direction instead.
float32 heading

float32 speed      # m/s, informational only

# --- TEST: F1 lookahead future states ---
# future_dt_s[i] = seconds ahead of header.stamp for the i-th future sample.
# All arrays empty when no lookahead data is available (legacy fallback applies).
float64[] future_dt_s
float64[] future_lat
float64[] future_lon
float64[] future_alt
float32[] future_heading
float32[] future_speed
