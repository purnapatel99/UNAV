# Command from Orchestrator to MPC controller.
# A new message overrides the previous command immediately.

std_msgs/Header header

uint8 LOITER            = 0
uint8 FOLLOW_TRAJECTORY = 1
uint8 FOLLOW_OBJECT     = 2
uint8 FOLLOW_POINT      = 3

uint8 command_type

# FOLLOW_TRAJECTORY fields
NedPath trajectory

# FOLLOW_OBJECT fields
# MPC subscribes to /objects/<object_id>/gps for the target's GPS stream.
#
# follow_direction: angle (degrees) relative to the object's heading.
#   0   = directly behind the object
#   90  = to the right
#   180 = directly in front
#  -90  = to the left
#
# follow_altitude_above: how far above the object the drone holds (meters).
string object_id
float64 follow_radius           # m — horizontal distance from object
float64 follow_direction        # degrees — see above
float64 follow_altitude_above   # m — altitude above object

# FOLLOW_POINT fields
# Single NED target — MPC flies there directly, self-braking via braking curve.
float64 follow_n
float64 follow_e
float64 follow_d
