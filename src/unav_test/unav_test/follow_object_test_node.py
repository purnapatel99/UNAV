"""
Follow-object test node.

Sends a single FOLLOW_OBJECT MPCCommand and exits.
The object simulator must already be running.

Direction convention:
    0   = directly behind the object  (default)
   90   = to the right
  -90   = to the left
  180   = directly in front

Usage:
  ros2 run unav_test follow_object_test                          # follow from behind
  ros2 run unav_test follow_object_test --ros-args -p direction:=-90.0  # from left
  ros2 run unav_test follow_object_test --ros-args -p direction:=90.0   # from right
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor

from unav_msgs.msg import MPCCommand

_DYN = ParameterDescriptor(dynamic_typing=True)  # accept int or float from CLI

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


class FollowObjectTestNode(Node):
    def __init__(self):
        super().__init__("follow_object_test")

        self.declare_parameter("object_id",       "f1_car")
        self.declare_parameter("direction",        0.0, _DYN)  # degrees — 0=behind 90=right -90=left 180=front
        self.declare_parameter("radius",          15.0, _DYN)  # m — larger for F1 speeds
        self.declare_parameter("altitude_above",  10.0, _DYN)  # m

        object_id = self.get_parameter("object_id").get_parameter_value().string_value
        direction  = float(self.get_parameter("direction").value)
        radius     = float(self.get_parameter("radius").value)
        alt_above  = float(self.get_parameter("altitude_above").value)

        pub = self.create_publisher(MPCCommand, "/mpc/command", _LATCHED_QOS)

        cmd = MPCCommand()
        cmd.command_type          = MPCCommand.FOLLOW_OBJECT
        cmd.object_id             = object_id
        cmd.follow_radius         = radius
        cmd.follow_direction      = direction
        cmd.follow_altitude_above = alt_above
        pub.publish(cmd)

        self.get_logger().info(
            f"FOLLOW_OBJECT sent: object='{object_id}'  "
            f"direction={direction:.0f}°  radius={radius:.0f} m  alt_above={alt_above:.0f} m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowObjectTestNode()
    rclpy.spin_once(node, timeout_sec=0.5)  # give latched pub time to deliver
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
