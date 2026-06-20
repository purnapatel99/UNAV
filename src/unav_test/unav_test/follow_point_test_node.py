"""
Follow-point test.

Waits for a position fix, picks a random direction and distance (20–150 m),
then sends a single FOLLOW_POINT MPCCommand directly to the MPC controller.
"""

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleLocalPosition
from unav_msgs.msg import MPCCommand


_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

_PX4_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

MIN_DIST = 20.0    # m
MAX_DIST = 150.0   # m


class FollowPointTestNode(Node):
    def __init__(self):
        super().__init__("follow_point_test")
        self._done = False
        self._position = None

        self._pos_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._on_position,
            _PX4_QOS,
        )

        self._cmd_pub = self.create_publisher(MPCCommand, "/mpc/command", _LATCHED_QOS)

        self.get_logger().info("Waiting for position fix...")
        self._timer = self.create_timer(0.1, self._try_send)

    def _on_position(self, msg):
        if msg.xy_valid and msg.z_valid:
            self._position = msg

    def _try_send(self):
        if self._done or self._position is None:
            return
        self._timer.cancel()
        self._send_follow_point()

    def _send_follow_point(self):
        msg = self._position
        pos_n = float(msg.x)
        pos_e = float(msg.y)
        pos_d = float(msg.z)

        dist = random.uniform(MIN_DIST, MAX_DIST)
        angle = random.uniform(0.0, 2.0 * math.pi)   # random bearing in NED plane

        target_n = pos_n + dist * math.cos(angle)
        target_e = pos_e + dist * math.sin(angle)
        target_d = pos_d  # same altitude

        cmd = MPCCommand()
        cmd.command_type = MPCCommand.FOLLOW_POINT
        cmd.follow_n = target_n
        cmd.follow_e = target_e
        cmd.follow_d = target_d

        self._cmd_pub.publish(cmd)
        self.get_logger().info(
            f"FOLLOW_POINT sent: N={target_n:.1f} E={target_e:.1f} D={target_d:.1f} "
            f"(dist={dist:.1f} m, bearing={math.degrees(angle):.1f} deg)"
        )
        self._done = True


def main(args=None):
    rclpy.init(args=args)
    node = FollowPointTestNode()
    while rclpy.ok() and not node._done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
