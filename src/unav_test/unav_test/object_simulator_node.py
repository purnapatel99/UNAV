"""
Object simulator — continuously publishes ObjectGps on a stadium-oval track.

Drives a simulated object around a large oval track at F1-car speeds to stress-test
the drone MPC at its limits.

Track:
  Centre:    LAT_CENTRE / LON_CENTRE (PX4 SITL default home)
  Straights: 2 × 2000 m (North/South)
  Curves:    2 × semicircle, radius 300 m
  Total:     ~5885 m (~5.9 km, comparable to a real F1 circuit)

Default speed: 67.056 m/s (150 mph)

Publishes only ObjectGps — sending the FOLLOW_OBJECT command is the
responsibility of follow_object_test_node, so this node can keep running
while follow parameters are changed without restarting the simulation.

Usage:
  ros2 run unav_test object_simulator
  ros2 run unav_test object_simulator --ros-args -p speed:=67.056
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from unav_msgs.msg import ObjectGps

# ── Track parameters (mirror test_mission_node.py) ────────────────────────────
LAT_CENTRE    = 47.397971
LON_CENTRE    =  8.546164
ALT           = 30.0

STRAIGHT_HALF = 1000.0   # 2000 m straight (F1 scale)
CURVE_RADIUS  =  300.0   # 300 m semicircle radius

_M_PER_DEG_LAT = 111_320.0
_M_PER_DEG_LON = _M_PER_DEG_LAT * math.cos(math.radians(LAT_CENTRE))

_OBJ_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def _ned_to_gps(n_m: float, e_m: float) -> tuple:
    lat = LAT_CENTRE + n_m / _M_PER_DEG_LAT
    lon = LON_CENTRE + e_m / _M_PER_DEG_LON
    return lat, lon, ALT


def _track_state(s: float) -> tuple:
    """
    Arc-length parameterised stadium oval (clockwise in NED, North up).

    Phases (R=CURVE_RADIUS, L=STRAIGHT_HALF):
      0  [0,      2L]      North straight — heading East
      1  [2L,  2L+πR]     East  semicircle — East→South
      2  [2L+πR, 4L+πR]  South straight — heading West
      3  [4L+πR, 4L+2πR] West  semicircle — West→North

    Returns (n, e, heading_rad) — heading in NED convention (0=N, π/2=E).
    """
    R = CURVE_RADIUS
    L = STRAIGHT_HALF
    total = 4 * L + 2 * math.pi * R

    s = math.fmod(s, total)
    if s < 0:
        s += total

    if s < 2 * L:
        n = R
        e = -L + s
        heading = math.pi / 2
    elif s < 2 * L + math.pi * R:
        alpha = math.pi / 2 - (s - 2 * L) / R
        n = R * math.sin(alpha)
        e = L + R * math.cos(alpha)
        heading = math.atan2(math.sin(alpha), -math.cos(alpha))
    elif s < 4 * L + math.pi * R:
        n = -R
        e = L - (s - 2 * L - math.pi * R)
        heading = -math.pi / 2
    else:
        beta = -math.pi / 2 - (s - 4 * L - math.pi * R) / R
        n = R * math.sin(beta)
        e = -L + R * math.cos(beta)
        heading = math.atan2(math.sin(beta), -math.cos(beta))

    return n, e, heading


class ObjectSimulatorNode(Node):
    def __init__(self):
        super().__init__("object_simulator")

        self.declare_parameter("object_id",  "car_01")
        self.declare_parameter("speed",      67.056)  # m/s  (150 mph)
        self.declare_parameter("publish_hz", 20.0)    # 20 Hz keeps step size ~3.4 m at 150 mph

        self._object_id = self.get_parameter("object_id").get_parameter_value().string_value
        self._speed     = self.get_parameter("speed").get_parameter_value().double_value
        pub_hz          = self.get_parameter("publish_hz").get_parameter_value().double_value

        self._dt = 1.0 / pub_hz
        self._s  = 0.0

        total = 4 * STRAIGHT_HALF + 2 * math.pi * CURVE_RADIUS
        topic = f"/objects/{self._object_id}/gps"

        self._pub = self.create_publisher(ObjectGps, topic, _OBJ_QOS)
        self.create_timer(self._dt, self._publish)

        self.get_logger().info(
            f"Object simulator running: id='{self._object_id}'  topic='{topic}'  "
            f"track ~{total:.0f} m  speed={self._speed:.1f} m/s  "
            f"lap ~{total / self._speed:.0f} s"
        )

    def _publish(self):
        n, e, heading = _track_state(self._s)
        lat, lon, alt = _ned_to_gps(n, e)

        msg = ObjectGps()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.object_id       = self._object_id
        msg.lat             = lat
        msg.lon             = lon
        msg.alt             = alt
        msg.heading         = float(heading)
        msg.speed           = float(self._speed)

        self._pub.publish(msg)
        self._s += self._speed * self._dt


def main(args=None):
    rclpy.init(args=args)
    node = ObjectSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
