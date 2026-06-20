"""
Test mission node.

Calls /orchestrator/upload_mission with a stadium-oval track to exercise
the full pipeline: orchestrator → trajectory generator → MPC.

Track shape: two 200 m straights (N and S) + two 50 m radius semicircles
(E and W), giving ~714 m total circuit at 8 m/s cruise (~89 s per lap).

Centre matches the PX4 SITL default home position (default.sdf).
"""

import math
import rclpy
from rclpy.node import Node
from unav_msgs.srv import UploadMission


# ── Track parameters ──────────────────────────────────────────────────────────
LAT_CENTRE = 47.397971   # matches PX4 SITL default home
LON_CENTRE = 8.546164
ALT        = 30.0        # m above home

STRAIGHT_HALF = 100.0    # m — half-length of each straight section
CURVE_RADIUS  = 50.0     # m — semicircle radius (= track half-width)
CRUISE_SPEED  = 8.0      # m/s

# ── WGS-84 flat-earth conversion ──────────────────────────────────────────────
_M_PER_DEG_LAT = 111_320.0
_M_PER_DEG_LON = 111_320.0 * math.cos(math.radians(LAT_CENTRE))


def _wp(n_m: float, e_m: float) -> tuple:
    return (
        LAT_CENTRE + n_m / _M_PER_DEG_LAT,
        LON_CENTRE + e_m / _M_PER_DEG_LON,
        ALT,
    )


def _build_track() -> list:
    """
    Stadium oval, traversed counter-clockwise when viewed from above (North up).

    Layout (all in metres NED):
      North straight : N = +CURVE_RADIUS, E from -STRAIGHT_HALF → +STRAIGHT_HALF
      East semicircle: centre (0, +STRAIGHT_HALF), going N→S
      South straight : N = -CURVE_RADIUS, E from +STRAIGHT_HALF → -STRAIGHT_HALF
      West semicircle: centre (0, -STRAIGHT_HALF), going S→N
    """
    R = CURVE_RADIUS
    L = STRAIGHT_HALF
    pts = []

    # North straight — 5 points heading east
    for k in range(5):
        pts.append(_wp(R, -L + k * L / 2))          # E = -100, -50, 0, 50, 100

    # East semicircle — 5 intermediate + endpoint, going south
    for i in range(1, 7):
        theta = math.radians(90 - i * 30)            # 60°, 30°, 0°, -30°, -60°, -90°
        pts.append(_wp(R * math.sin(theta), L + R * math.cos(theta)))

    # South straight — 4 points heading west (skip first, already added by curve)
    for k in range(1, 5):
        pts.append(_wp(-R, L - k * L / 2))           # E = 50, 0, -50, -100

    # West semicircle — 5 intermediate, going north
    for i in range(1, 6):
        theta = math.radians(-90 - i * 30)           # -120°…-240°
        pts.append(_wp(R * math.sin(theta), -L + R * math.cos(theta)))

    # Close the loop
    pts.append(pts[0])

    return pts


_WAYPOINTS = _build_track()


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class TestMissionNode(Node):
    def __init__(self):
        super().__init__("test_mission")
        self._done = False
        self._client = self.create_client(UploadMission, "/orchestrator/upload_mission")

        self.get_logger().info(
            f"Stadium oval: {len(_WAYPOINTS)} waypoints, "
            f"~{2*(2*STRAIGHT_HALF) + 2*math.pi*CURVE_RADIUS:.0f} m circuit, "
            f"{CRUISE_SPEED} m/s cruise"
        )
        self.get_logger().info("Waiting for /orchestrator/upload_mission service...")

        if not self._client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                "Service /orchestrator/upload_mission not available after 10 s. "
                "Is the orchestrator running?"
            )
            self._done = True
            return

        self._send_mission()

    def _send_mission(self):
        req = UploadMission.Request()
        req.lat               = [wp[0] for wp in _WAYPOINTS]
        req.lon               = [wp[1] for wp in _WAYPOINTS]
        req.alt               = [wp[2] for wp in _WAYPOINTS]
        req.cruise_speed      = CRUISE_SPEED
        req.start_from_1st_wp = True

        future = self._client.call_async(req)
        future.add_done_callback(self._mission_response_cb)

    def _mission_response_cb(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self._done = True
            return

        if response.success:
            self.get_logger().info(f"Mission accepted: {response.message}")
        else:
            self.get_logger().error(f"Mission rejected: {response.message}")

        self._done = True


def main(args=None):
    rclpy.init(args=args)
    node = TestMissionNode()
    while rclpy.ok() and not node._done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
