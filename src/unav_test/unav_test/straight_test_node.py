"""
Straight-line test mission.

Waits for a valid position fix, then uploads a 2-waypoint trajectory
from the drone's current location to 100 m directly ahead (North) at
the same altitude. Used to test smooth acceleration and deceleration.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from unav_msgs.srv import UploadMission

CRUISE_SPEED = 8.0   # m/s
DISTANCE     = 100.0  # m — straight-line distance ahead (North)


class StraightTestNode(Node):
    def __init__(self):
        super().__init__("straight_test")
        self._done = False
        self._position = None

        px4_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pos_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._on_position,
            px4_qos,
        )

        self._client = self.create_client(UploadMission, "/orchestrator/upload_mission")

        self.get_logger().info("Waiting for position fix...")
        self._timer = self.create_timer(0.1, self._try_send)

    def _on_position(self, msg):
        if msg.xy_global and msg.z_global:
            self._position = msg

    def _try_send(self):
        if self._done:
            return
        if self._position is None:
            return
        if not self._client.service_is_ready():
            return

        self._timer.cancel()
        self._send_mission()

    def _send_mission(self):
        msg = self._position
        ref_lat = msg.ref_lat
        ref_lon = msg.ref_lon
        ref_alt = msg.ref_alt

        m_per_deg_lat = 111_320.0
        m_per_deg_lon = 111_320.0 * math.cos(math.radians(ref_lat))

        drone_lat = ref_lat + msg.x / m_per_deg_lat
        drone_lon = ref_lon + msg.y / m_per_deg_lon
        drone_alt = ref_alt - msg.z   # NED z is down

        mid_lat = drone_lat + (DISTANCE / 2.0) / m_per_deg_lat
        end_lat = drone_lat + DISTANCE / m_per_deg_lat

        req = UploadMission.Request()
        req.lat               = [mid_lat, end_lat]
        req.lon               = [drone_lon, drone_lon]
        req.alt               = [drone_alt, drone_alt]
        req.cruise_speed      = CRUISE_SPEED
        req.start_from_1st_wp = False

        self.get_logger().info(
            f"Sending straight-line trajectory: {DISTANCE} m North at {CRUISE_SPEED} m/s"
        )
        future = self._client.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
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
    node = StraightTestNode()
    while rclpy.ok() and not node._done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
