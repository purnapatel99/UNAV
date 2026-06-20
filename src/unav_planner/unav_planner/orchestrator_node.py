"""
Orchestrator node.

Low-rate mission brain. Receives mission uploads, converts GPS waypoints to
a NED path via the trajectory_generator library, then sends a single
MPCCommand to the MPC controller. The MPC handles all 50Hz execution.

A new command always overrides the previous one in the MPC.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from px4_msgs.msg import VehicleLocalPosition
from unav_msgs.msg import MPCCommand, NedPath, NedWaypoint
from unav_msgs.srv import UploadMission

from .trajectory_generator import gps_to_ned, build_path


# Latched QoS: MPC always gets last command even if it starts after orchestrator
_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

# PX4 publishes all sensor/state topics as BEST_EFFORT — must match or no data arrives
_PX4_QOS = QoSProfile(
    depth=10,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class OrchestratorNode(Node):
    def __init__(self):
        super().__init__("orchestrator")

        # PX4 NED origin — populated from vehicle_local_position
        self._ref_lat = None
        self._ref_lon = None
        self._ref_alt = None

        # Drone's current NED position (x=North, y=East, z=Down)
        self._pos_n = 0.0
        self._pos_e = 0.0
        self._pos_d = 0.0
        self._have_position = False

        self._vehicle_local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._vehicle_local_position_cb,
            _PX4_QOS,
        )

        self._mpc_cmd_pub = self.create_publisher(
            MPCCommand,
            "/mpc/command",
            _LATCHED_QOS,
        )

        self._upload_mission_srv = self.create_service(
            UploadMission,
            "/orchestrator/upload_mission",
            self._upload_mission_cb,
        )

        # Shared motion limits — loaded from flight_limits.yaml via launch file.
        # Keep in sync with PX4 params (see config/flight_limits.yaml comments).
        self.declare_parameter("max_vel_xy",  12.0)  # MPC_XY_VEL_MAX
        self.declare_parameter("max_acc",      4.0)  # our own acc constraint

        # Publish initial LOITER command so MPC has something on startup
        self._publish_loiter()
        self.get_logger().info("Orchestrator ready. Waiting for NED origin from PX4...")

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _vehicle_local_position_cb(self, msg: VehicleLocalPosition):
        if msg.ref_lat != 0.0 and msg.ref_lon != 0.0:
            self._ref_lat = msg.ref_lat
            self._ref_lon = msg.ref_lon
            self._ref_alt = msg.ref_alt

        # Always track current drone position in NED frame
        self._pos_n = float(msg.x)   # North (m)
        self._pos_e = float(msg.y)   # East  (m)
        self._pos_d = float(msg.z)   # Down  (m)
        self._have_position = True

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _upload_mission_cb(self, request: UploadMission.Request,
                           response: UploadMission.Response):
        if self._ref_lat is None or not self._have_position:
            response.success = False
            response.message = "No NED origin from PX4 yet. Is the drone localised?"
            return response

        if len(request.lat) < 2:
            response.success = False
            response.message = "Need at least 2 waypoints."
            return response

        if len(request.lat) != len(request.lon) or len(request.lat) != len(request.alt):
            response.success = False
            response.message = "lat, lon, alt arrays must have equal length."
            return response

        max_vel_xy = self.get_parameter("max_vel_xy").get_parameter_value().double_value
        max_acc    = self.get_parameter("max_acc").get_parameter_value().double_value

        requested_speed = request.cruise_speed if request.cruise_speed > 0.0 else 5.0
        cruise_speed = min(requested_speed, max_vel_xy)
        if cruise_speed < requested_speed:
            self.get_logger().warn(
                f"Requested cruise speed {requested_speed:.1f} m/s exceeds "
                f"max_vel_xy={max_vel_xy:.1f} m/s — clamped to {cruise_speed:.1f} m/s"
            )

        try:
            ned_pts = gps_to_ned(
                request.lat, request.lon, request.alt,
                self._ref_lat, self._ref_lon, self._ref_alt,
            )

            if request.start_from_1st_wp:
                brake_dist = (cruise_speed ** 2) / (2.0 * max_acc)
                dn = ned_pts[0, 0] - self._pos_n
                de = ned_pts[0, 1] - self._pos_e
                dd = ned_pts[0, 2] - self._pos_d
                dist_to_first = float(np.sqrt(dn**2 + de**2 + dd**2))
                if dist_to_first > brake_dist:
                    drone_pos = np.array([[self._pos_n, self._pos_e, self._pos_d]])
                    ned_pts = np.vstack([drone_pos, ned_pts])
                    self.get_logger().info(
                        f"start_from_1st_wp: dist={dist_to_first:.1f} m > "
                        f"brake={brake_dist:.1f} m — prepending drone pos"
                    )

            path = build_path(ned_pts, cruise_speed=cruise_speed, decel_acc=max_acc / 3.0)
        except Exception as e:
            response.success = False
            response.message = f"Trajectory generation failed: {e}"
            return response

        cmd = self._build_follow_trajectory_cmd(path)
        self._mpc_cmd_pub.publish(cmd)

        response.success = True
        response.total_length_m = path["total_length"]
        response.message = (
            f"Mission uploaded: {len(path['arc_lengths'])} path points, "
            f"{path['total_length']:.1f} m total, "
            f"{cruise_speed:.1f} m/s cruise speed."
        )
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Command builders
    # ------------------------------------------------------------------

    def _build_follow_trajectory_cmd(self, path: dict) -> MPCCommand:
        cmd = MPCCommand()
        cmd.header = self._stamp()
        cmd.command_type = MPCCommand.FOLLOW_TRAJECTORY

        ned_path = NedPath()
        ned_path.header = self._stamp()
        ned_path.total_length = path["total_length"]
        ned_path.sample_spacing = path["sample_spacing"]
        ned_path.arc_lengths = path["arc_lengths"].tolist()

        waypoints = []
        for i in range(len(path["arc_lengths"])):
            wp = NedWaypoint()
            wp.n = float(path["positions"][i, 0])
            wp.e = float(path["positions"][i, 1])
            wp.d = float(path["positions"][i, 2])
            wp.yaw = float(path["yaws"][i])
            wp.speed = float(path["speeds"][i])
            waypoints.append(wp)
        ned_path.points = waypoints

        cmd.trajectory = ned_path
        return cmd

    def _publish_loiter(self):
        cmd = MPCCommand()
        cmd.header = self._stamp()
        cmd.command_type = MPCCommand.LOITER
        self._mpc_cmd_pub.publish(cmd)

    def _stamp(self) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = "map"
        return h


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
