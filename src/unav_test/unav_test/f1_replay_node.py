"""
F1 replay node — replays a real F1 lap as ObjectGps messages.

Downloads actual F1 telemetry via FastF1 and replays it at real-time speed.
Position and speed come from official F1 timing data for a specified driver/lap.

Coordinate approach:
  FastF1 X/Y are in a circuit-local frame (scale ≈ 0.1 m/unit, arbitrary axes).
  Rather than inverting the unknown absolute frame, we dead-reckon from the
  datum GPS using *relative* heading changes derived from consecutive X/Y deltas,
  then rotate the entire sequence so the initial heading matches the known
  geographic bearing of COTA's main straight.

  COTA defaults:
    datum      = start/finish line GPS (GPS-verified)
    initial_heading_deg = 124.0 (SE — bearing from S/F line toward T1 apex)
    scale      = 0.1            (FastF1 units → metres)

Requires:
  pip install fastf1

First run downloads ~100 MB from the F1 API into ~/.fastf1_cache.
Subsequent runs are instant (cached).

Usage:
  ros2 run unav_test f1_replay
  ros2 run unav_test f1_replay --ros-args -p driver:=LEC -p session:=R
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from unav_msgs.msg import ObjectGps

# ── COTA datum — must match PX4_HOME_LAT / PX4_HOME_LON in your SITL setup ──
COTA_LAT = 30.131877   # °N  start/finish line (GPS-verified)
COTA_LON = -97.639840  # °W
COTA_ALT = 165.0       # m ASL  (COTA elevation)

_M_PER_DEG_LAT = 111_320.0
_M_PER_DEG_LON = _M_PER_DEG_LAT * math.cos(math.radians(COTA_LAT))

_OBJ_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def _ned_to_gps(datum_lat, datum_lon, datum_alt, n_m, e_m):
    return (
        datum_lat + n_m / _M_PER_DEG_LAT,
        datum_lon + e_m / _M_PER_DEG_LON,
        datum_alt,
    )


class F1ReplayNode(Node):
    def __init__(self):
        super().__init__("f1_replay")

        self.declare_parameter("object_id",           "f1_car")
        self.declare_parameter("year",                2024)
        self.declare_parameter("event",               "United States Grand Prix")
        self.declare_parameter("session",             "Q")     # Q=Qualifying  R=Race
        self.declare_parameter("driver",              "VER")   # three-letter driver code
        self.declare_parameter("lap",                 -1)      # -1 = fastest lap
        self.declare_parameter("publish_hz",          10.0)
        self.declare_parameter("loop",                True)
        # TEST: how many future waypoints to pack into each ObjectGps message.
        # 10 points @ 0.1 s spacing = 1.0 s lookahead, covers the 0.6 s MPC horizon.
        self.declare_parameter("n_lookahead",         10)
        self.declare_parameter("cache_dir",           os.path.expanduser("~/.fastf1_cache"))
        # Datum GPS — override to relocate the track in the sim world
        self.declare_parameter("datum_lat",           COTA_LAT)
        self.declare_parameter("datum_lon",           COTA_LON)
        self.declare_parameter("datum_alt",           COTA_ALT)
        # Geographic bearing (°, 0=N, 90=E) of the car's travel direction at
        # the very start of the lap.  Computed from GPS: bearing S/F → T1 apex ≈ 124°.
        # Tune this if the track shape appears rotated in the sim.
        self.declare_parameter("initial_heading_deg", 124.0)
        # FastF1 units → metres. Derived: COTA 5513 m / 54018 FastF1 units ≈ 0.1
        self.declare_parameter("scale",               0.1)

        self._object_id  = self.get_parameter("object_id").value
        self._loop       = self.get_parameter("loop").value
        pub_hz           = self.get_parameter("publish_hz").value
        self._dt         = 1.0 / pub_hz
        self._idx        = 0
        self._n_lookahead = self.get_parameter("n_lookahead").value

        self._waypoints = self._load_f1_data()

        topic = f"/objects/{self._object_id}/gps"
        self._pub = self.create_publisher(ObjectGps, topic, _OBJ_QOS)
        self.create_timer(self._dt, self._publish)

        lap_s = self._waypoints[-1]["t"]
        spd_max = max(w["speed"] for w in self._waypoints)
        self.get_logger().info(
            f"F1 replay ready: {len(self._waypoints)} pts  "
            f"lap {lap_s:.1f} s  top speed {spd_max * 3.6:.0f} km/h  "
            f"topic='{topic}'  loop={self._loop}"
        )

    # ── Data loading ─────────────────────────────────────────────────────────

    def _load_f1_data(self) -> list:
        try:
            import fastf1  # noqa: PLC0415
        except ImportError:
            self.get_logger().fatal("fastf1 not installed — run: pip install fastf1")
            raise

        cache_dir = self.get_parameter("cache_dir").value
        os.makedirs(cache_dir, exist_ok=True)
        fastf1.Cache.enable_cache(cache_dir)

        year        = self.get_parameter("year").value
        event       = self.get_parameter("event").value
        session_str = self.get_parameter("session").value
        driver      = self.get_parameter("driver").value
        lap_idx     = self.get_parameter("lap").value
        init_hdg    = math.radians(self.get_parameter("initial_heading_deg").value)
        scale       = self.get_parameter("scale").value
        datum_lat   = self.get_parameter("datum_lat").value
        datum_lon   = self.get_parameter("datum_lon").value
        datum_alt   = self.get_parameter("datum_alt").value

        self.get_logger().info(
            f"Loading FastF1: {year} {event} {session_str}  "
            f"driver={driver}  lap={'fastest' if lap_idx == -1 else lap_idx}"
        )

        session = fastf1.get_session(year, event, session_str)
        session.load(telemetry=True, laps=True, weather=False, messages=False)

        driver_laps = session.laps.pick_driver(driver)
        lap = driver_laps.pick_fastest() if lap_idx == -1 else driver_laps.iloc[lap_idx]
        self.get_logger().info(
            f"  Lap #{int(lap['LapNumber'])}  time: {lap['LapTime']}"
        )

        # ── Position data ─────────────────────────────────────────────────────
        pos = lap.get_pos_data()
        x   = pos["X"].values.astype(float)
        y   = pos["Y"].values.astype(float)
        t_s = pos["Time"].dt.total_seconds().values  # seconds from lap start

        # Drop NaN rows — np.interp does not handle NaN and produces jumps.
        valid = ~(np.isnan(x) | np.isnan(y) | np.isnan(t_s))
        if not np.all(valid):
            self.get_logger().warn(
                f"  Dropped {(~valid).sum()} NaN position samples"
            )
            x, y, t_s = x[valid], y[valid], t_s[valid]

        # Deduplicate timestamps — np.interp requires strictly increasing x.
        _, uniq = np.unique(t_s, return_index=True)
        x, y, t_s = x[uniq], y[uniq], t_s[uniq]

        # ── Car data (speed) ──────────────────────────────────────────────────
        car       = lap.get_car_data()
        car_t     = car["Time"].dt.total_seconds().values
        car_spd   = car["Speed"].values / 3.6   # km/h → m/s
        # Interpolate speed onto the position time grid
        speed_arr = np.interp(t_s, car_t, car_spd)

        # ── Resample to uniform time grid at publish_hz ───────────────────────
        t0       = t_s[0]
        t_rel    = t_s - t0
        t_uni    = np.arange(0.0, t_rel[-1], self._dt)

        x_uni    = np.interp(t_uni, t_rel, x)
        # FastF1 Y is screen-down; negate so +Y points north in the circuit frame.
        y_uni    = -np.interp(t_uni, t_rel, y)
        spd_uni  = np.interp(t_uni, t_rel, speed_arr)

        # ── Compute headings in FastF1 frame via centered differences ─────────
        dx = np.gradient(x_uni)
        dy = np.gradient(y_uni)
        ff1_hdg = np.arctan2(dy, dx)   # heading in FastF1 frame, rad

        # ── Rotation: FastF1 frame → geographic frame ─────────────────────────
        # Use a smoothed initial direction (first 10 pts) to avoid noise.
        n_smooth = min(10, len(x_uni) - 1)
        ff1_init = math.atan2(y_uni[n_smooth] - y_uni[0],
                              x_uni[n_smooth] - x_uni[0])
        rot      = init_hdg - ff1_init

        geo_hdg = ff1_hdg + rot   # geographic heading (0=N, π/2=E)

        # ── Direct rotation transform: FastF1 → NED (no drift) ───────────────
        # Mathematically identical to dead-reckoning the path integral but
        # computed in one step from the original X/Y, so errors don't accumulate.
        #   N = scale * (cos(rot)*(x-x0) - sin(rot)*(y-y0))
        #   E = scale * (sin(rot)*(x-x0) + cos(rot)*(y-y0))
        cos_r  = math.cos(rot)
        sin_r  = math.sin(rot)
        dx_ff1 = x_uni - x_uni[0]
        dy_ff1 = y_uni - y_uni[0]
        n_arr  = scale * (cos_r * dx_ff1 - sin_r * dy_ff1)
        e_arr  = scale * (sin_r * dx_ff1 + cos_r * dy_ff1)

        # ── Build waypoint list ───────────────────────────────────────────────
        waypoints = []
        for i in range(len(t_uni)):
            lat, lon, alt = _ned_to_gps(datum_lat, datum_lon, datum_alt,
                                        n_arr[i], e_arr[i])
            waypoints.append({
                "t":       float(t_uni[i]),
                "lat":     lat,
                "lon":     lon,
                "alt":     alt,
                "heading": float(geo_hdg[i]),
                "speed":   float(spd_uni[i]),
            })

        self.get_logger().info(
            f"  Track loaded: {len(waypoints)} waypoints  "
            f"scale={scale} m/unit  initial_heading={math.degrees(init_hdg):.1f}°"
        )
        return waypoints

    # ── Timer callback ────────────────────────────────────────────────────────

    def _publish(self):
        if self._idx >= len(self._waypoints):
            if self._loop:
                self._idx = 0
            else:
                return

        wp  = self._waypoints[self._idx]
        n   = len(self._waypoints)

        msg = ObjectGps()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.object_id       = self._object_id
        msg.lat             = wp["lat"]
        msg.lon             = wp["lon"]
        msg.alt             = wp["alt"]
        msg.heading         = float(wp["heading"])
        msg.speed           = float(wp["speed"])

        # --- TEST: pack future states so MPC can use real trajectory lookahead ---
        future_dt_s   = []
        future_lat    = []
        future_lon    = []
        future_alt    = []
        future_heading = []
        future_speed  = []
        for j in range(1, self._n_lookahead + 1):
            fwd_idx = self._idx + j
            if fwd_idx >= n:
                if self._loop:
                    fwd_idx = fwd_idx % n
                else:
                    break
            fwp = self._waypoints[fwd_idx]
            future_dt_s.append(j * self._dt)
            future_lat.append(fwp["lat"])
            future_lon.append(fwp["lon"])
            future_alt.append(fwp["alt"])
            future_heading.append(float(fwp["heading"]))
            future_speed.append(float(fwp["speed"]))

        msg.future_dt_s    = future_dt_s
        msg.future_lat     = future_lat
        msg.future_lon     = future_lon
        msg.future_alt     = future_alt
        msg.future_heading = future_heading
        msg.future_speed   = future_speed

        self._pub.publish(msg)
        self._idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = F1ReplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
