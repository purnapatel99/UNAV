"""
Trajectory generator library.

Converts a list of GPS waypoints (lat, lon, alt) into a dense NED path
using PX4's local coordinate origin. The path is arc-length parameterised
so the MPC can sample any look-ahead horizon without time-base coupling.
"""

import math
import numpy as np
from scipy.interpolate import CubicSpline


# WGS-84 semi-major axis (meters)
_R_EARTH = 6378137.0


def gps_to_ned(lats, lons, alts, ref_lat, ref_lon, ref_alt):
    """
    Convert GPS coordinates to NED using PX4's local origin (flat-earth).

    Args:
        lats, lons, alts: lists of float (degrees, degrees, meters AMSL)
        ref_lat, ref_lon: PX4 origin (degrees) from vehicle_local_position
        ref_alt: PX4 origin altitude (meters AMSL)

    Returns:
        ned_points: np.ndarray shape (N, 3) columns [N, E, D]
    """
    ref_lat_rad = math.radians(ref_lat)
    cos_lat = math.cos(ref_lat_rad)

    ned_points = []
    for lat, lon, alt in zip(lats, lons, alts):
        d_lat = math.radians(lat - ref_lat)
        d_lon = math.radians(lon - ref_lon)
        n = d_lat * _R_EARTH
        e = d_lon * _R_EARTH * cos_lat
        d = -(alt - ref_alt)  # NED: down is positive
        ned_points.append([n, e, d])

    return np.array(ned_points)


def build_path(ned_points, cruise_speed=5.0, sample_spacing=0.5, decel_acc=4.0):
    """
    Fit a cubic spline through NED points and sample at fixed arc-length intervals.

    Parameterises by chord length to avoid oscillation with uneven spacing.
    Returns dense arrays ready to embed in a NedPath message.

    Args:
        ned_points: np.ndarray shape (M, 3)
        cruise_speed: desired speed along path (m/s)
        sample_spacing: arc-length between output samples (meters)
        decel_acc: deceleration used to compute the speed ramp-down distance (m/s²)

    Returns:
        dict with keys:
            positions  - np.ndarray (N, 3) NED
            yaws       - np.ndarray (N,)   radians, NED convention
            speeds     - np.ndarray (N,)   m/s
            arc_lengths - np.ndarray (N,)  meters
            total_length - float
    """
    if len(ned_points) < 2:
        raise ValueError("Need at least 2 waypoints")

    # Chord-length parameterisation
    diffs = np.diff(ned_points, axis=0)
    chord_lengths = np.linalg.norm(diffs, axis=1)
    t = np.concatenate([[0.0], np.cumsum(chord_lengths)])

    # Fit independent cubic splines per axis
    cs_n = CubicSpline(t, ned_points[:, 0])
    cs_e = CubicSpline(t, ned_points[:, 1])
    cs_d = CubicSpline(t, ned_points[:, 2])

    # Dense evaluation to compute accurate arc length
    t_dense = np.linspace(0, t[-1], max(1000, int(t[-1] * 20)))
    n_d = cs_n(t_dense)
    e_d = cs_e(t_dense)
    d_d = cs_d(t_dense)

    dn = np.diff(n_d)
    de = np.diff(e_d)
    dd = np.diff(d_d)
    ds = np.sqrt(dn**2 + de**2 + dd**2)
    arc_dense = np.concatenate([[0.0], np.cumsum(ds)])
    total_length = arc_dense[-1]

    # Resample at fixed arc-length intervals
    arc_samples = np.arange(0.0, total_length, sample_spacing)
    if arc_samples[-1] < total_length:
        arc_samples = np.append(arc_samples, total_length)

    # arc → t mapping via interpolation
    t_samples = np.interp(arc_samples, arc_dense, t_dense)

    positions = np.column_stack([
        cs_n(t_samples),
        cs_e(t_samples),
        cs_d(t_samples),
    ])

    # Yaw from tangent (NED: yaw = atan2(East, North))
    dn_dt = cs_n(t_samples, 1)
    de_dt = cs_e(t_samples, 1)
    yaws = np.arctan2(de_dt, dn_dt)

    speeds = np.full(len(arc_samples), cruise_speed)

    # Ramp speed down over the braking distance so the drone decelerates smoothly.
    brake_dist = cruise_speed ** 2 / (2.0 * decel_acc)
    ramp_start = total_length - brake_dist
    for i, s in enumerate(arc_samples):
        if s >= ramp_start:
            t = (s - ramp_start) / brake_dist  # 0 → 1
            speeds[i] = cruise_speed * (1.0 - t)
    speeds[-1] = 0.0  # guarantee exact zero at endpoint

    return {
        "positions": positions,
        "yaws": yaws,
        "speeds": speeds,
        "arc_lengths": arc_samples,
        "total_length": total_length,
        "sample_spacing": sample_spacing,
    }
