# UNAV — Universal Navigation

Multi-airframe autonomous path planning framework built on ROS2 and PX4.

**Capabilities**
- Object following (GPS-based formation control)
- Waypoint navigation (spatial spline with contouring MPC)
- Obstacle avoidance (MPC constraints)
- Airframe-agnostic: quadrotor → quadtailsitter → fixed-wing

**Stack:** ROS2 Humble · PX4 v1.15 · ACADOS MPC · Jetson Orin NX

---

## Packages

| Package | Language | Role |
|---|---|---|
| `unav_msgs` | — | Custom ROS2 messages and services |
| `unav_mpc` | C++ | ACADOS-based MPC controller (50 Hz) |
| `unav_planner` | Python | Trajectory generator and mode manager |
| `unav_test` | Python | Simulation and test nodes |

## Dependencies

- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [PX4-Autopilot v1.15.4](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.15.4)
- [px4_msgs release/1.15](https://github.com/PX4/px4_msgs/tree/release/1.15)
- [ACADOS](https://github.com/acados/acados)
- Eigen3

## Getting Started

```bash
git clone https://github.com/purnapatel99/UNAV.git
cd UNAV
./scripts/setup.sh          # installs px4_msgs and rosdep deps
colcon build --symlink-install
source install/setup.bash
```

For PX4 SITL:
```bash
INSTALL_PX4_SITL=1 ./scripts/setup.sh
```
