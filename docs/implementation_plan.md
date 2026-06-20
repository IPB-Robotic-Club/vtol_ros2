# Rework VTOL Core to Basic Node

Simplify `vtol_core.py` from a complex failsafe/watchdog node to a basic, clean ROS2 node that logs status changes and requests the autopilot telemetry stream rate.

## Proposed Changes

### vtol_control Package

#### [MODIFY] [vtol_core.py](../workspace/src/vtol_control/vtol_control/vtol_core.py)

- Change inheritance of `VtolCore` from `VtolBaseNode` to `rclpy.node.Node`.
- Remove the 10Hz failsafe watchdog timer.
- Remove automated failsafe `LAND` commands.
- Remove `StatusText` telemetry subscription and forwarding.
- Remove GCS/Pilot manual override detection flags.
- Keep:
  - Subscription to `/mavros/state`.
  - Logging of connection status, arm status, and flight mode changes.
  - Telemetry stream rate request to autopilot via `/mavros/set_stream_rate`.

#### [MODIFY] [struktur_kode.md](struktur_kode.md)

- Update documentation to reflect the simplified code structure of the new basic `vtol_core.py`.

#### [MODIFY] [failsafe_dan_konfigurasi.md](failsafe_dan_konfigurasi.md)

- Update/simplify the section on `vtol_core.py` failsafes, clarifying that it has been reverted to a basic state monitoring node without active watchdog control.

## Verification Plan

### Automated Tests
- Run `colcon build --packages-select vtol_control` to compile the package.
- Run `ros2 run vtol_control vtol_core` or launch it via `ros2 launch vtol_control vtol_core.launch.py` to verify it runs and initializes successfully.
