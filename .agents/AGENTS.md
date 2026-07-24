# Project Customization Rules (`vtol_ros2`)

This document defines behavior guidelines and style rules for any AI agents working on this codebase.

## 1. Artifacts Storage & Accessibility
- **Rule**: All planning and walkthrough artifacts (`implementation_plan.md`, `task.md`, `walkthrough.md`) must be created/copied directly into the workspace's `docs/` directory (e.g., `docs/implementation_plan.md`, `docs/task.md`, `docs/walkthrough.md`) in addition to their default system location.
- **Reason**: The user cannot easily access the default system application data directory (due to WSL or sandbox restrictions). Placing them in `docs/` makes them immediately visible and readable within the workspace environment.

## 2. Documentation and Link Formats
- **Rule**: All links to files inside documentation files must use **relative paths** (e.g., `../workspace/...` or `docs/...`) instead of absolute paths (e.g., `file:///wsl.localhost/...`).
- **Reason**: Absolute UNC paths are non-portable and fail to resolve across different host environments.

## 3. Development & Safety Guidelines
- **Rule**: Always compile the workspace using `colcon build --packages-select vtol_control` inside the running `vtol_dev` container to verify build integrity before declaring tasks complete.
- **Rule**: Low-level heartbeat/RC-loss failsafes (e.g., auto-LAND on GCS heartbeat loss) must be configured directly on the autopilot firmware (ArduPilot/PX4) parameters — NOT from ROS2 nodes.
- **Rule**: Mission-level failsafes triggered by mission logic (e.g., calling `self.land()` or `self.abort_flight()` when a marker is lost for too long) ARE permitted in ROS2 nodes. These are intentional mission abort sequences, not low-level hardware watchdogs.
- **Rule**: When changing flight altitude (takeoff/target altitude) or climb thrust (throttle), agents must configure it inside [vtol_config.yaml](../workspace/src/vtol_control/config/vtol_config.yaml) under the `takeoff` block. Do NOT hardcode these values in Python mission scripts.

## 4. Troubleshooting and Log Analysis
- **Rule**: When troubleshooting issues related to flight logs, PID tuning, RC overrides, or centering behavior, agents MUST use the consolidated analysis script at [workspace/analyze.py](../workspace/analyze.py) to investigate correlation and command mappings.
- **Usage**:
  - To view correlation metrics: `python3 workspace/analyze.py --mode correlation` (reads `centering_data.csv`).
  - To inspect pitch control data (Err_Y vs RC_P): `python3 workspace/analyze.py --mode pitch --start 100 --end 120`.
  - To inspect roll control data (Err_X vs RC_R): `python3 workspace/analyze.py --mode roll --start 100 --end 120`.
  - To analyze vision detection performance: `python3 workspace/analyze.py --mode vision` (reads `aruco_vision.log`).
  - To analyze mission PID and altitude metrics: `python3 workspace/analyze.py --mode mission` (reads `mission_centering.log`).
  - To run all analysis tools at once: `python3 workspace/analyze.py --mode all`.
  - You can specify custom log paths via `--csv <path>`, `--vision-log <path>`, or `--mission-log <path>`.

## 5. Code Architecture Guidelines

### 5.1 Base Class vs Mission Scripts
- **Rule**: Any functionality that is **shared across multiple nodes** (telemetry reading, altitude abstraction, arming, mode change, takeoff, land, abort) MUST live in `VtolBaseNode` (`vtol_base.py`). Mission scripts (`mission_centering.py`, `mission_hover.py`, etc.) must only contain **mission-specific logic** — they inherit shared behavior from the base class.
- **Rule**: When adding new telemetry sources (sensors, topics) to the system, the subscription, state variable, and accessor method must be implemented in `vtol_base.py`. Mission files call the inherited accessor — they do NOT subscribe to topics directly.
- **Reason**: Placing shared behavior in mission files leads to code duplication, inconsistency between missions, and increased maintenance burden. Any bug fix or improvement would need to be replicated across all mission files manually.
- **Example (correct)**:
  ```python
  # vtol_base.py — satu tempat implementasi
  def get_current_altitude(self) -> float:
      if self.use_rangefinder:
          return self.rangefinder_range if self.has_rangefinder else 0.0
      return self.current_pose.pose.position.z if self.has_pose else 0.0

  # mission_centering.py — hanya memanggil, tidak reimplementasi
  alt = self.get_current_altitude()
  ```

### 5.2 Altitude Source
- **Rule**: Always use `self.get_current_altitude()` (method dari `VtolBaseNode`) for reading current altitude — never read `self.current_pose.pose.position.z` directly in mission files. The base class automatically selects the correct source based on `active_profile` in `vtol_config.yaml`.
- **Rule**: Altitude source is switched automatically by `active_profile` in `vtol_config.yaml`:
  - `tcp` (SITL) → `local_position/pose.z` (EKF)
  - `serial` (real drone) → rangefinder topic (AGL)
- **Note (Rangefinder Topic)**: Topic MAVROS untuk rangefinder yang sudah diverifikasi dari hardware adalah `/mavros/rangefinder/rangefinder` (tipe `sensor_msgs/Range`, unit: **meter**). Topic ini sudah di-set di `vtol_base.py`. Jika ganti hardware, verifikasi ulang dengan `ros2 topic list | grep -i range`.

