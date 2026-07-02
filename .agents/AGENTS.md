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
