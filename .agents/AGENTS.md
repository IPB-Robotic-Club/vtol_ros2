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
- **Rule**: Active failsafes and safety watchdog commands (like sending automatic software `LAND` mode changes on loss of heartbeat) must be configured directly on the autopilot firmware (ArduPilot/PX4) parameters, keeping the ROS2 nodes lightweight and purely focused on telemetry monitoring and mission logic.
