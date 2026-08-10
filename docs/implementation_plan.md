# Implementation Plan: Operator Confirmation After Arming in `VtolBaseNode`

Refactoring `VtolBaseNode` in `vtol_base.py` to add an interactive operator confirmation prompt after arming, requiring the operator to press `[ENTER]` before proceeding to the next flight stage (such as climb/takeoff).

## User Review Required

> [!IMPORTANT]
> **Safety & Spin behavior during waiting for input**:
> While waiting for the operator to press `[ENTER]`, `rclpy.spin_once()` will continue running in a non-blocking loop (using `select.select` on `sys.stdin`). This ensures ROS2 node callbacks, MAVROS heartbeats, and RC override timers remain active while waiting. If autopilot disconnects or disarms while waiting, the process will automatically abort.

## Proposed Changes

### `vtol_control`

#### [MODIFY] [vtol_base.py](../workspace/src/vtol_control/vtol_control/vtol_base.py)

- **Add `wait_for_operator_confirmation(self, prompt)`**:
  - Displays standard logger info and formatted terminal prompt.
  - Non-blocking stdin check via `select.select([sys.stdin], [], [], 0.0)`.
  - Runs `rclpy.spin_once(self, timeout_sec=0.05)` per iteration to keep ROS2 background callbacks alive.
  - Performs connection and arming safety checks during wait.
  - Returns `True` when operator presses `[ENTER]` (or when non-interactive EOF is detected).

- **Add `arm(self, timeout=5.0, confirm=True)`**:
  - Encapsulates arming request and waiting for `self.current_state.armed == True`.
  - Calls `wait_for_operator_confirmation` after arming if `confirm=True`.

- **Update `takeoff(self, ...)`**:
  - Replaces manual arming block with `self.arm(confirm=confirm)`.

## Verification Plan

### Automated Build Verification
- Run `colcon build --packages-select vtol_control` inside workspace to confirm compilation succeeds.

### Manual Verification
- Test python import and syntax integrity.
