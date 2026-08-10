# Implementation Plan: Vision Axis Correction & PID Tuning for Real Drone Centering

## Diagnostics & Root Cause Analysis Summary

Based on analyzing the log files (`centering_data.csv`, `mission_centering.log`, and `aruco_vision.log`):

1. **180° Camera Frame Inversion (Primary Root Cause)**:
   - **Log Data Evidence**:
     - `norm_ex` (Roll Error) was negative **96.2%** of the entire flight (averaging -0.20 to -0.45). PID kept commanding `RC1 = 1470-1474` (Roll Left), but the error remained negative, indicating the drone moved away from the target instead of toward it.
     - `norm_ey` (Pitch Error) was negative **73%** of the time. PID commanded Pitch Down (`RC2 < 1500`), but error did not converge.
     - Roll correlation was `0.207` and Pitch correlation was `0.225` (no convergence).
   - **Physical Mechanism**:
     - On real Raspberry Pi 5 hardware, the camera module is mounted 180° rotated (or `Picamera2` streams un-flipped frames).
     - When marker is physically in FRONT of the drone, it appears at the BOTTOM of the image frame (`center_y > 240` $\rightarrow$ `norm_ey > 0`). PID incorrectly assumes marker is BEHIND and commands Pitch UP (`RC2 > 1500`), driving the drone backward.
     - When marker is physically to the RIGHT of the drone, it appears on the LEFT of the image frame (`center_x < 320` $\rightarrow$ `norm_ex < 0`). PID incorrectly commands Roll LEFT (`RC1 < 1500`), driving the drone left.
   - This matches 100% with the observed behavior: *"kebalik / gmn gitu sehingga gk bisa maju kedepan dan roll ke kanan"*.

2. **PID Output Saturation & Bang-Bang Behavior**:
   - `max_override: 30`, `deadzone_bias: 25.0` $\rightarrow$ `pid_max_raw = 5.0`.
   - With `kp = 5.0`, an error of just 0.05-0.10 immediately hits maximum raw output (`5.0`), causing `apply_smooth_deadzone` to jump straight to the maximum limit of ±30 PWM units (`1470` or `1530`).
   - Control operates like an ON/OFF switch rather than a linear PID controller.

3. **High Derivative Gain Noise (`kd = 8.0`)**:
   - `kd = 8.0` is higher than `kp = 5.0`. In a vision loop running at ~20-30Hz, quantization and timing jitter cause large derivative spikes that fight proportional control.

---

## User Review Required

> [!IMPORTANT]
> **Physical Camera Orientation Verification**:
> Before flying, verify camera orientation on the web dashboard (`http://<raspi-ip>:8086`).
> When you move a marker to the FRONT of the drone (towards the nose), the marker center point on screen MUST move UP towards the top of the screen (`Y` decreases).
> When you move a marker to the RIGHT of the drone, the marker center point on screen MUST move RIGHT (`X` increases).

---

## Proposed Changes

### Vision Component (`vtol_vision`)

#### [MODIFY] [vision_config.yaml](file:///home/qois/vtol-dev/workspace/src/vtol_vision/config/vision_config.yaml)
- Add `flip_camera: true` option under the `raspi` profile block.

#### [MODIFY] [aruco_receiver.py](file:///home/qois/vtol-dev/workspace/src/vtol_vision/vtol_vision/aruco_receiver.py)
- Load `flip_camera` parameter from configuration.
- Apply `cv2.flip(frame, -1)` (180° rotation) inside `_process_frame` when `flip_camera` is `true`.

---

### Control Component (`vtol_control`)

#### [MODIFY] [vtol_config.yaml](file:///home/qois/vtol-dev/workspace/src/vtol_control/config/vtol_config.yaml)
- Increase `max_override` from `30` to `50` or `60` PWM units (gives drone adequate tilt angle against real-world wind/drag).
- Adjust `deadzone_bias` to `20.0` (provides a wider linear control region: `pid_max_raw = 50 - 20 = 30.0`).
- Adjust PID gains for hardware flight:
  - `kp_roll: 6.0`, `ki_roll: 0.1`, `kd_roll: 2.5`
  - `kp_pitch: 6.0`, `ki_pitch: 0.1`, `kd_pitch: 2.5`

---

## Verification Plan

### Automated Verification / Build Test
1. Build workspace inside environment:
   ```bash
   colcon build --packages-select vtol_vision vtol_control
   ```

### Manual Verification Steps
1. **Web Dashboard Inspection**:
   - Run `pi5_streamer.py` and `ros2 run vtol_vision aruco_receiver`.
   - Open `http://<raspi_ip>:8086` and verify frame orientation when moving ArUco marker by hand.
2. **Post-Flight Log Analysis**:
   - Run `python3 workspace/analyze.py --mode all` after test flight to verify `Roll correlation` and `Pitch correlation` approach ~0.0 or negative error reduction.
