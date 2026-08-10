# Walkthrough: Camera Axis Inversion Fix & PID Tuning for Real Drone Centering

## Summary of Changes

We identified that on the physical Raspberry Pi 5 drone hardware, the camera stream was inverted by 180° relative to the drone's body axes, causing the drone to move away from the marker when trying to correct roll/pitch. Additionally, PID parameters in `vtol_config.yaml` were overly constrained by `max_override` and high derivative gain.

### 1. Vision Configuration & Inversion Support (`vtol_vision`)
- **[vision_config.yaml](file:///home/qois/vtol-dev/workspace/src/vtol_vision/config/vision_config.yaml)**: Added `flip_camera: true` under the `raspi` hardware profile.
- **[aruco_receiver.py](file:///home/qois/vtol-dev/workspace/src/vtol_vision/vtol_vision/aruco_receiver.py)**:
  - Loaded `flip_camera` configuration option in `__init__`.
  - Applied 180° rotation `cv2.flip(frame, -1)` at the start of `_process_frame` when `flip_camera` is active.

### 2. PID & Control Optimization (`vtol_control`)
- **[vtol_config.yaml](file:///home/qois/vtol-dev/workspace/src/vtol_control/config/vtol_config.yaml)**:
  - `max_override`: Increased from `30` to `50` PWM units (providing sufficient tilt angle for real-world drone flight).
  - `deadzone_bias`: Adjusted to `20.0` (providing a smooth linear control range `pid_max_raw = 50 - 20 = 30.0`).
  - `kp_roll` / `kp_pitch`: Increased to `6.0`.
  - `ki_roll` / `ki_pitch`: Set to `0.1`.
  - `kd_roll` / `kd_pitch`: Reduced from `8.0` to `2.5` to eliminate derivative noise spikes.

---

## Verification Results

### Build Integrity Test
Ran `colcon build` inside the `vtol_dev` Docker container:
```bash
docker exec vtol_dev bash -c "cd /home/pilot/workspace && colcon build --packages-select vtol_vision vtol_control"
```
**Output**:
```text
Starting >>> vtol_control
Finished <<< vtol_control [0.79s]
Starting >>> vtol_vision
Finished <<< vtol_vision [0.69s]

Summary: 2 packages finished [1.61s]
```
✅ Both packages compiled cleanly with 0 errors.

---

## Pre-Flight Checklist for Real Drone Test

1. **Verify Web Stream Dashboard**:
   - Run `pi5_streamer.py` on host and `ros2 run vtol_vision aruco_receiver` in container.
   - Access `http://<raspi-ip>:8086`.
   - Move marker forward $\rightarrow$ Verify marker center moves UP on screen.
   - Move marker right $\rightarrow$ Verify marker center moves RIGHT on screen.

2. **Post-Flight Diagnostics**:
   - Run `python3 workspace/analyze.py --mode all` after flight to verify convergence.
