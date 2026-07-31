"""
mission_debug_aruco.py — ArUco Pre-Flight Debug & Dry-Run Node

Node ini TIDAK menerbangkan drone. Fungsinya:
  - Subscribe ke /vtol/aruco/detection
  - Verifikasi health pipeline vision (FPS, latency, detection rate, pose sanity)
  - Simulasi PID math (dry-run) untuk menampilkan RC value yang AKAN dikirim
  - Print ringkasan gerakan drone yang akan dilakukan berdasarkan data ArUco
  - Tampilkan heading/yaw error dari orientasi marker

Jalankan ini saat drone di darat untuk memastikan seluruh pipeline
ArUco sudah benar sebelum mission centering yang sesungguhnya.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import time
from collections import deque


# ── ANSI Color helpers ──────────────────────────────────────────────────────

class C:
    RESET  = '\033[0m'
    BOLD   = '\033[1m'
    RED    = '\033[91m'
    YELLOW = '\033[93m'
    GREEN  = '\033[92m'
    CYAN   = '\033[96m'
    MAGENTA= '\033[95m'
    WHITE  = '\033[97m'
    DIM    = '\033[2m'

def color_val(val, warn_thresh, err_thresh, invert=False):
    """Warnai nilai: hijau=OK, kuning=warning, merah=error."""
    if invert:
        good = abs(val) > warn_thresh
        bad  = abs(val) > err_thresh
    else:
        good = abs(val) < warn_thresh
        bad  = abs(val) > err_thresh
    if bad:
        return C.RED
    if not good:
        return C.YELLOW
    return C.GREEN

def fmt_error(val, warn=0.15, err=0.35):
    col = color_val(val, warn, err)
    sign = '+' if val >= 0 else ''
    return f"{col}{sign}{val:.3f}{C.RESET}"

def fmt_rc(val):
    dev = val - 1500
    col = C.GREEN if abs(dev) < 30 else (C.YELLOW if abs(dev) < 60 else C.RED)
    return f"{col}{val}{C.RESET}"


# ── Minimal PID (identik dengan mission_centering) ──────────────────────────

def apply_smooth_deadzone(u_raw, deadzone, band=2.0):
    if abs(u_raw) < band:
        return u_raw * ((deadzone + band) / band)
    else:
        return u_raw + math.copysign(deadzone, u_raw)


class PIDSimulator:
    """
    Menjalankan perhitungan PID yang identik dengan mission_centering
    untuk memperkirakan RC value yang akan dikirim, tanpa publish apapun.
    """
    def __init__(self, kp, ki, kd, max_out, d_filter_alpha=0.4):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.d_filter_alpha = d_filter_alpha
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.last_output = 0.0
        self.filtered_derivative = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

    def update(self, error, current_time):
        if self.last_time is None:
            self.last_time = current_time
            self.last_error = error
            return 0.0
        dt = current_time - self.last_time
        if dt <= 0.03:
            return self.last_output
        self.p_term = self.kp * error
        self.integral += error * dt
        max_i = self.max_out / (self.ki if self.ki != 0 else 1.0)
        self.integral = max(min(self.integral, max_i), -max_i)
        self.i_term = self.ki * self.integral
        raw_deriv = (error - self.last_error) / dt
        raw_deriv = max(min(raw_deriv, 2.0), -2.0)
        self.filtered_derivative = (
            self.d_filter_alpha * raw_deriv
            + (1.0 - self.d_filter_alpha) * self.filtered_derivative
        )
        self.d_term = self.kd * self.filtered_derivative
        output = self.p_term + self.i_term + self.d_term
        output = max(min(output, self.max_out), -self.max_out)
        self.last_error = error
        self.last_time = current_time
        self.last_output = output
        return output

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.last_output = 0.0
        self.filtered_derivative = 0.0


# ── Debug Node ───────────────────────────────────────────────────────────────

class ArucoDebugNode(Node):
    def __init__(self):
        super().__init__('aruco_debug_node')

        # Load PID config
        try:
            from vtol_control.config_reader import get_pid_config
            pid = get_pid_config()
        except Exception as e:
            self.get_logger().warn(f"Gagal load PID config, pakai default: {e}")
            pid = {}

        self.max_override      = pid.get('max_override', 40)
        self.max_yaw_override  = pid.get('max_yaw_override', 60)
        self.error_threshold   = pid.get('error_threshold', 0.08)
        self.yaw_error_threshold = pid.get('yaw_error_threshold', 0.1)
        deadzone_bias          = pid.get('deadzone_bias', 12.0)
        pid_max_raw            = self.max_override - deadzone_bias

        # PID simulators (dry-run, tidak publish)
        self.pid_roll  = PIDSimulator(
            pid.get('kp_roll', 15.0), pid.get('ki_roll', 0.5),
            pid.get('kd_roll', 5.0), pid_max_raw
        )
        self.pid_pitch = PIDSimulator(
            pid.get('kp_pitch', 15.0), pid.get('ki_pitch', 0.5),
            pid.get('kd_pitch', 5.0), pid_max_raw
        )
        self.pid_yaw   = PIDSimulator(
            pid.get('kp_yaw', 8.0), pid.get('ki_yaw', 0.1),
            pid.get('kd_yaw', 4.0), self.max_yaw_override - deadzone_bias
        )
        self.deadzone_bias = deadzone_bias

        # Health tracking state
        self.frame_window          = deque(maxlen=20)   # True/False per frame deteksi
        self.frame_timestamps      = deque(maxlen=30)   # Waktu frame untuk hitung FPS
        self.last_frame_no         = -1
        self.node_start_time       = time.time()
        self.last_msg_time         = None
        self.no_data_warned        = False

        # Subscribe
        self.sub = self.create_subscription(
            String,
            '/vtol/aruco/detection',
            self._detection_callback,
            10
        )

        # Watchdog timer: periksa apakah topic sudah ada data
        self.watchdog_timer = self.create_timer(3.0, self._watchdog_check)

        print(f"\n{C.BOLD}{C.CYAN}{'═'*58}{C.RESET}")
        print(f"{C.BOLD}{C.CYAN}   ARUCO DEBUG DRY-RUN — PRE-FLIGHT HEALTH CHECK{C.RESET}")
        print(f"{C.BOLD}{C.CYAN}{'═'*58}{C.RESET}")
        print(f"{C.DIM}  Subscribe: /vtol/aruco/detection{C.RESET}")
        print(f"{C.DIM}  max_override={self.max_override} | yaw_override={self.max_yaw_override}{C.RESET}")
        print(f"{C.DIM}  error_threshold={self.error_threshold} | deadzone={self.deadzone_bias}{C.RESET}")
        print(f"{C.YELLOW}  ⚠  Mode DRY-RUN: Drone TIDAK akan terbang.{C.RESET}")
        print(f"{C.BOLD}{C.CYAN}{'─'*58}{C.RESET}\n")

    # ── Watchdog ────────────────────────────────────────────────────────────

    def _watchdog_check(self):
        """Warn jika tidak ada data dari vision setelah 3 detik."""
        if self.last_msg_time is None and not self.no_data_warned:
            elapsed = time.time() - self.node_start_time
            if elapsed >= 3.0:
                print(
                    f"\n{C.RED}{C.BOLD}[WARN] Tidak ada data dari /vtol/aruco/detection "
                    f"selama {elapsed:.0f}s!{C.RESET}"
                )
                print(f"{C.YELLOW}       Pastikan node aruco_receiver sudah berjalan:{C.RESET}")
                print(f"{C.DIM}       ros2 run vtol_vision aruco_receiver{C.RESET}\n")
                self.no_data_warned = True

    # ── Detection Callback ───────────────────────────────────────────────────

    def _detection_callback(self, msg):
        now = time.time()
        self.last_msg_time = now

        try:
            data = json.loads(msg.data)
        except Exception as e:
            print(f"{C.RED}[ERROR] Gagal parse JSON: {e}{C.RESET}")
            return

        frame_no  = data.get('count', 0)
        vision_ts = data.get('timestamp', 0.0)
        detected  = data.get('detected', False)
        markers   = data.get('markers', [])
        frame_w   = float(data.get('frame_w', 640))
        frame_h   = float(data.get('frame_h', 480))

        # Skip frame duplikat
        if frame_no == self.last_frame_no:
            return
        self.last_frame_no = frame_no

        # ── FPS & Latency ──
        self.frame_timestamps.append(now)
        self.frame_window.append(detected)

        fps = 0.0
        if len(self.frame_timestamps) >= 2:
            span = self.frame_timestamps[-1] - self.frame_timestamps[0]
            if span > 0:
                fps = (len(self.frame_timestamps) - 1) / span

        latency_ms = (now - vision_ts) * 1000.0 if vision_ts > 0 else -1.0

        det_count  = sum(self.frame_window)
        total_count = len(self.frame_window)
        det_rate   = det_count / total_count * 100.0 if total_count > 0 else 0.0

        # ── Build output ──
        lines = []

        # Header baris
        fps_col = C.GREEN if fps >= 15 else (C.YELLOW if fps >= 8 else C.RED)
        lag_col = C.GREEN if 0 <= latency_ms < 150 else (C.YELLOW if latency_ms < 300 else C.RED)
        rate_col = C.GREEN if det_rate >= 85 else (C.YELLOW if det_rate >= 60 else C.RED)

        fps_str = f"{fps_col}{fps:.1f} fps{C.RESET}"
        lag_str = (f"{lag_col}{latency_ms:.0f}ms ✓{C.RESET}" if latency_ms < 150
                   else f"{lag_col}{latency_ms:.0f}ms ⚠{C.RESET}")
        rate_str = f"{rate_col}{det_rate:.0f}% ({det_count}/{total_count}){C.RESET}"

        lines.append(f"\n{C.BOLD}Frame #{frame_no}{C.RESET}")
        lines.append(f"  [FPS  ]  {fps_str:<30}  [LAG  ]  {lag_str}")
        lines.append(f"  [STATS]  Rate: {rate_str}")

        if not detected or len(markers) == 0:
            lines.append(f"  [MARKER] {C.RED}{C.BOLD}NO MARKER DETECTED{C.RESET}")
            lines.append(f"{'─'*58}")
            print('\n'.join(lines))
            return

        # ── Multi-marker warning ──
        if len(markers) > 1:
            ids_str = ', '.join(str(m['id']) for m in markers)
            lines.append(
                f"  [WARN ]  {C.YELLOW}{C.BOLD}{len(markers)} marker terdeteksi! "
                f"(ID: {ids_str}) — hanya ID {markers[0]['id']} digunakan{C.RESET}"
            )

        marker = markers[0]
        marker_id = marker['id']
        cx, cy = marker['center']

        # ── Pose tvec ──
        has_tvec = 'pose_tvec' in marker
        if has_tvec:
            tx, ty, tz = marker['pose_tvec']
        else:
            # Fallback ke pixel normalized
            tx = (cx - frame_w / 2.0) / (frame_w / 2.0)
            ty = (cy - frame_h / 2.0) / (frame_h / 2.0)
            tz = 0.0

        # Sanity check pose
        pose_valid = True
        pose_issues = []
        if has_tvec:
            if tz <= 0:
                pose_valid = False
                pose_issues.append(f"Z={tz:.3f}m NEGATIF (pose flip!)")
            elif tz > 10.0:
                pose_valid = False
                pose_issues.append(f"Z={tz:.3f}m terlalu jauh")
            if abs(tx) > tz * 1.5 or abs(ty) > tz * 1.5:
                pose_issues.append("lateral error > distance (kemungkinan noise)")

        pose_col = C.GREEN if pose_valid and not pose_issues else (C.YELLOW if pose_valid else C.RED)
        pose_status = "✓ VALID" if pose_valid and not pose_issues else ("⚠ PERIKSA" if pose_valid else "✗ INVALID")
        tz_str = f"{tz:+.3f}m" if has_tvec else "N/A"
        lines.append(
            f"  [POSE ]  ID={marker_id} | "
            f"tvec X={tx:+.3f} Y={ty:+.3f} Z={tz_str}  "
            f"{pose_col}{pose_status}{C.RESET}"
        )
        for issue in pose_issues:
            lines.append(f"           {C.YELLOW}⚠ {issue}{C.RESET}")

        # ── Yaw dari rvec ──
        yaw_error = 0.0
        has_rvec = 'pose_rvec' in marker
        if has_rvec:
            rvec = marker['pose_rvec']
            # Yaw relatif marker terhadap kamera: rvec[2] (rotasi sumbu Z)
            yaw_error = float(rvec[2])
            # Wrap ke [-pi, pi]
            yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
            yaw_deg = math.degrees(yaw_error)
            yaw_col = C.GREEN if abs(yaw_error) < self.yaw_error_threshold else (C.YELLOW if abs(yaw_error) < 0.5 else C.RED)
            yaw_sign = '+' if yaw_error >= 0 else ''
            lines.append(
                f"  [YAW  ]  rvec[2]: {yaw_col}{yaw_sign}{yaw_error:.3f} rad "
                f"({yaw_sign}{yaw_deg:.1f}°){C.RESET}"
            )

        # ── Distance check ──
        dist = math.sqrt(tx**2 + ty**2)
        in_zone = dist <= self.error_threshold
        zone_col = C.GREEN if in_zone else (C.YELLOW if dist < self.error_threshold * 2 else C.RED)
        zone_status = "✓ DALAM TOLERANSI" if in_zone else "BELUM STABIL"
        lines.append(
            f"  [ZONE ]  Dist: {zone_col}{dist:.3f}m{C.RESET} "
            f"| threshold: {self.error_threshold}m → "
            f"{zone_col}{zone_status}{C.RESET}"
        )

        # ── PID Simulation ──
        current_time = time.time()
        u_roll_raw  = self.pid_roll.update(tx, current_time)
        u_pitch_raw = self.pid_pitch.update(ty, current_time)
        u_yaw_raw   = self.pid_yaw.update(yaw_error, current_time) if has_rvec else 0.0

        u_roll  = apply_smooth_deadzone(u_roll_raw, self.deadzone_bias, band=0.5)
        u_pitch = apply_smooth_deadzone(u_pitch_raw, self.deadzone_bias, band=0.5)
        u_yaw   = apply_smooth_deadzone(u_yaw_raw, self.deadzone_bias, band=0.5)

        u_roll  = max(min(u_roll,  self.max_override), -self.max_override)
        u_pitch = max(min(u_pitch, self.max_override), -self.max_override)
        u_yaw   = max(min(u_yaw,   self.max_yaw_override), -self.max_yaw_override)

        rc1 = int(1500 + u_roll)
        rc2 = int(1500 + u_pitch)
        rc4 = int(1500 + u_yaw) if has_rvec else 1500

        roll_dir  = "ROLL KANAN" if u_roll > 0 else ("ROLL KIRI" if u_roll < 0 else "NETRAL")
        pitch_dir = "MAJU"       if u_pitch < 0 else ("MUNDUR"    if u_pitch > 0 else "NETRAL")
        yaw_dir   = "YAW KANAN"  if u_yaw > 0 else ("YAW KIRI"   if u_yaw < 0 else "NETRAL")

        lines.append(
            f"  [PID  ]  Roll : err={fmt_error(tx)} | "
            f"P={self.pid_roll.p_term:+.2f} I={self.pid_roll.i_term:+.2f} D={self.pid_roll.d_term:+.2f} "
            f"→ {fmt_rc(rc1)} {C.DIM}({roll_dir}){C.RESET}"
        )
        lines.append(
            f"  [PID  ]  Pitch: err={fmt_error(ty)} | "
            f"P={self.pid_pitch.p_term:+.2f} I={self.pid_pitch.i_term:+.2f} D={self.pid_pitch.d_term:+.2f} "
            f"→ {fmt_rc(rc2)} {C.DIM}({pitch_dir}){C.RESET}"
        )
        if has_rvec:
            lines.append(
                f"  [PID  ]  Yaw  : err={fmt_error(yaw_error, 0.1, 0.5)} | "
                f"P={self.pid_yaw.p_term:+.2f} I={self.pid_yaw.i_term:+.2f} D={self.pid_yaw.d_term:+.2f} "
                f"→ {fmt_rc(rc4)} {C.DIM}({yaw_dir}){C.RESET}"
            )

        # ── ACTION SUMMARY LINE ──
        actions = []
        if abs(u_roll) > 1.0:
            actions.append(f"{'GESER KANAN' if u_roll > 0 else 'GESER KIRI'}")
        if abs(u_pitch) > 1.0:
            actions.append(f"{'MAJU' if u_pitch < 0 else 'MUNDUR'}")
        if has_rvec and abs(u_yaw) > 1.0:
            actions.append(f"{'PUTAR KANAN' if u_yaw > 0 else 'PUTAR KIRI'}")

        if in_zone and (not has_rvec or abs(yaw_error) < self.yaw_error_threshold):
            action_str = f"{C.GREEN}{C.BOLD}✓ SUDAH TERPUSAT — DRONE DIAM (HOVER){C.RESET}"
        elif not actions:
            action_str = f"{C.GREEN}STABIL — koreksi minimal{C.RESET}"
        else:
            action_str = f"{C.BOLD}{C.MAGENTA}" + " + ".join(actions) + C.RESET

        lines.append(f"  {'─'*54}")
        lines.append(f"  {C.BOLD}[ACTION] → Drone akan: {action_str}")
        lines.append(f"{'═'*58}")

        print('\n'.join(lines))


# ── main ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{C.DIM}[INFO] Debug session selesai.{C.RESET}\n")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
