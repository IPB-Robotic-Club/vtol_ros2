#!/usr/bin/env python3
"""
check_camera.py — Diagnostik CSI Camera dari Ubuntu Docker Container
=====================================================================
Karena rpicam TIDAK tersedia di Ubuntu/container, kamera CSI diakses
melalui V4L2 driver yang di-load oleh kernel host (Raspberry Pi OS).

Cara pakai:
    python3 workspace/check_camera.py             # scan semua /dev/video*
    python3 workspace/check_camera.py /dev/video0  # cek device tertentu

Log disimpan ke: workspace/camera_check.log
"""

import sys
import os
import glob
import subprocess
import time
import datetime

# Coba import OpenCV (opsional, tapi dibutuhkan untuk capture frame)
try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_check.log")

# ANSI colors untuk terminal
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

lines_buf = []


def log(msg: str, color: str = ""):
    """Print ke terminal + simpan ke buffer log."""
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    plain = f"[{ts}] {msg}"
    lines_buf.append(plain)
    if color:
        print(f"{color}{plain}{RESET}")
    else:
        print(plain)


def run_cmd(cmd: str) -> str:
    """Jalankan shell command, return output (stdout+stderr)."""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=5
        )
        return (result.stdout + result.stderr).strip()
    except subprocess.TimeoutExpired:
        return "(timeout)"
    except Exception as e:
        return f"(error: {e})"


def save_log():
    with open(LOG_PATH, "w") as f:
        f.write("\n".join(lines_buf) + "\n")
    print(f"\n{CYAN}Log disimpan ke: {LOG_PATH}{RESET}")


# ─────────────────────────────────────────────────────────────────────────────
# BAGIAN 1: Cek environment container
# ─────────────────────────────────────────────────────────────────────────────
def check_environment():
    log("=" * 60)
    log("BAGIAN 1: Cek Environment Container")
    log("=" * 60)

    # Cek apakah berjalan di dalam Docker
    in_docker = os.path.exists("/.dockerenv")
    log(f"Berjalan di Docker: {'YA' if in_docker else 'TIDAK (host langsung)'}")

    # Kernel & distro
    kernel = run_cmd("uname -r")
    log(f"Kernel host     : {kernel}")

    distro = run_cmd("cat /etc/os-release | grep PRETTY_NAME | cut -d= -f2 | tr -d '\"'")
    log(f"Distro container: {distro}")

    # Cek OpenCV
    if HAVE_CV2:
        log(f"OpenCV          : {cv2.__version__}  ✓")
    else:
        log("OpenCV          : TIDAK TERINSTALL — pip3 install opencv-python")

    # Cek v4l-utils (opsional, untuk info detail)
    v4l2_ctl = run_cmd("which v4l2-ctl")
    if v4l2_ctl:
        log(f"v4l2-ctl        : {v4l2_ctl}  ✓")
    else:
        log("v4l2-ctl        : tidak ada (install: sudo apt install v4l-utils)")


# ─────────────────────────────────────────────────────────────────────────────
# BAGIAN 2: Scan semua device video
# ─────────────────────────────────────────────────────────────────────────────
def scan_video_devices() -> list:
    log("\n" + "=" * 60)
    log("BAGIAN 2: Scan /dev/video* Devices")
    log("=" * 60)

    devices = sorted(glob.glob("/dev/video*"))

    if not devices:
        log("[GAGAL] Tidak ada /dev/video* yang ditemukan!", RED)
        log("Kemungkinan penyebab:")
        log("  1. Kamera CSI belum terhubung ke Raspberry Pi")
        log("  2. V4L2 driver belum di-load di host:")
        log("     → Jalankan di HOST (bukan container): sudo modprobe bcm2835-v4l2")
        log("     → Atau: sudo modprobe v4l2-compat-ioctl32")
        log("  3. Docker tidak pass /dev — cek docker-compose.yml sudah ada:")
        log("     volumes: - /dev:/dev")
        log("     privileged: true")
        return []

    log(f"Device video yang ditemukan: {devices}")

    for dev in devices:
        # Cek permission
        readable = os.access(dev, os.R_OK)
        log(f"  {dev}  — readable: {'✓' if readable else '✗ (cek --privileged)'}")

        # Info dari v4l2-ctl jika tersedia
        info = run_cmd(f"v4l2-ctl --device={dev} --info 2>/dev/null | grep -E 'Driver|Card|Bus'")
        if info:
            for line in info.splitlines():
                log(f"    {line.strip()}")

    return devices


# ─────────────────────────────────────────────────────────────────────────────
# BAGIAN 3: Cek format yang didukung device
# ─────────────────────────────────────────────────────────────────────────────
def check_device_formats(device: str):
    log(f"\n{'=' * 60}")
    log(f"BAGIAN 3: Format yang Didukung — {device}")
    log("=" * 60)

    formats = run_cmd(f"v4l2-ctl --device={device} --list-formats-ext 2>/dev/null")
    if formats:
        log(f"Format output dari v4l2-ctl:\n{formats}")
    else:
        log("v4l2-ctl tidak tersedia atau tidak bisa membaca format.")
        log("Lanjut ke tes capture OpenCV langsung...")


# ─────────────────────────────────────────────────────────────────────────────
# BAGIAN 4: Capture frame via OpenCV V4L2
# ─────────────────────────────────────────────────────────────────────────────
def test_capture(device: str, num_frames: int = 5) -> bool:
    log(f"\n{'=' * 60}")
    log(f"BAGIAN 4: Tes Capture Frame via OpenCV — {device}")
    log("=" * 60)

    if not HAVE_CV2:
        log("[SKIP] OpenCV tidak terinstall, tidak bisa capture.", YELLOW)
        return False

    log(f"Membuka {device} dengan CAP_V4L2...")
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

    if not cap.isOpened():
        log(f"[GAGAL] cv2.VideoCapture tidak bisa membuka {device}", RED)
        log("Kemungkinan:")
        log("  → Device sedang dipakai proses lain (cek: fuser /dev/video*)")
        log("  → Driver CSI belum aktif (bcm2835-v4l2 belum di-modprobe)")
        log("  → Format tidak kompatibel (perlu YUYV atau MJPEG)")
        return False

    # Baca properti
    w   = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h   = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    log(f"[OK] Device terbuka: {int(w)}x{int(h)} @ {fps:.1f} FPS", GREEN)

    # Warmup — beberapa kamera CSI butuh beberapa frame sebelum stabil
    log("Warmup (5 frame pertama dibuang)...")
    for _ in range(5):
        cap.read()

    # Tes capture
    success_count = 0
    for i in range(num_frames):
        t0 = time.time()
        ret, frame = cap.read()
        elapsed = (time.time() - t0) * 1000

        if ret and frame is not None:
            h_actual, w_actual = frame.shape[:2]
            mean_brightness = frame.mean()
            log(
                f"  Frame {i+1}/{num_frames}: OK — {w_actual}x{h_actual}px, "
                f"brightness={mean_brightness:.1f}, latency={elapsed:.1f}ms",
                GREEN,
            )

            # Peringatan frame hitam (kamera ada tapi tidak kirim data valid)
            if mean_brightness < 5.0:
                log(
                    f"  [PERINGATAN] Frame hampir hitam! "
                    f"Kamera mungkin belum siap atau exposure sangat gelap.",
                    YELLOW,
                )
            success_count += 1
        else:
            log(f"  Frame {i+1}/{num_frames}: GAGAL read()", RED)

    cap.release()

    # Simpan 1 frame sebagai bukti
    if success_count > 0:
        log(f"\nMenyimpan frame terakhir sebagai 'camera_check_snapshot.jpg'...")
        cap2 = cv2.VideoCapture(device, cv2.CAP_V4L2)
        for _ in range(3):
            cap2.read()
        ret, frame = cap2.read()
        cap2.release()
        if ret and frame is not None:
            snap_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "camera_check_snapshot.jpg"
            )
            cv2.imwrite(snap_path, frame)
            log(f"[OK] Snapshot disimpan: {snap_path}", GREEN)

    log(f"\nHasil: {success_count}/{num_frames} frame berhasil ditangkap")
    return success_count == num_frames


# ─────────────────────────────────────────────────────────────────────────────
# BAGIAN 5: Diagnosa jika semua gagal
# ─────────────────────────────────────────────────────────────────────────────
def diagnose_csi_on_ubuntu():
    log(f"\n{'=' * 60}")
    log("BAGIAN 5: Panduan Aktifkan CSI Camera di Ubuntu Container")
    log("=" * 60)
    log("")
    log("Karena menggunakan Ubuntu (bukan Raspberry Pi OS), rpicam")
    log("TIDAK tersedia. Gunakan V4L2 driver yang di-load di HOST.")
    log("")
    log("Langkah fix di HOST Raspberry Pi (di luar container):")
    log("")
    log("  Option A — bcm2835-v4l2 (RPi OS Bullseye atau lebih lama):")
    log("    sudo modprobe bcm2835-v4l2")
    log("    → Device akan muncul di /dev/video0")
    log("")
    log("  Option B — libcamera + v4l2-compat (RPi OS Bookworm / Ubuntu 22+):")
    log("    sudo apt install libcamera-apps libcamera-v4l2")
    log("    → Lalu: libcamera-vid --list-cameras")
    log("    → Device muncul di /dev/video0, /dev/video1, dll.")
    log("")
    log("  Option C — rpicam-apps dengan v4l2loopback (advanced):")
    log("    sudo apt install v4l2loopback-dkms")
    log("    sudo modprobe v4l2loopback")
    log("    rpicam-vid -t 0 --codec yuv420 -o - | ffmpeg -i - ...")
    log("")
    log("Setelah driver aktif, pastikan di docker-compose.yml:")
    log("    volumes:")
    log("      - /dev:/dev")
    log("    privileged: true")
    log("")
    log("Verifikasi dari HOST:")
    log("    ls -la /dev/video*")
    log("    v4l2-ctl --list-devices")
    log("")
    log("Verifikasi dari dalam container:")
    log("    python3 workspace/check_camera.py")


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main():
    log(f"{BOLD}{'=' * 60}")
    log("CSI CAMERA DIAGNOSTIC — Ubuntu Docker Container")
    log(f"Waktu: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    log("=" * 60)

    check_environment()
    devices = scan_video_devices()

    if not devices:
        diagnose_csi_on_ubuntu()
        save_log()
        sys.exit(1)

    # Pilih target device
    if len(sys.argv) > 1:
        target = sys.argv[1]
        if not os.path.exists(target):
            log(f"[ERROR] Device '{target}' tidak ditemukan di sistem.", RED)
            save_log()
            sys.exit(1)
    else:
        target = devices[0]
        log(f"\nTidak ada device yang dispesifikasi, pakai: {target}")

    check_device_formats(target)
    success = test_capture(target)

    log(f"\n{'=' * 60}")
    if success:
        log(f"[HASIL] KAMERA OK — {target} berfungsi normal!", GREEN)
    else:
        log(f"[HASIL] KAMERA GAGAL — {target} tidak bisa digunakan.", RED)
        diagnose_csi_on_ubuntu()

    save_log()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
