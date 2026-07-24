#!/usr/bin/env python3
import sys
import os
import glob
import cv2

def main():
    print("=== TEST V4L2 CAMERA CAPTURE ===")
    
    # 1. Cari semua device video yang ada di sistem
    video_devices = sorted(glob.glob('/dev/video*'))
    print(f"Device video terdeteksi di /dev: {video_devices if video_devices else 'TIDAK ADA'}")
    
    device_path = sys.argv[1] if len(sys.argv) > 1 else '/dev/video0'
    print(f"Mencoba membuka: {device_path}")
    
    if not os.path.exists(device_path):
        print(f"\n[ERROR] Device '{device_path}' tidak ditemukan!")
        print("Petunjuk untuk Raspberry Pi:")
        print("1. Pastikan kamera CSI / USB sudah terpasang di Raspberry Pi.")
        print("2. Jika di Docker, pastikan container dijalankan dengan flag:")
        print("   docker run --device /dev/video0 ... (atau --privileged)")
        return

    # 2. Coba buka kamera via OpenCV V4L2
    cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[ERROR] cv2.VideoCapture gagal membuka '{device_path}'.")
        return
        
    print("[OK] Device berhasil dibuka oleh OpenCV (CAP_V4L2)!")
    
    # Set resolusi & FPS test
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Resolusi Kamera: {int(w)}x{int(h)} @ {int(fps)} FPS")
    
    # 3. Tangkap 1 frame
    print("Mencoba menangkap frame...")
    ret, frame = cap.read()
    cap.release()
    
    if ret and frame is not None:
        output_file = "test_v4l_frame.jpg"
        cv2.imwrite(output_file, frame)
        print(f"[BERHASIL] Frame berhasil ditangkap dan disimpan ke: {output_file}")
        print(f"Ukuran frame: {frame.shape[1]}x{frame.shape[0]} px, {frame.shape[2]} channels")
    else:
        print("[GAGAL] Tidak dapat membaha frame dari kamera (read() return False).")

if __name__ == '__main__':
    main()
