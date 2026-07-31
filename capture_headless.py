import urllib.request
import numpy as np
import cv2
import threading
import os

# === KONFIGURASI ===
STREAM_URL = "http://localhost:8086/stream"
SAVE_DIR = "calibration_images"

# Buat folder penyimpanan jika belum ada
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

latest_frame = None
frame_lock = threading.Lock()
running = True

def stream_reader():
    global latest_frame, running
    try:
        stream = urllib.request.urlopen(STREAM_URL)
        bytes_data = bytes()
        while running:
            bytes_data += stream.read(1024)
            a = bytes_data.find(b'\xff\xd8') # Awal JPEG
            b = bytes_data.find(b'\xff\xd9') # Akhir JPEG
            if a != -1 and b != -1:
                jpg_data = bytes_data[a:b+2]
                bytes_data = bytes_data[b+2:]
                frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    with frame_lock:
                        latest_frame = frame
    except Exception as e:
        print(f"\n[ERROR] Koneksi stream terputus: {e}")
        running = False

# Mulai thread untuk terus membaca stream di latar belakang
thread = threading.Thread(target=stream_reader, daemon=True)
thread.start()

print("=========================================================")
print("   HEADLESS CAPTURE SYSTEM (Cocok untuk Code-Server)")
print("=========================================================")
print("Petunjuk:")
print("  - Tekan [ENTER] di terminal untuk menangkap gambar (capture)")
print("  - Ketik 'q' lalu tekan [ENTER] untuk keluar")
print("=========================================================")

img_counter = 1
while running:
    user_input = input("Tekan ENTER untuk capture: ").strip().lower()
    
    if user_input == 'q':
        running = False
        break
        
    with frame_lock:
        if latest_frame is not None:
            filename = os.path.join(SAVE_DIR, f"calib_{img_counter:02d}.png")
            cv2.imwrite(filename, latest_frame)
            print(f"[SUKSES] Gambar disimpan -> {filename}")
            img_counter += 1
        else:
            print("[PERINGATAN] Belum ada frame diterima. Pastikan streamer & Docker menyala!")
