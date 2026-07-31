import numpy as np
import cv2
import glob
import os
import yaml

# === KONFIGURASI ===
# Pola papan catur kita memiliki 9x7 kotak, artinya ada 8x6 sudut internal
CHECKERBOARD = (8, 6) 
IMAGE_DIR = "/home/vtol/vtol_ros2/calibration_images"
OUTPUT_FILE = "/home/vtol/vtol_ros2/camera_calibration.yaml"

# Kriteria untuk optimalisasi sudut sub-pixel
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Titik 3D dunia nyata (koordinat ideal)
objpoints = [] 
# Titik 2D gambar nyata
imgpoints = [] 

# Definisikan koordinat ideal sudut papan catur
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Menggunakan ukuran kotak fisik dalam meter (misal kotak 2.5cm = 0.025m)
square_size_meters = 0.025 
objp = objp * square_size_meters

# Cari semua gambar .png di folder kalibrasi
images = glob.glob(os.path.join(IMAGE_DIR, "*.png"))
print(f"Menemukan {len(images)} gambar untuk kalibrasi...")

success_count = 0
gray = None

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Cari sudut-sudut internal papan catur
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, 
                                            cv2.CALIB_CB_ADAPTIVE_THRESH + 
                                            cv2.CALIB_CB_FAST_CHECK + 
                                            cv2.CALIB_CB_NORMALIZE_IMAGE)
    
    if ret == True:
        objpoints.append(objp)
        # Sempurnakan koordinat sudut gambar secara sub-pixel
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        success_count += 1
        print(f"[OK] Sudut terdeteksi pada: {os.path.basename(fname)}")
    else:
        print(f"[LEWAT] Gagal mendeteksi sudut pada: {os.path.basename(fname)}")

if success_count < 10:
    print("\n[ERROR] Kurang dari 10 gambar yang berhasil dideteksi. Pastikan gambar papan catur Anda fokus, tidak melengkung, dan memiliki pencahayaan merata!")
    exit()

print(f"\nMelakukan kalkulasi kalibrasi menggunakan {success_count} gambar...")
# Hitung parameter intrinsik kamera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Hitung Reprojection Error (Tingkat akurasi kalibrasi) menggunakan NumPy (lebih aman)
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    
    # Ratakan bentuk array koordinat menjadi (N, 2)
    p1 = imgpoints[i].reshape(-1, 2)
    p2 = imgpoints2.reshape(-1, 2)
    
    # Hitung rata-rata jarak piksel untuk gambar ini
    error = np.mean(np.linalg.norm(p1 - p2, axis=1))
    mean_error += error

total_error = mean_error / len(objpoints)

print("\n=== HASIL KALIBRASI ===")
print(f"Reprojection Error: {total_error:.4f} pixels (Semakin kecil semakin bagus. Idealnya < 0.5)")
print("\nCamera Matrix (K):")
print(mtx)
print("\nDistortion Coefficients (D):")
print(dist)

# Simpan data kalibrasi ke file YAML
calib_data = {
    "camera_matrix": mtx.tolist(),
    "distortion_coefficients": dist.tolist(),
    "reprojection_error": float(total_error)
}

with open(OUTPUT_FILE, "w") as f:
    yaml.dump(calib_data, f)

print(f"\n[SUKSES] Parameter kalibrasi berhasil disimpan di: {OUTPUT_FILE}")
