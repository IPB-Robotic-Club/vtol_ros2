# Walkthrough - Perbaikan Mission Centering & Uji Darat Vision

Telah diselesaikan analisis log penerbangan Raspberry Pi, perbaikan masalah Yaw Flipping, optimasi deteksi ArUco Vision, serta penyusunan panduan pengujian darat dan transfer log.

---

## Ringkasan Perubahan Kode & Konfigurasi

### 1. Penanganan Masalah Yaw (Yaw Alignment Disabled)
- **[vtol_config.yaml](../workspace/src/vtol_control/config/vtol_config.yaml)**:
  - Menambahkan `enable_yaw_alignment: false` pada blok `pid_centering`.
  - Menyesuaikan `marker_lost_timeout: 5.0` (dinaikkan dari 3.0s ke 5.0s).
- **[config_reader.py](../workspace/src/vtol_control/vtol_control/config_reader.py)**:
  - Menambahkan `enable_yaw_alignment` ke dictionary `default_pid`.
- **[mission_centering.py](../workspace/src/vtol_control/vtol_control/mission_centering.py)**:
  - Node langsung memulai fase `CENTERING` (fokus PID Roll & Pitch) tanpa terjebak *loop* `YAW_ALIGN`.
  - Proteksi `YAW_DRIFT` dan kalkulasi PID Yaw di-bypass sehingga channel RC4 (Yaw) tetap netral 1500.

### 2. Optimasi Pemrosesan ArUco Vision (Mengatasi 77% Frame Loss)
- **[aruco_receiver.py](../workspace/src/vtol_vision/vtol_vision/aruco_receiver.py)**:
  - Mengubah `adaptiveThreshWinSizeMax` dari 45 ke 23 dan `adaptiveThreshWinSizeStep` dari 4 ke 10.
  - Memangkas *rejected candidates* hingga ~75% dan membebaskan beban CPU Raspberry Pi 5 dari pembuatan 11+ threshold image per frame.

### 3. Dokumentasi Baru
- **[docs/panduan_analisis_log.md](panduan_analisis_log.md)**:
  - Menambahkan panduan lengkap transfer log (SCP/rsync) dari Raspi ke PC.
  - Menambahkan *cheat sheet* penggunaan `workspace/analyze.py`.
  - Menambahkan prosedur pengujian darat (*dry run / bench test*) 4 metode tanpa harus terbang.
- **[README.md](../README.md)**:
  - Menambahkan tautan panduan analisis log ke daftar isi utama.

---

## Hasil Pengujian & Verifikasi

### 1. Kompilasi Paket Docker (`colcon build`)
Perintah kompilasi dijalankan di kontainer `vtol_dev`:
```bash
docker exec vtol_dev bash -c "cd /home/pilot/workspace && colcon build --packages-select vtol_control vtol_vision"
```
**Hasil:** `Summary: 2 packages finished [5.51s]` (Berhasil 100% tanpa error).

### 2. Evaluasi Data Dry Run Terbaru (`analyze.py`)
Hasil uji gerak kamera/marker darat (*shake test*):
- **Tingkat Deteksi Marker:** Meloncat naik dari **22.31% menjadi 71.58%** (peningkatan > 3.2x lipat).
- **Frame Loss:** Turun dari **77.69% menjadi 28.42%**.
- **Streak Hilang Terlama:** Turun dari 1.352 frame ($\sim 45$s) menjadi **79 frame ($\sim 2.6$s)**.
- **Status Failsafe:** Durasi hilang 2.6s berada jauh di bawah batas timeout **5.0s**, sehingga dipastikan aman dari pemicu LAND Failsafe prematur.
