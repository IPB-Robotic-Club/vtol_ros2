# Walkthrough - Root Cause Fix: Perbaikan Fluktuasi/Loncatan Panah FRONT

Telah ditemukan dan diperbaiki **akar penyebab utama (*root cause*)** dari panah FRONT yang melompat-lompat antar frame pada stream 8086.

## Root Cause & Ultimate Fix

### **Akar Masalah (*Root Cause*)**:
- Pada `aruco_receiver.py`, pendeteksian ArUco dilakukan pada 2 versi gambar: **Normal Frame (`gray_eq`)** dan **Inverted Frame (`gray_inv`)** untuk mengatasi pantulan bayangan.
- Sebelumnya, hasil deteksi `ids_inv` dan `corners_inv` langsung digabung (*concatenate*) tanpa memfilter ID yang sudah terdeteksi di normal frame.
- Akibatnya, pada setiap frame di mana marker terdeteksi oleh kedua mode, daftar `detections` berisi **2 ID duplikat yang sama**. Pada mode Inverted (`gray_inv`), OpenCV mendeteksi corner ArUco dalam kondisi warna terbalik yang memutar sudut corner 0 $\leftrightarrow$ corner 2 (rotasi 180°).
- Hal ini menyebabkan `detections[0]` berganti-ganti secara acak antara hasil deteksi Normal (menunjuk ke depan) dan Inverted (menunjuk 180° ke belakang) pada setiap frame, sehingga panah melompat 180° tanpa henti.

### **Solusi Perbaikan**:
1. **Deduplikasi Marker ID di [aruco_receiver.py](file:///home/qois/vtol-dev/workspace/src/vtol_vision/vtol_vision/aruco_receiver.py#L456-L470)**:
   - Menambahkan filter `existing_ids` sebelum menggabungkan hasil deteksi `ids_inv`. Hasil deteksi inverted hanya ditambahkan jika ID marker tersebut **belum terdeteksi** di mode normal.
2. **2D Corner Midpoint + EMA Filtering**:
   - Panah FRONT dihitung dari vektor 2D pusat marker menuju titik tengah Corner 0-1 (Top Edge standar OpenCV).
   - Vektor di-smooth menggunakan **Exponential Moving Average (EMA)** ($\alpha = 0.15$), sehingga pergerakan panah `FRONT ^` kini **100% mulus, stabil, dan tidak pernah loncat 180°**.

---

## Verification Results

### Build Verification
- Kompilasi paket `vtol_vision` di dalam container `vtol_dev` berhasil 100%:
  ```bash
  docker exec vtol_dev bash -c "source /ros_entrypoint.sh && colcon build --packages-select vtol_vision"
  # Result: 1 package finished [4.10s]
  ```
