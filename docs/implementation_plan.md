# Implementation Plan - Penyederhanaan HUD 8086 (Panah FRONT & Toleransi 50%)

Menyederhanakan tampilan HUD overlay pada video stream MJPEG (port 8086) di `aruco_receiver.py`, menambahkan indikator visual **Panah Arah FRONT Marker** yang jelas, serta memperketat batas toleransi error (dist & yaw) sebesar 50%.

## User Review Required

> [!IMPORTANT]
> **Penurunan Nilai Toleransi (Diperketat 50%)**:
> 1. `error_threshold` (translasi X/Y): dari **`0.05m` (5 cm)** $\rightarrow$ **`0.025m` (2.5 cm)**.
> 2. `yaw_error_threshold` (rotasi Yaw): dari **`0.15 rad` (~8.6°)** $\rightarrow$ **`0.075 rad` (~4.3°)**.

> [!NOTE]
> **Visualisasi Baru pada Stream Port 8086**:
> 1. **Panah Arah FRONT Marker (`FRONT ▲`)**: Panah 3D menonjol dari pusat ArUco marker yang mengindikasikan arah depan marker yang harus dicapai oleh heading drone.
> 2. **Single Top HUD Banner**: Panel horizontal ringkas di atas frame: `MARKER #<ID> | DIST: <X.XX>m | YAW: <X.X>° | <STATUS>`.
> 3. **Center Crosshair & Target Dot**: Crosshair tipis pusat kamera (`+`) dan 1 titik target dengan garis penghubung (Hijau = IN ZONE $\le 2.5\text{ cm}$, Oranye/Merah = OUT OF ZONE).
> 4. **Elemen yang Dihapus**: Gauge bar Yaw horizontal di bawah, legend box, panel teks multi-baris, dan titik deteksi duplikat.

## Proposed Changes

### 1. Control Package Configuration — `vtol_config.yaml`

#### [MODIFY] [vtol_config.yaml](../workspace/src/vtol_control/config/vtol_config.yaml)
- Perbarui `yaw_error_threshold: 0.075` (turun 50% dari 0.15).
- Perbarui `error_threshold: 0.025` (turun 50% dari 0.05).

---

### 2. Vision Package — `aruco_receiver.py`

#### [MODIFY] [aruco_receiver.py](../workspace/src/vtol_vision/vtol_vision/aruco_receiver.py)
- **Kalkulasi Panah FRONT 3D**:
  - Proyeksikan vektor arah depan marker (`[0, marker_length * 1.5, 0]` atau axis heading marker) dari ruang 3D marker ke ruang piksel kamera menggunakan `cv2.projectPoints`.
  - Gambar panah tebal `cv2.arrowedLine` (warna Kuning/Sian) dari pusat marker ke titik arah FRONT beserta label `FRONT ▲`.
- **Refaktor Overlay Stream (`_draw_stream_overlay`)**:
  - Terapkan toleransi baru `tol_m = 0.025` (2.5 cm) dan `yaw_ok = abs(yaw_rad) < 0.075`.
  - Bersihkan gauge bar horizontal bawah, legend box, dan panel teks bertumpuk.
  - Tampilkan Top HUD Banner ringkas.

---

## Verification Plan

### Automated Tests
- Build paket `vtol_control` dan `vtol_vision` untuk memverifikasi sintaksis:
  ```bash
  docker exec vtol_dev bash -c "source /ros_entrypoint.sh && colcon build --packages-select vtol_control vtol_vision"
  ```

### Manual Verification
- Jalankan node vision dan kontrol, lalu periksa stream `http://localhost:8086`:
  1. Pastikan panah **FRONT** marker terlihat jelas dan menunjuk ke arah heading marker.
  2. Pastikan lingkaran toleransi menyusut ke **2.5 cm** dan indikator **IN ZONE** hanya aktif saat error $< 2.5\text{ cm}$ dan Yaw error $< 4.3^\circ$.
