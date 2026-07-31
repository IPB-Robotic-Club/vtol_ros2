# Panduan Kalibrasi Kamera & Deteksi ArUco Vision (`vtol_vision`)

Dokumen ini menjelaskan alur kerja, konsep arsitektur, dan langkah-langkah praktis untuk melakukan **kalibrasi kamera** serta pengujian deteksi **ArUco Marker** pada paket vision VTOL (`vtol_vision`).

---

## 1. Konsep Arsitektur Kamera (UDP Streaming & Profile Separation)

Pada sistem VTOL ini, seluruh gambar/frame kamera dikirimkan ke Docker/ROS 2 melalui **UDP Datagram (Port 5005)**:
- **Profil `sitl`**: Digunakan saat uji coba simulasi (Webots). Gambar bersumber dari simulator dengan karakteristik kamera ideal perspektif matematis (tanpa distorsi lensa, $D = [0, 0, 0, 0, 0]$).
- **Profil `raspi`**: Digunakan pada drone fisik (Raspberry Pi 5). Script `pi5_streamer.py` berjalan di host Raspi menangkap frame kamera CSI via `Picamera2` lalu memforward paket JPEG via UDP ke port 5005. Profil ini memuat **matriks kalibrasi fisik lensa** hasil proses kalibrasi papan catur.

Profil dikonfigurasi melalui [workspace/src/vtol_vision/config/vision_config.yaml](../workspace/src/vtol_vision/config/vision_config.yaml):

```yaml
# Pilih profil aktif: "sitl" (Simulasi Webots) atau "raspi" (Drone Fisik Raspi 5)
active_profile: "sitl"
```

---

## 2. Prasyarat Kalibrasi Kamera

Sebelum memulai kalibrasi kamera fisik Raspi 5, pastikan Anda telah menyiapkan:
1. **Pola Papan Catur (Checkerboard 9x7 Kotak / 8x6 Sudut Internal)**:
   Gunakan script bawaan untuk menghasilkan pola catur:
   ```bash
   python3 generate_board.py
   ```
   Script ini menghasilkan file `chessboard.png`. Cetak atau tampilkan pola tersebut pada permukaan yang rata/kaku (misal ditempel di papan tripleks/karton tebal).
2. **Ukuran Kotak Fisik**: Ukur panjang 1 kotak catur hasil cetakan dengan penggaris dalam satuan meter (secara default diset `0.025` meter / 2.5 cm).

---

## 3. Langkah-Langkah Kalibrasi Kamera Fisik

### Langkah 1: Jalankan Streamer & Receiver Vision
1. **Di Host Raspberry Pi 5**: Jalankan script streamer kamera CSI:
   ```bash
   python3 pi5_streamer.py
   ```
2. **Di Container Docker (`vtol_dev`)**: Jalankan node penerima ArUco:
   ```bash
   ros2 run vtol_vision aruco_receiver
   ```
   *(Node ini membuka MJPEG stream server di `http://localhost:8086`)*.

---

### Langkah 2: Ambil Foto Sampel Kalibrasi (15 - 25 Foto)
Gunakan salah satu dari dua alat penangkap foto yang telah disediakan:

#### Pilihan A: Menggunakan Web UI Capture Tool (Rekomendasi)
1. Jalankan script Web UI capture:
   ```bash
   python3 capture_web_ui.py
   ```
2. Buka peramban (browser) di URL `http://localhost:8087`.
3. Posisikan papan catur di depan kamera, lalu klik tombol **CAPTURE FRAME**. Foto akan disimpan otomatis ke folder `calibration_images/`.

#### Pilihan B: Menggunakan Terminal CLI (Headless Mode)
1. Jalankan script terminal capture:
   ```bash
   python3 capture_headless.py
   ```
2. Tekan tombol `ENTER` di terminal setiap kali mengambil foto baru.

> **💡 Tips Pengambilan Sampel Foto yang Presisi**:
> - Ambil foto papan catur dari **berbagai sudut kemiringan** (miring kiri, miring kanan, miring atas, miring bawah).
> - Variasikan **jarak papan catur** (jarak dekat memenuhi sebagian besar layar hingga jarak menengah).
> - Pastikan seluruh **8x6 sudut internal** terlihat utuh (tidak terpotong tepi layar) dan gambar **fokus/tidak blur**.

---

### Langkah 3: Eksekusi Kalkulasi Kalibrasi
Setelah mengumpulkan minimal 15–20 foto pada folder `calibration_images/`, jalankan script kalibrasi:

```bash
python3 calibrate_camera.py
```

**Proses yang Dilakukan Script**:
1. Membaca seluruh foto `.png` dari `calibration_images/`.
2. Mendeteksi titik-titik sudut internal dengan `cv2.findChessboardCorners()`.
3. Menghaluskan titik koordinat secara sub-pixel (`cv2.cornerSubPix()`).
4. Menghitung Matriks Kamera ($K$) dan Koefisien Distorsi Lensa ($D$) dengan `cv2.calibrateCamera()`.
5. Menghitung nilai **Reprojection Error** (skala piksel). Idealnya bernilai **$< 0.5$ piksel**.
6. Menyimpan parameter kalibrasi ke file `camera_calibration.yaml`.

---

## 4. Mengaktifkan Profil Kamera Fisik (Hasil Kalibrasi)

Setelah file `camera_calibration.yaml` berhasil terbentuk:

1. Buka file konfigurasi vision `workspace/src/vtol_vision/config/vision_config.yaml`.
2. Ubah `active_profile` menjadi `raspi`:
   ```yaml
   active_profile: "raspi"
   ```
3. Saat node `aruco_receiver` dijalankan, ia akan secara otomatis membaca file `camera_calibration.yaml` dan menerapkan koreksi distorsi lensa secara real-time.

---

## 5. Ringkasan File Terkait Vision & Kalibrasi

| Nama File | Deskripsi / Kegunaan |
| :--- | :--- |
| [pi5_streamer.py](../pi5_streamer.py) | Streamer Picamera2 di host Raspi 5 ke UDP port 5005. |
| [generate_board.py](../generate_board.py) | Generator gambar pola papan catur & ArUco marker ID 0. |
| [capture_web_ui.py](../capture_web_ui.py) | Tool Web UI di port 8087 untuk mengambil foto sampel kalibrasi. |
| [capture_headless.py](../capture_headless.py) | Tool CLI terminal untuk mengambil foto sampel kalibrasi. |
| [calibrate_camera.py](../calibrate_camera.py) | Script kalkulasi parameter intrinsik & distorsi lensa OpenCV. |
| [camera_calibration.yaml](../camera_calibration.yaml) | Output file parameter kalibrasi hasil kalkulasi. |
| [vision_config.yaml](../workspace/src/vtol_vision/config/vision_config.yaml) | File konfigurasi utama vision & profil aktif (`sitl` vs `raspi`). |
