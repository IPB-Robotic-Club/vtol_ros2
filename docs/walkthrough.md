# Walkthrough - Konfigurasi MAVLink FCU URL Berbasis YAML & Setup SITL

Seluruh proses modifikasi untuk memindahkan konfigurasi FCU URL dari berkas teks mentah (`fcu_url.txt`) ke berkas konfigurasi YAML (`vtol_config.yaml`) yang terstruktur, serta penambahan panduan setup SITL baru, telah berhasil dilaksanakan dan divalidasi.

## Perubahan yang Dilakukan

1.  **Berkas Konfigurasi YAML Baru:**
    - Membuat berkas baru [vtol_config.yaml](../workspace/src/vtol_control/config/vtol_config.yaml) berisi profil koneksi untuk `tcp` (simulasi) dan `serial` (Pixhawk fisik).
    - Menghapus berkas konfigurasi lama `fcu_url.txt`.
2.  **Helper Python Parser (`config_reader.py`):**
    - Membuat modul pembaca konfigurasi terpusat pada [config_reader.py](../workspace/src/vtol_control/vtol_control/config_reader.py). Modul ini membaca `vtol_config.yaml` dan mem-parsing profil yang aktif secara dinamis.
3.  **Pembaruan Launch Files:**
    - Mengintegrasikan pembaca konfigurasi baru ke dalam [vtol_core.launch.py](../workspace/src/vtol_control/launch/vtol_core.launch.py) dan [test_arm.launch.py](../workspace/src/vtol_control/launch/test_arm.launch.py).
    - Menambahkan mekanisme fallback path (`sys.path.append`) sehingga modul tetap dapat dibaca secara dinamis baik saat package telah ter-source maupun saat proses development sebelum sourcing penuh.
4.  **Panduan Setup SITL & Integrasi Dokumentasi:**
    - Membuat berkas panduan baru [setup_sitl.md](setup_sitl.md) yang mendokumentasikan instalasi, eksekusi, konfigurasi koordinat awal (home), serta aktivasi rangefinder dan optical flow pada ArduPilot SITL.
    - Menghubungkan pranala rujukan dari [README.md](../README.md) dan [failsafe_dan_konfigurasi.md](failsafe_dan_konfigurasi.md) ke panduan setup SITL yang baru dibuat.

## Pengujian dan Hasil Validasi

### 1. Kompilasi Workspace
Kompilasi berhasil dilakukan tanpa ada error pada package `vtol_control`:
```bash
docker exec vtol_dev bash -c "cd /home/pilot/workspace && colcon build --packages-select vtol_control"
```
**Hasil:** `Finished <<< vtol_control` (Sukses).

### 2. Pengujian Unit Pembaca Konfigurasi
Menguji keluaran dari helper `get_fcu_url()` secara langsung dalam lingkungan ROS2 Jazzy di kontainer:
- **Profil `tcp` (Default):**
  Output: `FCU URL: tcp://127.0.0.1:5762`
- **Profil `serial`:**
  Output: `FCU URL: /dev/ttyACM0:921600`

### 3. Pengujian Launching
Menjalankan launch file untuk memverifikasi inisialisasi MAVROS dengan parameter URL baru:
```bash
ros2 launch vtol_control vtol_core.launch.py
```
**Log output yang diperoleh:**
```
======================================================
 VTOL Core Launching with FCU URL: tcp://127.0.0.1:5762
 FastDDS Configuration File: /home/pilot/workspace/src/vtol_control/config/fastdds.xml
======================================================
[INFO] [mavros_node-1]: process started with pid [423]
[INFO] [vtol_core-2]: process started with pid [424]
...
[mavros_node-1] [INFO] [mavros.mavros_node]: FCU URL: tcp://127.0.0.1:5762
```
MAVROS berhasil diluncurkan dengan URL FCU yang sesuai dengan berkas konfigurasi YAML aktif.
