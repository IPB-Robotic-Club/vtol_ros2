# Panduan Perintah Penting ROS 2 & MAVROS (`vtol_control`)

Dokumen ini berisi rangkuman seluruh perintah penting untuk mengelola, menjalankan, memantau, dan menghentikan (*kill*) proses ROS 2 dan MAVROS pada proyek `vtol_control`.

---

## 1. Menghentikan (*Kill*) & Memeriksa Node ROS 2 / MAVROS

Karena ROS 2 dan MAVROS berjalan sebagai proses latar belakang di dalam kontainer Docker, proses tersebut kadang-kadang tidak tertutup dengan bersih saat terminal dihentikan. Gunakan perintah berikut untuk menghentikan dan memeriksa sisa proses yang menggantung:

### A. Menghentikan Semua Proses Sekaligus (Direkomendasikan)
Untuk mematikan seluruh proses node watchdog (`vtol_core`), status HUD (`vehicle_status`), tester (`test_arm`/`mission_hover`), serta wrapper MAVROS sekaligus:
```bash
pkill -f -9 "vtol|mavros|ros2"
```
> [!NOTE]
> Perintah `pkill -f -9 ros2` saja **tidak cukup** untuk mematikan MAVROS atau `vtol_core`, karena biner eksekusi asli mereka berjalan tanpa memuat substring kata `"ros2"` pada argumen baris perintahnya.

### B. Memeriksa Sisa Proses yang Aktif
Untuk memastikan tidak ada proses duplikat atau sisa proses yang menggantung:
```bash
ps aux | grep -E "vtol|mavros|ros2"
```
Jika bersih, output perintah di atas hanya akan menampilkan baris pencarian `grep` itu sendiri.

### C. Menghentikan Secara Manual Menggunakan PID (Process ID)
Jika Anda ingin membunuh proses tertentu secara spesifik:
1. Jalankan perintah pemeriksaan di atas untuk menemukan nomor PID-nya (kolom kedua).
2. Bunuh proses menggunakan PID-nya (misal PID-nya adalah `1234`):
   ```bash
   kill -9 1234
   ```

---

## 2. Perintah Kompilasi (Build) Workspace

Setiap kali Anda melakukan perubahan pada file Python (`.py`), file launch (`.launch.py`), atau konfigurasi di dalam folder `src/vtol_control`, Anda wajib melakukan kompilasi ulang agar perubahan tersebut masuk ke direktori instalasi ROS 2:

*   **Kompilasi Package Tertentu (Direkomendasikan & Cepat):**
    ```bash
    cd /home/pilot/workspace && colcon build --packages-select vtol_control
    ```
*   **Kompilasi Seluruh Workspace:**
    ```bash
    cd /home/pilot/workspace && colcon build
    ```
*   **Source Environment (Wajib dijalankan setelah kompilasi di terminal baru):**
    ```bash
    source /opt/ros/jazzy/setup.bash
    source /home/pilot/workspace/install/setup.bash
    ```

---

## 3. Perintah Menjalankan Sistem (*Execution*)

Pastikan Anda selalu men-*source* environment terlebih dahulu sebelum menjalankan perintah-perintah di bawah ini:

*   **Opsi A: Jalankan MAVROS & Core Node (Background/Senyap)**
    *Menjalankan MAVROS dan sistem pengaman failsafe latar belakang secara senyap.*
    ```bash
    ros2 launch vtol_control vtol_core.launch.py
    ```

*   **Opsi B: Jalankan MAVROS, Core, & HUD Printer Bersamaan**
    *Menampilkan HUD grafis langsung di terminal saat startup.*
    ```bash
    ros2 launch vtol_control vtol_core.launch.py show_hud:=true
    ```

*   **Opsi C: Jalankan Menu Launcher Utama (CLI Menu)**
    *Menampilkan menu CLI interaktif untuk memilih program (HUD, Uji Coba Arming, Misi Hover).*
    ```bash
    ros2 run vtol_control menu_launcher
    ```

*   **Opsi D: Jalankan HUD Visualizer Mandiri (On-Demand)**
    *Membuka monitor HUD telemetri di terminal terpisah.*
    ```bash
    ros2 run vtol_control vehicle_status
    ```

---

## 4. Perintah Pemantauan Telemetri (*Telemetry Monitoring*)

Gunakan perintah-perintah ini untuk memeriksa kesehatan dan komunikasi data drone:

*   **Melihat Daftar Node yang Aktif:**
    ```bash
    ros2 node list
    ```
*   **Melihat Daftar Topik yang Aktif:**
    ```bash
    ros2 topic list
    ```
*   **Memeriksa Status Koneksi & Flight Mode Autopilot (Sekali echo):**
    ```bash
    ros2 topic echo /mavros/state --once
    ```
*   **Memantau Nilai Override Input Radio (RC):**
    ```bash
    ros2 topic echo /mavros/rc/in
    ```
*   **Membaca Koordinat & Ketinggian Lokal (Local Position ENU):**
    ```bash
    ros2 topic echo /mavros/local_position/pose
    ```
*   **Memeriksa Status Baterai Drone:**
    ```bash
    ros2 topic echo /mavros/battery
    ```

---

## 5. Troubleshooting: Jika MAVROS Stuck / Menunggu Lama

Jika node watchdog terus-menerus mencetak `Waiting for MAVROS state messages...` lebih dari 45 detik, ada dua kemungkinan penyebab utama:

### Opsi A: Simulator SITL di Host Windows Membeku (*Freeze*)
SITL ArduPilot di Windows kadang-kadang berhenti mengirimkan data (misal setelah laptop masuk mode sleep). 
*   **Solusi**: Tutup konsol SITL ArduPilot di Windows Anda, lalu jalankan kembali simulator SITL tersebut.

### Opsi B: Gangguan DDS Shared Memory di WSL2 (DDS Lockup)
Proses ROS 2 yang dimatikan paksa berulang kali dapat merusak segmen memori bersama (*shared memory*) pada WSL2, sehingga komunikasi antar-node tersumbat.
*   **Solusi**:
    1. Bersihkan seluruh proses ROS 2 dan MAVROS yang menggantung di kontainer:
       ```bash
       pkill -f -9 "vtol|mavros|ros2"
       ```
    2. Aktifkan konfigurasi FastDDS bypass shared memory (memaksa memakai UDP loopback) dengan mengekspor variabel lingkungan berikut sebelum menjalankan launch file:
       ```bash
       export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pilot/workspace/fastdds.xml
       ```
    3. Jalankan kembali launch file seperti biasa. Konfigurasi `fastdds.xml` ini akan menjamin jalur komunikasi DDS selalu bersih dan bebas dari *infinite spin-lock*!
