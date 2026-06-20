# Panduan Perintah Penting ROS 2 & MAVROS (`vtol_control`)

Dokumen ini berisi rangkuman seluruh perintah penting untuk mengelola, menjalankan, memantau, dan menghentikan (*kill*) proses ROS 2 dan MAVROS pada proyek `vtol_control`.

---

## 1. Menghentikan (*Kill*) Node ROS 2 & MAVROS

Karena ROS 2 berjalan sebagai proses Linux biasa di latar belakang kontainer, Anda dapat menghentikannya secara paksa menggunakan perintah terminal berikut:

### A. Menghentikan Menggunakan Nama Proses (Direkomendasikan)
Gunakan perintah `pkill` dengan opsi pencocokan pola nama `-f` untuk menghentikan node tertentu:

*   **Menghentikan MAVROS:**
    ```bash
    pkill -f -9 mavros_node
    ```
*   **Menghentikan VTOL Core Node:**
    ```bash
    pkill -f -9 vtol_core
    ```
*   **Menghentikan HUD/Vehicle Status:**
    ```bash
    pkill -f -9 vehicle_status
    ```
*   **Menghentikan Semua Proses ROS 2 Sekaligus:**
    ```bash
    pkill -f -9 ros2
    ```

### B. Menghentikan Secara Manual Menggunakan PID (Process ID)
Jika Anda ingin melihat dan membunuh proses secara spesifik:
1.  Cari PID dari proses ROS 2 yang sedang berjalan:
    ```bash
    ps aux | grep -E "mavros|vtol|ros2"
    ```
2.  Bunuh proses tersebut menggunakan PID-nya (misal PID-nya adalah `1234`):
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
