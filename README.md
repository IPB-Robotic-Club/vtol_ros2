# VTOL Autonomous ROS2 Workspace

Repositori ini menyediakan lingkungan pengembangan drone autonomous **Vertical Take-Off and Landing (VTOL)** menggunakan **ROS2**, **Docker**, dan **WSL2**.

Sistem ini didukung oleh:
*   **Autopilot (ArduPilot):** Navigasi dan penstabil penerbangan (fisik maupun SITL).
*   **MAVROS:** Jembatan penerjemah protokol MAVLink ke dalam ekosistem ROS2 secara real-time.

---

## Daftar Isi
1. [Prasyarat & Setup Cepat](#1-prasyarat--setup-cepat)
2. [Alur Kerja Harian](#2-alur-kerja-harian)
3. [Koneksi dengan Autopilot SITL & Mission Planner](#3-koneksi-dengan-autopilot-sitl--mission-planner)
4. [Cheat Sheet Perintah ROS 2 & MAVROS](#4-cheat-sheet-perintah-ros-2--mavros)
5. [Struktur Direktori & Mekanisme Volume Mount](#5-struktur-direktori--mekanisme-volume-mount)
6. [Deskripsi Skrip Kontrol (`vtol_control`)](#6-deskripsi-skrip-kontrol-vtol_control)
7. [Deployment ke SBC (Raspberry Pi)](#7-deployment-ke-sbc-raspberry-pi)
8. [Panduan Kalibrasi Kamera & Vision (`vtol_vision`)](docs/kalibrasi_kamera.md)


---

## 1. Prasyarat & Setup Cepat

### Prasyarat
Sebelum memulai, pastikan sistem Anda telah memiliki:
*   **WSL2 (Ubuntu)** - Khusus pengguna Windows.
*   **Docker Engine** & **Docker Compose** terinstal dan berjalan.

### Setup Cepat (Quick Start)
Buka terminal WSL2/Linux Anda, lalu jalankan perintah berikut:

1.  **Clone Repositori:**
    ```bash
    git clone git@github.com:IPB-Robotic-Club/vtol_ros2.git ~/vtol-dev
    cd ~/vtol-dev
    ```
2.  **Build Image & Jalankan Kontainer:**
    ```bash
    # Membangun image pengembangan (dev)
    docker compose build dev

    # Menyalakan kontainer di latar belakang
    docker compose up -d dev
    ```
3.  **Masuk ke Lingkungan Kontainer:**
    ```bash
    docker exec -it vtol_dev bash
    ```

---

## 2. Alur Kerja Harian

Gunakan urutan perintah berikut saat ingin mulai bekerja sehari-hari:

1.  **Nyalakan Layanan Docker (jika belum menyala):**
    ```bash
    sudo service docker start
    ```
2.  **Masuk & Tarik Pembaruan Kode:**
    ```bash
    cd ~/vtol-dev && git pull origin main
    ```
3.  **Nyalakan & Masuk Kontainer:**
    ```bash
    docker compose up -d dev
    docker exec -it vtol_dev bash
    ```
4.  **Matikan Kontainer (setelah selesai bekerja):**
    ```bash
    exit # keluar dari terminal kontainer
    docker compose down
    ```

---

## 3. Koneksi dengan Autopilot SITL & Mission Planner

> [!TIP]
> **Setup SITL Simulator:** Jika Anda belum memasang simulator SITL di PC/Host Anda, ikuti panduan instalasi dan konfigurasinya pada **[Panduan Setup SITL untuk Uji Coba](docs/setup_sitl.md)**.

Untuk menguji simulasi terbang drone VTOL secara autonomous, hubungkan ROS2 di dalam Docker dengan simulator internal di **Mission Planner** (Windows Host):

```
+----------------------------------------+          +-----------------------------------------+
|              WINDOWS HOST              |          |            DOCKER CONTAINER             |
|                                        |          |                                         |
|  [ Mission Planner (SITL Otomatis) ]  ◄┼──────────┼────► [ MAVROS Node (tcp://$WIN_IP:5762) ] |
|                                        | Jaringan |                                         |
|                                        |  WSL2    |                                         |
+----------------------------------------+          +-----------------------------------------+
```

1.  **Nyalakan SITL di Mission Planner (Windows):**
    *   Buka **Mission Planner**.
    *   Buka tab **Simulation**, lalu klik ikon wahana **Plane** atau **QuadPlane** (VTOL). Wahana simulator akan mulai secara mandiri dan langsung terhubung (*auto-connect*).
2.  **Jalankan MAVROS di Kontainer Docker (WSL):**
    Di dalam kontainer `vtol_dev`, jalankan perintah MAVROS untuk menyambung ke simulator Windows Host:
    ```bash
    ros2 run mavros mavros_node --ros-args -p fcu_url:="tcp://$WIN_IP:5762"
    ```
3.  **Verifikasi Konektivitas:**
    Di terminal kontainer baru, jalankan perintah telemetri berikut. Jika status `connected` bernilai `True`, koneksi berhasil tersambung:
    ```bash
    ros2 topic echo /mavros/state --once
    ```

---

## 4. Cheat Sheet Perintah ROS 2 & MAVROS

Jalankan perintah-perintah ini di dalam terminal kontainer Docker Anda:

### A. Pengelolaan Proses & Pembersihan (Kill)
Jika proses ROS2/MAVROS menggantung di latar belakang, bersihkan dengan perintah:
```bash
# Menghentikan semua proses node ROS2 & MAVROS sekaligus
pkill -f -9 "vtol|mavros|ros2"

# Memeriksa apakah masih ada sisa proses aktif
ps aux | grep -E "vtol|mavros|ros2"
```

### B. Kompilasi Workspace
Wajib dijalankan setiap kali Anda memodifikasi script Python, file launch, atau konfigurasi di dalam folder `src/vtol_control`:
```bash
cd /home/pilot/workspace
colcon build --packages-select vtol_control
source install/setup.bash
```

### C. Menjalankan Kontrol & Node Misi
*   **Opsi A: Jalankan MAVROS & Core Node (Background/Senyap):**
    ```bash
    ros2 launch vtol_control vtol_core.launch.py
    ```
*   **Opsi B: Jalankan MAVROS, Core, & HUD Printer Bersamaan:**
    ```bash
    ros2 launch vtol_control vtol_core.launch.py show_hud:=true
    ```
*   **Opsi C: Jalankan Menu Launcher CLI Interaktif (Mudah & Rekomendasi):**
    ```bash
    ros2 run vtol_control menu_launcher
    ```
*   **Opsi D: Jalankan HUD Status Drone Terpisah:**
    ```bash
    ros2 run vtol_control vehicle_status
    ```

### D. Pemantauan Telemetri
*   **Cek Daftar Node Aktif:** `ros2 node list`
*   **Cek Daftar Topik Aktif:** `ros2 topic list`
*   **Cek Input Radio Remote (RC):** `ros2 topic echo /mavros/rc/in`
*   **Cek Ketinggian & Koordinat Lokal:** `ros2 topic echo /mavros/local_position/pose`
*   **Cek Status Baterai:** `ros2 topic echo /mavros/battery`

---

## 5. Struktur Direktori & Mekanisme Volume Mount

Folder kode Anda di-mount langsung menggunakan fitur **Volume Mount** Docker:
*   Folder di WSL/Host Anda: `~/vtol-dev/workspace`
*   Terhubung langsung ke folder kontainer: `/home/pilot/workspace`

```
  KOMPUTER ANDA (WSL)                 KONTAINER DOCKER
 ┌──────────────────────┐            ┌──────────────────────┐
 │ ~/vtol-dev/          │            │ /home/pilot/         │
 │  ├── Dockerfile      │            │                      │
 │  ├── docker-compose  │            │                      │
 │  └── workspace/  ◄───┼──(Jembatan)┼───► workspace/       │
 └──────────────────────┘            └──────────────────────┘
```
**Keuntungan:** Anda bisa mengedit kode dengan nyaman menggunakan VS Code di Windows Host, dan perubahannya akan langsung terupdate di dalam kontainer Docker.

---

## 6. Deskripsi Skrip Kontrol (`vtol_control`)

Package **`vtol_control`** di dalam folder **[workspace/src/vtol_control/vtol_control/](workspace/src/vtol_control/vtol_control/)** berisi skrip-skrip berikut:

*   **[vtol_base.py](workspace/src/vtol_control/vtol_control/vtol_base.py)**: Skrip dasar (*base node class*) yang membungkus fungsi utilitas navigasi reusable (state monitoring, helper `takeoff`, `hover`, `land`, `abort_flight`).
*   **[vtol_core.py](workspace/src/vtol_control/vtol_control/vtol_core.py)**: Node monitor telemetri. Berjalan di background untuk mencatat perubahan koneksi, status arming, mode terbang, dan meminta data rate 10Hz dari autopilot.
*   **[menu_launcher.py](workspace/src/vtol_control/vtol_control/menu_launcher.py)**: Program menu CLI interaktif untuk memilih dan menjalankan modul pengujian atau misi.
*   **[vehicle_status.py](workspace/src/vtol_control/vtol_control/vehicle_status.py)**: Visualizer HUD telemetri (Tabel status mode, arming, baterai, ketinggian) di terminal.
*   **[test_arm.py](workspace/src/vtol_control/vtol_control/test_arm.py)**: Skrip uji coba sederhana untuk fitur arming/disarming drone.
*   **[mission_hover.py](workspace/src/vtol_control/vtol_control/mission_hover.py)**: Skrip misi autonomous lepas landas, hover beberapa detik, lalu mendarat otomatis.
*   **[mission_maneuver.py](workspace/src/vtol_control/vtol_control/mission_maneuver.py)**: Skrip misi autonomous lepas landas, maju/mundur, lalu mendarat otomatis.

---

## 7. Deployment ke SBC (Raspberry Pi)

Untuk melakukan deployment kode secara fisik ke komputer drone pendamping (SBC ARM64 seperti Raspberry Pi/Jetson), ikuti panduan langkah demi langkahnya di:
👉 **[Panduan Deployment ke SBC (Raspberry Pi)](docs/sbc_deployment.md)**
