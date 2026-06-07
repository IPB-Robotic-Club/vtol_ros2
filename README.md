# Panduan Lengkap & Komprehensif: Pengembangan VTOL Autonomous (Untuk Pemula)

Selamat datang di repositori pengembangan **Autonomous Vertical Take-Off and Landing (VTOL)**! Repositori ini dirancang khusus untuk memfasilitasi Anda yang baru memulai belajar pemrograman drone autonomous menggunakan **ROS2**, **Docker**, dan **WSL2** di sistem operasi Windows.

Sistem ini dirancang untuk mendukung teknologi mutakhir drone autonomous modern:
*   **Perception & Machine Learning:** Integrasi modul kamera cerdas untuk deteksi target, presisi landing, dan penghindaran rintangan berbasis AI/ML.
*   **Autopilot (ArduPilot):** Menggunakan sistem autopilot kelas dunia untuk navigasi dan penstabil penerbangan fisik maupun SITL.
*   **MAVROS:** Jembatan komunikasi berkinerja tinggi yang menerjemahkan protokol MAVLink ke dalam ekosistem ROS2 secara real-time.

Dengan metode kontainerisasi (Docker), Anda tidak perlu khawatir merusak sistem komputer Anda atau pusing menginstal puluhan dependensi robotika yang rumit. Semuanya sudah dikemas rapi dan siap dijalankan.

---

## Daftar Isi

### BAGIAN I: PENGENALAN & KONSEP
1. [Konsep Dasar Bagi Pemula](#1-konsep-dasar-bagi-pemula)
2. [Arsitektur Sistem (Bagaimana Semua Saling Terhubung)](#2-arsitektur-sistem-bagaimana-semua-saling-terhubung)

#### BAGIAN II: PERSIAPAN & INSTALASI (SETUP)
3. [Prasyarat & Persiapan Sistem Windows](#3-prasyarat--persiapan-sistem-windows)
4. [Panduan Instalasi Langkah-demi-Langkah (Zero to Hero)](#4-panduan-instalasi-langkah-demi-langkah-zero-to-hero)
   - [Langkah 1: Setup WSL2 & WSLg](#langkah-1-setup-wsl2--wslg)
   - [Langkah 2: Instalasi Docker Engine (Native di WSL)](#langkah-2-instalasi-docker-engine-native-di-wsl)
   - [Langkah 3: Clone Repositori & Jalankan Kontainer](#langkah-3-clone-repositori--jalankan-kontainer)
   - [Langkah 4: Eksekusi dan Verifikasi](#langkah-4-eksekusi-dan-verifikasi)

### BAGIAN III: ALUR KERJA HARIAN (DAILY WORKFLOW)
5. [Alur Kerja Harian (Daily Workflow)](#5-alur-kerja-harian-daily-workflow)
6. [Koneksi dengan Autopilot SITL & Mission Planner](#6-koneksi-dengan-autopilot-sitl--mission-planner)
7. [Panduan Manajemen Perintah Docker](#7-panduan-manajemen-perintah-docker)

### BAGIAN IV: MANIFESTO PROYEK DRONE AUTONOMOUS (STRUKTUR & ARSITEKTUR)
8. [Struktur Direktori & Mekanisme Berbagi File (Volume Mount)](#8-struktur-direktori--mekanisme-berbagi-file-volume-mount)
9. [Arsitektur Software Drone: Monolitik vs Modular](#9-arsitektur-software-drone-monolitik-vs-modular)
10. [Panduan Pembuatan & Pengembangan ROS2 Package](#10-panduan-pembuatan--pengembangan-ros2-package)

---

## 1. Konsep Dasar Bagi Pemula

Sebelum masuk ke instalasi teknis, mari kita pahami istilah-istilah utama yang akan sering Anda gunakan:

*   **WSL2 (Windows Subsystem for Linux 2):** Fitur Windows yang memungkinkan Anda menjalankan sistem operasi Linux (Ubuntu) secara native di dalam Windows tanpa perlu dual-boot atau menggunakan VirtualBox yang lambat.
*   **WSLg (WSL GUI):** Subsistem WSL yang otomatis meneruskan tampilan grafis aplikasi Linux ke Windows. Berkat ini, simulator 3D seperti Gazebo bisa tampil di Windows Anda.
*   **Docker:** Bayangkan Docker seperti "kotak bekal" yang sudah berisi makanan lengkap. Di dunia software, Docker mengemas sistem operasi Linux mini beserta semua library ROS2 dan Gazebo ke dalam satu paket (**Image**). Saat dijalankan, paket ini menjadi **Container** yang terisolasi dari sistem utama PC Anda.
*   **ROS2 (Robot Operating System 2):** Bukan sistem operasi seperti Windows atau Linux, melainkan sebuah framework/middleware. ROS2 menyediakan pipa komunikasi (disebut **Topics**, **Services**, dan **Actions**) agar program-program kecil (disebut **Nodes**) seperti program sensor, kamera, dan kontrol motor dapat saling bertukar data dengan mudah.
*   **MAVLink & MAVROS:**
    *   *MAVLink* adalah bahasa protokol komunikasi standar yang digunakan oleh Flight Controller drone (seperti Pixhawk dengan firmware ArduPilot atau PX4).
    *   *MAVROS* adalah program penerjemah di ROS2 yang menerjemahkan bahasa MAVLink menjadi bahasa ROS2 (Topics), sehingga Anda bisa mengontrol drone lewat script ROS2.
*   **Perception & Machine Learning:** Sistem kecerdasan drone yang bertugas mengolah input citra dari sensor kamera onboard (menggunakan pustaka seperti OpenCV, PyTorch, atau YOLO) untuk mengenali lingkungan sekitar secara dinamis, melakukan navigasi visual, atau mendarat presisi pada target.
*   **SITL (Software In The Loop):** Simulator autopilot drone yang berjalan di komputer. Autopilot mengira ia sedang terbang di drone asli, padahal ia hanya menerima sensor buatan dan mengirim perintah motor ke lingkungan simulasi.

---

## 2. Arsitektur Sistem (Bagaimana Semua Saling Terhubung)

Diagram berikut menjelaskan bagaimana komponen perangkat lunak di Windows, WSL2, dan di dalam kontainer Docker saling berkomunikasi:

```mermaid
graph TD
    subgraph WH ["Windows Host (PC/Laptop)"]
        MP["Mission Planner / SITL Autopilot"]
    end

    subgraph WSL ["WSL2 Environment (Ubuntu 24.04)"]
        Docker["Docker Engine Native"]
        WSLg["WSLg - Server Antarmuka Grafis"]
    end

    subgraph DC ["Docker Container"]
        subgraph DF ["vtol_dev (ROS2 & MAVROS)"]
            ROS2["ROS 2 Jazzy (Desktop)"] <--> MAVROS["Node MAVROS"]
        end
    end

    MP <-->|Komunikasi Jaringan TCP: WIN_IP| MAVROS
    WSLg <-->|Meneruskan Tampilan Grafis (RViz, etc.)| ROS2
```

---

## 3. Prasyarat & Persiapan Sistem Windows

Untuk memastikan simulasi 3D berjalan dengan lancar, pastikan PC/Laptop Anda memenuhi spesifikasi berikut:

### Spesifikasi Perangkat Keras
| Komponen | Spesifikasi Minimum |
| :--- | :--- |
| **CPU** | Intel Core i5 / AMD Ryzen 5 (Generasi 8+) |
| **RAM** | 8 GB |
| **Storage** | 10 GB ruang kosong (SSD sangat disarankan) |

### Langkah Persiapan di Windows (Sebelum Mulai)
1.  **Update Driver VGA NVIDIA (Khusus pengguna NVIDIA):**
    *   Unduh dan instal driver resmi terbaru melalui aplikasi NVIDIA GeForce Experience atau website resmi NVIDIA.
    *   **Catatan Kritis:** Cukup instal driver di Windows. Jangan pernah mengunduh/menginstal driver NVIDIA untuk Linux di dalam terminal Ubuntu WSL Anda! WSL akan otomatis menjembatani akses ke driver Windows tersebut.

---

## 4. Panduan Instalasi Langkah-demi-Langkah (Zero to Hero)

Silakan buka komputer Anda dan ikuti panduan instalasi di bawah ini secara perlahan.

### Langkah 1: Setup WSL2 & WSLg
1.  Buka **Windows Search**, ketik `powershell`, klik kanan pada **Windows PowerShell**, lalu pilih **Run as Administrator**.
2.  Ketik perintah berikut untuk menginstal WSL2 secara otomatis (secara default akan mengunduh Ubuntu):
    ```powershell
    wsl --install
    ```
3.  Setelah selesai, lakukan pembaruan sistem WSL untuk memastikan sistem grafis (WSLg) terbaru telah terpasang:
    ```powershell
    wsl --update
    ```
4.  **Restart PC/Laptop Anda.**
5.  Setelah PC menyala kembali, buka menu Start Windows, cari dan jalankan aplikasi bernama **Ubuntu**.
6.  Terminal Ubuntu pertama kali akan meminta Anda memasukkan **Username** dan **Password** baru untuk sistem Linux Anda. Catat password ini karena akan digunakan saat menjalankan perintah `sudo`!

---

### Langkah 2: Instalasi Docker Engine (Native di WSL)
> [!NOTE]
> Kami sengaja tidak menggunakan **Docker Desktop** karena aplikasi tersebut cukup berat dan sering menimbulkan konflik routing jaringan dengan WSL2. Kita akan menginstal Docker Engine asli Linux langsung di dalam terminal Ubuntu Anda agar performanya jauh lebih cepat.

Buka terminal **Ubuntu (WSL)** Anda, lalu salin dan jalankan perintah-perintah berikut (Anda akan diminta memasukkan password Linux yang Anda buat di Langkah 1):

```bash
# 1. Perbarui daftar paket aplikasi & instal peralatan dasar
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg lsb-release

# 2. Buat folder untuk menyimpan kunci keamanan Docker
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# 3. Daftarkan repositori resmi Docker ke dalam sistem Ubuntu Anda
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 4. Instal Docker Engine dan plugin Docker Compose
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# 5. Beri izin user Anda agar bisa menjalankan Docker tanpa harus mengetik 'sudo' terus-menerus
sudo usermod -aG docker $USER
```

> [!IMPORTANT]
> **Tutup terminal Ubuntu Anda (ketik `exit` atau klik tombol X), lalu buka kembali aplikasi Ubuntu.** Langkah ini wajib dilakukan agar izin grup docker yang baru saja kita tambahkan aktif.

Untuk memastikan Docker sudah berjalan dengan benar, ketik:
```bash
docker ps
```
Jika tidak muncul pesan error dan terminal menampilkan daftar tabel kosong, berarti Docker Anda telah aktif!

---

### Langkah 3: Instalasi NVIDIA Container Toolkit
*(Langkah ini khusus untuk laptop yang memiliki kartu grafis diskrit **NVIDIA**. Jika laptop Anda hanya menggunakan Intel HD atau AMD Radeon terintegrasi, Anda bisa melewati langkah ini).*

Langkah ini diperlukan agar kontainer Docker Anda dapat mendeteksi dan menggunakan kekuatan GPU NVIDIA Anda untuk rendering 3D di Gazebo.

Di dalam terminal **Ubuntu (WSL)**, jalankan:

```bash
# 1. Unduh dan daftarkan kunci keamanan repositori NVIDIA
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# 2. Daftarkan repositori toolkit NVIDIA ke sistem Ubuntu
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 3. Instal NVIDIA Container Toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# 4. Konfigurasi runtime Docker agar mengenali kartu grafis NVIDIA
sudo nvidia-ctk runtime configure --runtime=docker

# 5. Mulai ulang layanan Docker di WSL Anda
sudo service docker restart
```

---

### Langkah 4: Membangun & Menjalankan Kontainer
*   **Clone Git Repository:**
    ```bash
    git clone https://github.com/IPB-Robotic-Club/vtol_ros2.git ~/vtol_dev
    cd ~/vtol_dev
    ```
2.  **Bangun Image Kontainer:**
    ```bash
    docker compose build
    ```
3.  **Jalankan Kontainer:**
    ```bash
    docker compose up -d
    ```

---

### Langkah 4: Eksekusi dan Verifikasi
Untuk memverifikasi bahwa kontainer berhasil terpasang dan siap digunakan:

1.  **Masuk ke dalam kontainer yang menyala:**
    ```bash
    docker exec -it vtol_dev bash
    ```
2.  **Cek Deteksi IP Windows Host:**
    Di dalam kontainer, jalankan:
    ```bash
    echo $WIN_IP
    ```
    *Terminal harus mencetak alamat IP Gateway Windows Anda (misal: `172.x.x.x`).*

---

## 5. Alur Kerja Harian (Daily Workflow)

Bagaimana cara menggunakan lingkungan pengembangan ini setiap harinya? Ini adalah urutan langkah yang perlu Anda lakukan saat ingin mulai bekerja:

1.  **Buka Terminal Ubuntu WSL.**
2.  **Pastikan Service Docker Menyala:**
    WSL2 terkadang tidak otomatis menyalakan Docker saat Windows baru menyala. Jalankan perintah ini untuk memastikan layanan Docker aktif:
    ```bash
    sudo service docker start
    ```
3.  **Masuk ke Folder Project & Tarik Update Terbaru:**
    Lakukan `git pull` setiap hari agar Anda selalu mendapatkan pembaruan file dan dockerfile terbaru dari repositori utama:
    ```bash
    cd ~/vtol_dev
    git pull origin main
    ```
4.  **Nyalakan Kontainer:**
    ```bash
    docker compose up -d
    ```
5.  **Masuk ke Dalam Lingkungan Kontainer Linux ROS2:**
    ```bash
    docker exec -it vtol_dev bash
    ```
6.  **Setelah Selesai Bekerja:**
    Keluar dari kontainer dengan mengetik `exit`, lalu matikan kontainer Docker Anda agar tidak memakan memori RAM laptop Anda di latar belakang:
    ```bash
    docker compose down
    ```

---

## 6. Koneksi dengan Autopilot SITL & Mission Planner

Untuk menguji simulasi terbang drone VTOL Anda secara autonomous, kita dapat menyambungkan ROS2 di dalam Docker langsung dengan simulator internal di **Mission Planner**.

```
+----------------------------------------+          +-----------------------------------------+
|              WINDOWS HOST              |          |            DOCKER CONTAINER             |
|                                        |          |                                         |
|  [ Mission Planner (SITL Otomatis) ]  ◄┼──────────┼────► [ MAVROS Node (tcp://$WIN_IP:5762) ] |
|                                        | Jaringan |                                         |
|                                        |  WSL2    |                                         |
+----------------------------------------+          +-----------------------------------------+
```

Langkah-langkah koneksi:

1.  **Nyalakan SITL di Mission Planner (Windows):**
    *   Buka aplikasi **Mission Planner** di Windows Anda.
    *   Masuk ke tab menu **Simulation** di bagian atas.
    *   Klik tombol ikon wahana **Plane** atau **QuadPlane** (VTOL).
    *   Mission Planner akan mengunduh firmware secara otomatis, memulai simulator penerbangan SITL secara mandiri, dan langsung tersambung secara otomatis (*Auto-connect*). Anda **tidak perlu menginput IP address manual** di langkah ini karena semuanya sudah ditangani langsung oleh antarmuka Mission Planner.
2.  **Jalankan Jembatan MAVROS di Kontainer Docker (WSL):**
    *   Buka terminal Ubuntu WSL Anda dan masuk ke kontainer:
        ```bash
        docker exec -it vtol_dev bash
        ```
    *   Jalankan node MAVROS untuk tersambung ke simulator di Windows menggunakan port MAVLink TCP bawaan (biasanya `5762` or `5760`):
        ```bash
        ros2 run mavros mavros_node --ros-args -p fcu_url:="tcp://$WIN_IP:5762"
        ```
3.  **Verifikasi Konektivitas Topik ROS2:**
    *   Buka tab terminal Ubuntu WSL baru (biarkan MAVROS tetap menyala di terminal pertama).
    *   Masuk kembali ke dalam kontainer:
        ```bash
        docker exec -it vtol_dev bash
        ```
    *   Tampilkan semua topik aktif yang sedang diterbitkan oleh MAVROS:
        ```bash
        ros2 topic list
        ```
    *   Jika data sensor autopilot berhasil tersambung, Anda akan melihat puluhan topik terdaftar (seperti `/mavros/state`, `/mavros/global_position/local`, dll.). Anda bisa membaca status koneksi dengan perintah:
        ```bash
        ros2 topic echo /mavros/state
        ```
        Jika baris log terminal menampilkan `connected: True`, berarti program kontrol ROS2 Anda di Docker sudah tersambung sepenuhnya dengan simulasi drone di Windows!

---

## 7. Panduan Manajemen Perintah Docker

Berikut adalah perintah-perintah dasar Docker Compose untuk mengelola kontainer VTOL Anda:

*   **Membangun Ulang Image:**
    ```bash
    docker compose build
    ```
*   **Menjalankan Kontainer:**
    ```bash
    docker compose up -d
    ```
*   **Masuk ke Kontainer:**
    ```bash
    docker exec -it vtol_dev bash
    ```
*   **Menghentikan Kontainer:**
    ```bash
    docker compose down
    ```

Untuk deployment ke komputer drone fisik (SBC ARM64) seperti Raspberry Pi atau NVIDIA Jetson, Anda dapat menyalin file `Dockerfile` dan menjalankan build secara native pada SBC tersebut menggunakan perintah build yang sama.

---

## 8. Struktur Direktori & Mekanisme Berbagi File (Volume Mount)

Bagi pemula, konsep penyimpanan Docker terkadang membingungkan. Jika Anda menghapus kontainer, apakah file kode Anda akan hilang? **Jawabannya: Tidak!**

Kami menggunakan fitur bernama **Volume Mount** (atau bind mount) yang menjembatani folder di komputer asli Anda dengan folder di dalam kontainer.

### Pemetaan Folder
*   Folder fisik di komputer Anda: `~/vtol_dev/workspace`
*   Terhubung langsung ke folder di dalam kontainer: `/home/pilot/workspace`

```
  KOMPUTER ANDA (WSL)                 KONTAINER DOCKER
 ┌──────────────────────┐            ┌──────────────────────┐
 │ ~/vtol_dev/          │            │ /home/pilot/         │
 │  ├── Dockerfile      │            │                      │
 │  ├── docker-compose  │            │                      │
 │  └── workspace/  ◄───┼──(Jembatan)┼───► workspace/       │
 │       └── main.py    │            │       └── main.py    │
 └──────────────────────┘            └──────────────────────┘
```

### Keuntungan Utama:
Anda bisa membuka VS Code di Windows, mengedit file python di dalam folder `~/vtol_dev/workspace`, dan seketika itu juga file tersebut akan terupdate di dalam kontainer Docker Anda untuk dijalankan menggunakan ROS2! Anda tidak perlu mengetik kode menggunakan editor terminal yang menyulitkan seperti `nano` or `vi`.

---

## 9. Arsitektur Software Drone: Monolitik vs Modular

Saat merancang arsitektur sistem kendali drone autonomous secara *full stack* (dilengkapi kamera navigasi AI + kontrol penerbangan MAVROS), **sangat tidak disarankan menyatukan seluruh kode dalam satu package tunggal (Monolitik)**. Pendekatan terbaik adalah memecahnya menjadi beberapa package modular yang saling berkomunikasi melalui topik ROS2.

### Mengapa Harus Modular?
1. **Pemisahan Peran (Separation of Concerns):** Menjaga agar kode navigasi terbang (`vtol_control`), pengolahan kamera AI (`vtol_perception`), dan visualisasi/simulasi (`vtol_simulation`) terpisah satu sama lain.
2. **Efisiensi Deployment SBC (Raspberry Pi):** Saat dipasang pada komputer drone fisik, Anda hanya perlu mentransfer package navigasi dan sensor. File berat simulator 3D bisa diabaikan untuk menghemat RAM dan memori.
3. **Ketahanan Sistem (Failsafe):** Jika modul visi komputer berbasis Deep Learning mengalami error/crash, sistem kontrol penerbangan utama tetap berjalan mandiri dan dapat mengeksekusi prosedur pendaratan darurat (*Return-to-Launch*).

### Rekomendasi Struktur Workspace Modular (Kasus: ArUco Precision Landing)

Berikut adalah struktur direktori workspace ideal (`~/vtol_dev/workspace/src/`) untuk sistem drone autonomous yang menggunakan **ArUco Marker** untuk presisi pendaratan:

```text
workspace/
└── src/
    ├── vtol_msgs/                  <-- [Package Custom Message]
    │   └── msg/
    │       └── ArucoMarkerPose.msg # Mendefinisikan koordinat X, Y, Z marker & ID ArUco
    │
    ├── vtol_perception/            <-- [Package Visi Komputer / ML]
    │   ├── vtol_perception/
    │   │   ├── __init__.py
    │   │   └── aruco_detector.py   # Node Python (OpenCV) untuk mendeteksi ArUco dari kamera,
    │   │                           # menghitung 3D pose, lalu mem-publish topik '/vtol/aruco_pose'
    │   └── package.xml             # Dependensi: rclpy, cv_bridge, v4l2_camera, vtol_msgs
    │
    ├── vtol_control/               <-- [Package Otak Kendali & Navigasi]
    │   ├── vtol_control/
    │   │   ├── __init__.py
    │   │   └── precision_landing.py # Node untuk membaca '/vtol/aruco_pose', menghitung aksi
    │   │                            # koreksi terbang, lalu mengirimkan setpoint ke MAVROS
    │   └── package.xml              # Dependensi: rclpy, mavros_msgs, vtol_msgs
    │
    └── vtol_bringup/               <-- [Package Saklar / Launch Configuration]
        └── launch/
            └── autonomous_landing.launch.py # Menyalakan driver kamera, MAVROS, detector,
                                             # dan kontroler sekaligus secara otomatis
```

### Alur Aliran Data (Data Flow):
1. **`v4l2_camera`** atau Gazebo kamera mengirimkan gambar mentah (`sensor_msgs/msg/Image`) ke topik `/camera/image_raw`.
2. **`vtol_perception` (Node `aruco_detector`)** mengambil gambar tersebut, memprosesnya dengan OpenCV ArUco, menghitung translasi 3D target, lalu menyiarkan hasilnya (`vtol_msgs/msg/ArucoMarkerPose`) ke topik `/vtol/aruco_pose`.
3. **`vtol_control` (Node `precision_landing`)** mendengarkan topik tersebut, menjalankan kontroler penyesuaian posisi (misal PID), lalu mem-publish koordinat pendaratan presisi ke **MAVROS** via topik `/mavros/setpoint_position/local` atau mengirim perintah pendaratan MAVLink.

---

## 10. Panduan Pembuatan & Pengembangan ROS2 Package

Bila Anda ingin memaketkan kode Python Anda menjadi modul yang rapi dan terstandarisasi di ROS2 (misal untuk monitoring sensor, auto-takeoff, dll.), ikuti langkah-langkah di bawah ini.

### Langkah 1: Masuk ke Kontainer & Buat Package
1. Masuk ke kontainer yang berjalan:
   ```bash
   docker exec -it vtol_dev bash
   ```
2. Pindah ke direktori workspace utama dan buat package berbasis Python dengan dependensi `rclpy` dan `mavros_msgs`:
   ```bash
   cd ~/workspace
   ros2 pkg create --build-type ament_python vtol_monitoring --dependencies rclpy mavros_msgs
   ```

### Langkah 2: Buat Node Baru
Buat file python baru, misalnya `state_listener_node.py` di dalam folder `vtol_monitoring/vtol_monitoring/`:
```python
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State

class MavrosStateListener(Node):
    def __init__(self):
        super().__init__('mavros_state_listener')
        self.subscription = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        self.get_logger().info('Node monitor /mavros/state telah aktif!')

    def state_callback(self, msg):
        self.get_logger().info(f'Connected: {msg.connected} | Armed: {msg.armed} | Mode: {msg.mode}')

def main(args=None):
    rclpy.init(args=args)
    node = MavrosStateListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Langkah 3: Konfigurasi setup.py
Buka file `setup.py` di folder root package (`vtol_monitoring/setup.py`), lalu daftarkan node Anda di dalam `entry_points` agar dapat dipanggil menggunakan perintah `ros2 run`:
```python
    entry_points={
        'console_scripts': [
            'state_listener = vtol_monitoring.state_listener_node:main',
        ],
    },
```

### Langkah 4: Kompilasi Workspace & Jalankan
Kembali ke root workspace (`~/workspace`) lalu jalankan `colcon build`:
```bash
cd ~/workspace
colcon build --packages-select vtol_monitoring
```
Setelah kompilasi selesai, jalankan source dan node Anda:
```bash
source install/setup.bash
ros2 run vtol_monitoring state_listener
```

---
