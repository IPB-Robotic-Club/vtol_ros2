# Panduan Konfigurasi & Mekanisme Failsafe (`vtol_control`)

Dokumen ini menjelaskan arsitektur sistem pemantauan dan pengelolaan koneksi pada package `vtol_control`.

---

## 1. Manajemen Konfigurasi Koneksi (`fcu_url.txt`)

Untuk mempermudah perpindahan antara koneksi **Simulasi (TCP)** dan **Drone Fisik (Serial)** tanpa perlu mengganti perintah di terminal, MAVROS membaca konfigurasi dari sebuah file text.

*   **Lokasi File Config:** **[fcu_url.txt](../workspace/src/vtol_control/config/fcu_url.txt)**
*   **Cara Penggunaan:** Cukup edit file tersebut dan pilih baris yang ingin diaktifkan (hilangkan tanda `#` pada baris tersebut).
    *   **TCP Mode (SITL):** `tcp://127.0.0.1:5762`
    *   **Serial Mode (Pixhawk fisik):** `/dev/ttyACM0:921600`
*   **Mekanisme Launch:** Saat Anda memanggil `ros2 launch`, file launch **[vtol_core.launch.py](../workspace/src/vtol_control/launch/vtol_core.launch.py)** akan membaca file `fcu_url.txt` secara otomatis di latar belakang, mem-parsing isinya, dan langsung menyambungkan MAVROS menggunakan alamat tersebut.

---

## 2. Pemantau Status Core (`vtol_core.py`)

Node **[vtol_core.py](../workspace/src/vtol_control/vtol_control/vtol_core.py)** berfungsi sebagai *core monitor* telemetri. Node ini disederhanakan agar berjalan secara ringan:

*   **Mencatat Koneksi Autopilot:** Menampilkan informasi ketika koneksi serial/TCP MAVLink berhasil terhubung atau terputus.
*   **Mencatat Perubahan Mode Terbang:** Memantau dan mencetak perubahan mode (misal dari `STABILIZE` ke `GUIDED` atau `LAND`).
*   **Mencatat Status Arming:** Melaporkan status arming drone (ARMED / DISARMED).
*   **Request Stream Rate:** Mengirim request ke autopilot untuk memulai publikasi telemetri (misal sensor GPS, posisi lokal, status baterai) pada kecepatan 10Hz.

> [!IMPORTANT]
> **Failsafe & Proteksi Keselamatan:**
> Node `vtol_core` ini tidak lagi melakukan intervensi perintah secara aktif (seperti mengirimkan perintah `LAND` otomatis via software watchdog). Disarankan untuk selalu mengonfigurasi failsafe bawaan pada firmware autopilot (seperti *GCS Connection Failsafe*, *Radio Failsafe*, atau *Battery Failsafe* pada ArduPilot/PX4) agar penanganan darurat dapat dieksekusi secara instan dan aman langsung dari Flight Controller.

---

## 3. Cara Menjalankan Program (Execution Modes)

Setelah melakukan perubahan alamat koneksi di `fcu_url.txt`, lakukan kompilasi workspace terlebih dahulu di terminal kontainer Docker:
```bash
cd /home/pilot/workspace && colcon build --packages-select vtol_control
source install/setup.bash
```

Anda dapat menjalankan sistem ini dalam 3 cara berbeda tergantung kebutuhan pengembangan Anda:

### Opsi A: Mode Senyap Latar Belakang (Direkomendasikan untuk Misi Otomatis)
Menjalankan MAVROS dan node pengaman `vtol_core` secara senyap di latar belakang. Terminal Anda akan tetap bersih dari log sehingga Anda bisa leluasa menjalankan script misi (seperti Misi A atau Misi B) di terminal lain.
```bash
ros2 launch vtol_control vtol_core.launch.py
```

### Opsi B: Satu Terminal Tunggal (Core + HUD Printer)
Menjalankan MAVROS, node pengaman `vtol_core`, sekaligus meluncurkan node visualizer HUD `vehicle_status` secara bersamaan dalam satu perintah tunggal.
```bash
ros2 launch vtol_control vtol_core.launch.py show_hud:=true
```

### Opsi C: HUD Status Mandiri (On-Demand)
Jika Anda menggunakan Opsi A (menjalankan core secara senyap), Anda dapat sewaktu-waktu membuka visualizer HUD status drone di terminal terpisah tanpa memengaruhi jalannya core node utama.
```bash
ros2 run vtol_control vehicle_status
```
*Gunakan `Ctrl+C` untuk menutup viewer HUD kapan saja secara aman tanpa mengganggu sistem pengaman utama.*
