# Panduan Konfigurasi & Mekanisme Failsafe Robust (`vtol_control`)

Dokumen ini menjelaskan arsitektur baru sistem pemantauan dan pengaman (*failsafe*) core pada package `vtol_control`.

---

## 1. Manajemen Konfigurasi Koneksi (`fcu_url.txt`)

Untuk mempermudah perpindahan antara koneksi **Simulasi (TCP)** dan **Drone Fisik (Serial)** tanpa perlu mengganti perintah di terminal, MAVROS kini membaca konfigurasi dari sebuah file text.

*   **Lokasi File Config:** **[fcu_url.txt](file:///wsl.localhost/Ubuntu-24.04/home/qois51/vtol-dev/workspace/src/vtol_control/config/fcu_url.txt)**
*   **Cara Penggunaan:** Cukup edit file tersebut dan pilih baris yang ingin diaktifkan (hilangkan tanda `#` pada baris tersebut).
    *   **TCP Mode (SITL):** `tcp://127.0.0.1:5762`
    *   **Serial Mode (Pixhawk fisik):** `/dev/ttyACM0:921600`
*   **Mekanisme Launch:** Saat Anda memanggil `ros2 launch`, file launch **[vtol_core.launch.py](file:///wsl.localhost/Ubuntu-24.04/home/qois51/vtol-dev/workspace/src/vtol_control/launch/vtol_core.launch.py)** akan membaca file `fcu_url.txt` secara otomatis di latar belakang, mem-parsing isinya, dan langsung menyambungkan MAVROS menggunakan alamat tersebut.

---

## 2. Mekanisme Failsafe Core (`vtol_core.py`)

Seluruh logika pengaman (*failsafe*) dan jembatan pemantau telemetri kini disatukan secara permanen di dalam satu file inti: **[vtol_core.py](file:///wsl.localhost/Ubuntu-24.04/home/qois51/vtol-dev/workspace/src/vtol_control/vtol_control/vtol_core.py)**. 

Program ini memantau dua tingkat kegagalan koneksi secara otomatis pada frekuensi **10 Hz**:

### A. Failsafe 1: MAVROS Heartbeat Watchdog (Komputer Pendamping Mati/Hang)
*   **Masalah:** Program MAVROS mati, hang, atau tidak merespons di komputer pendamping (Raspberry Pi/WSL).
*   **Cara Kerja:** Node `vtol_core` mencatat waktu kedatangan pesan `/mavros/state` terakhir. **Pemeriksaan ini baru aktif setelah koneksi pertama dengan autopilot terjalin** (`connection_established = True`) untuk menghindari alarm palsu saat proses inisialisasi awal MAVROS yang lambat. Jika waktu jeda sejak pesan terakhir melebihi **3,0 detik**:
    *   Sistem mencatat status kehilangan heartbeat MAVROS secara internal.
    *   **Jika Drone sedang armed (terbang):** Node akan langsung mengirimkan perintah darurat **`LAND`** secara asinkron ke autopilot.

### B. Failsafe 2: Autopilot Connection Watchdog (Kabel Data Terputus/Jalur Serial Mati)
*   **Masalah:** MAVROS tetap hidup, namun hubungan komunikasi MAVLink antara MAVROS dengan Pixhawk terputus di tengah penerbangan (kabel USB/Serial longgar atau copot).
*   **Cara Kerja:** Node mendeteksi jika parameter `connected` pada topik `/mavros/state` berubah menjadi `False` saat status drone masih **Armed** (sedang terbang).
    *   Node mencatat status kegagalan koneksi ke autopilot secara internal dan memicu log error.
    *   Node langsung mengeksekusi service panggilan darurat **`LAND`** ke Pixhawk untuk segera mendaratkan drone di tempat demi keamanan.

---

## 3. Deteksi Override Pilot (LAND Mode Signal)

Salah satu kendala dalam kendali otomatis adalah ketika pilot manusia ingin mengambil alih drone di tengah misi otomatis dengan mengganti mode terbang menjadi **`LAND`** (baik melalui switch remote control maupun Mission Planner GCS).

*   **Masalah Lama:** Program autonomous luar terkadang tetap mengirimkan koordinat target terbang (setpoint) meskipun pilot telah memindahkan mode ke `LAND`, menyebabkan drone bergetar atau menolak mendarat karena bertabrakan perintah.
*   **Solusi Baru di `vtol_core`:**
    *   Node `vtol_core` secara aktif memantau perubahan mode penerbangan drone.
    *   Jika terdeteksi mode terbang berubah menjadi **`LAND`** (baik diperintahkan oleh program sendiri, oleh GCS, atau oleh switch remote control pilot), node akan mengaktifkan bendera keselamatan `self.pilot_override_active = True` dan `self.mission_completed = True`.
    *   Sinyal ini menandakan kepada seluruh sistem kontrol autonomous luar bahwa **misi telah selesai / dibatalkan**, dan pengiriman setpoint koordinat harus segera dihentikan total agar proses pendaratan berjalan mulus tanpa intervensi data luar.

---

## 4. Cara Menjalankan Program (Execution Modes)

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


