# Panduan Sistem Kerja & Keamanan Drone Autonomous (ROS2 - MAVROS - Autopilot)

Dokumen ini menjelaskan arsitektur sistem, aliran data, dan protokol keselamatan (failsafe) pada repositori pengembangan VTOL Autonomous ini. Memahami cara kerja sistem ini sangat penting untuk mencegah kegagalan terbang (*flyaway* atau *crash*) baik saat simulasi maupun pada drone fisik.

---

## 1. Arsitektur Komunikasi Sistem

Sistem ini terdiri dari tiga komponen utama yang bekerja secara berlapis:

1. **Flight Controller Unit (FCU) / Autopilot**: Otak fisik drone (seperti Pixhawk yang menjalankan firmware ArduPilot/PX4). FCU menangani stabilisasi motor secara langsung, pembacaan sensor dasar (IMU, GPS, Kompas), dan kalkulasi navigasi level rendah.
2. **MAVROS**: Jembatan (*bridge*) middleware yang menerjemahkan data dari protokol **MAVLink** (bahasa komunikasi autopilot) menjadi **Topik dan Service ROS 2**, dan sebaliknya.
3. **ROS 2 Node (Program Anda)**: Program logika cerdas (seperti pemantau status atau modul AI) yang berjalan di komputer pendamping (*companion computer* seperti Raspberry Pi di drone fisik, atau WSL2 di laptop).

---

## 2. Aliran Data Telemetri & Kontrol

### A. Aliran Telemetri (Dari Drone ke ROS 2)
1. Autopilot membaca sensor fisik $\rightarrow$ memaketkan data ke dalam format MAVLink (contoh: pesan `#1` `SYS_STATUS` atau `#24` `GPS_RAW_INT`).
2. MAVROS menerima paket MAVLink tersebut via kabel Serial (drone fisik) atau koneksi TCP/UDP (simulasi).
3. MAVROS menerjemahkan paket tersebut dan mempublikasikannya ke dalam topik ROS 2 (contoh: `/mavros/state`, `/mavros/battery`, `/mavros/global_position/global`).
4. Node ROS 2 Anda berlangganan (*subscribe*) ke topik-topik tersebut untuk memantau status drone secara real-time.

### B. Aliran Perintah (Dari ROS 2 ke Drone)
1. Node ROS 2 Anda mempublikasikan perintah target (misal: target koordinat ke topik `/mavros/setpoint_position/local`).
2. MAVROS menerjemahkan perintah tersebut ke dalam format pesan MAVLink `SET_POSITION_TARGET_LOCAL_NED`.
3. Autopilot menerima target koordinat tersebut, memprosesnya melalui kontroler PID internal, dan mengirimkan sinyal PWM ke ESC motor untuk menggerakkan drone fisik menuju target.

---

## 3. Protokol Keselamatan & Failsafe (Penting untuk Keamanan!)

Mengontrol drone seberat beberapa kilogram dengan baling-baling berputar tinggi sangatlah berbahaya. Oleh karena itu, sistem ini dirancang dengan mekanisme pengaman (*failsafe*) berikut:

### A. Failsafe Kehilangan Koneksi Kontrol (Setpoint Timeout Failsafe)
* **Aturan Utama MAVROS**: Autopilot membutuhkan aliran perintah target (*setpoint*) secara terus-menerus.
* **Cara Kerja**: Sebelum beralih ke mode kontrol autonomous (`GUIDED` / `OFFBOARD`), program ROS 2 **harus mengirimkan setpoint terlebih dahulu** dengan frekuensi minimal 2 Hz (direkomendasikan **20 Hz**).
* **Failsafe**: Jika program ROS 2 Anda mengalami *crash*, hang, atau kabel data terputus selama $\ge 0.5$ detik saat drone terbang autonomous, autopilot akan mendeteksi *Setpoint Timeout*. Autopilot secara otomatis akan mengambil alih kendali dan mengganti mode penerbangan menjadi **`LAND`** (mendarat di tempat secara otomatis) atau **`RTL`** (kembali ke titik awal lepas landas secara otomatis) untuk mencegah drone terbang tanpa kendali (*flyaway*).

### B. Hak Pengambilalihan Kendali Manual (RC Override / GCS Kill Switch)
* **Prioritas Utama**: Pilot manusia (dengan Remote Control fisik) memegang prioritas kendali tertinggi di atas program autonomous ROS 2.
* **RC Override**: Jika terjadi kegagalan logika pada program Python Anda (misal drone terbang ke arah yang salah), pilot fisik cukup mengubah switch mode pada RC (misal dari mode `GUIDED` ke mode manual `ALT_HOLD` atau `LAND`). Pindahnya switch fisik ini akan langsung mematikan kendali autonomous ROS 2 seketika itu juga.
* **GCS Override**: Di Mission Planner simulasi, Anda dapat mengklik tombol **`LAND`** atau **`RTL`** kapan saja untuk membatalkan kontrol ROS 2.

### C. Failsafe Geofence & Batas Ketinggian
* Autopilot dikonfigurasi dengan batas wilayah terbang virtual (*Geofence*) baik secara radius horizontal (misal 50 meter) maupun ketinggian vertikal (misal 10 meter).
* Jika program autonomous ROS 2 Anda memerintahkan drone melintasi batas geofence ini, autopilot akan mengabaikan perintah tersebut dan langsung mengeksekusi prosedur pendaratan darurat.

---

## 4. Cara Kerja Node `vtol_core` (Core Monitor)

Node pemantau inti (**[vtol_core.py](../workspace/src/vtol_control/vtol_control/vtol_core.py)**) berjalan secara diam (*silent*) di latar belakang dan memantau status dasar wahana:

```
[Start Node]
    │
    ▼
[Mencoba Berlangganan ke Topik MAVROS]
    └── /mavros/state  (Mendapatkan info Koneksi, Arming, Mode Terbang)
    │
    ▼
[Menerima Data Telemetri]
    ├── 1. Log Perubahan Koneksi Autopilot (Mencatat status saat tersambung / terputus)
    ├── 2. Log Perubahan Status Arming (Mencatat status ARMED / DISARMED)
    ├── 3. Log Perubahan Mode Terbang (Mencatat perubahan ke mode GUIDED, LAND, dll.)
    └── 4. Request Telemetry Stream Rate (Sekali saja saat pertama terhubung, meminta data telemetri 10Hz)
```

Variabel status (koneksi, arming, mode) dipantau secara langsung melalui callback event di dalam kelas `VtolCore` agar memudahkan pemantauan dasar tanpa membebani sistem dengan pemrosesan failsafe software yang berlebihan di tingkat pendamping (companion). Failsafe kritis diserahkan kepada konfigurasi firmware Flight Controller.


