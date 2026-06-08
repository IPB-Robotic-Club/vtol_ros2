# Panduan Deployment ke Single Board Computer (SBC) - Raspberry Pi (ARM64)

Dokumen ini menjelaskan langkah-langkah lengkap untuk melakukan setup awal, membangun image (*native build*), dan menjalankan kontainer Docker VTOL (`sbc`) secara langsung pada perangkat Raspberry Pi.

---

## Daftar Isi
- [BAGIAN I: SETUP AWAL (Hanya Sekali Setup)](#bagian-i-setup-awal-hanya-sekali-setup)
  - [1. Generate SSH Key & Hubungkan ke GitHub](#1-generate-ssh-key--hubungkan-ke-github)
  - [2. Clone Repositori menggunakan SSH](#2-clone-repositori-menggunakan-ssh)
  - [3. Build Image secara Native di Raspberry Pi](#3-build-image-secara-native-di-raspberry-pi)
- [BAGIAN II: ALUR HARIAN / SETIAP UPDATE (Daily Workflow)](#bagian-ii-alur-harian--setiap-update-daily-workflow)
  - [1. Tarik Pembaruan Kode (Git Pull)](#1-tarik-pembaruan-kode-git-pull)
  - [2. Bangun Ulang Image (Jika Ada Perubahan Dockerfile)](#2-bangun-ulang-image-jika-ada-perubahan-dockerfile)
  - [3. Jalankan Kontainer](#3-jalankan-kontainer)
- [BAGIAN III: PENGUJIAN KONEKSI PIXHAWK (MAVROS)](#bagian-iii-pengujian-koneksi-pixhawk-mavros)
  - [1. Jalankan Node MAVROS](#1-jalankan-node-mavros)
  - [2. Verifikasi Konektivitas Pixhawk](#2-verifikasi-konektivitas-pixhawk)

---

## BAGIAN I: SETUP AWAL (Hanya Sekali Setup)

Jalankan langkah-langkah ini saat pertama kali menyiapkan Raspberry Pi Anda.

### 1. Generate SSH Key & Hubungkan ke GitHub
Karena repositori proyek ini bersifat **Private**, Raspberry Pi memerlukan akses autentikasi menggunakan SSH Key untuk melakukan clone.

1. **Generate SSH Key baru di Raspberry Pi:**
   Buka terminal Raspberry Pi Anda, lalu jalankan:
   ```bash
   ssh-keygen -t ed25519 -C "email_anda@example.com"
   ```
   *Tekan Enter terus menerus untuk menyetujui lokasi penyimpanan default dan tanpa passphrase.*

2. **Salin Public Key yang Dihasilkan:**
   Tampilkan isi file public key Anda dengan:
   ```bash
   cat ~/.ssh/id_ed25519.pub
   ```
   *Blok dan salin seluruh teks yang muncul (dimulai dengan `ssh-ed25519` sampai email Anda).*

3. **Masukkan Public Key ke Akun GitHub:**
   * Masuk ke GitHub Anda, lalu buka **Settings** > **SSH and GPG keys**.
   * Klik tombol **New SSH key**.
   * Beri judul (misal: `Raspberry Pi VTOL Drone`).
   * Tempelkan teks public key yang tadi disalin ke dalam kotak **Key**.
   * Klik **Add SSH key**.

4. **Uji Koneksi SSH ke GitHub:**
   ```bash
   ssh -T git@github.com
   ```
   *Jika muncul konfirmasi sidik jari (fingerprint), ketik `yes`. Jika sukses, akan muncul pesan seperti: "Hi username! You've successfully authenticated...".*

### 2. Clone Repositori menggunakan SSH
Setelah autentikasi SSH aktif, Anda dapat melakukan clone repositori private ke folder home (`~/vtol-dev`):
```bash
git clone git@github.com:IPB-Robotic-Club/vtol_ros2.git ~/vtol-dev
cd ~/vtol-dev
```

### 3. Build Image secara Native di Raspberry Pi
Mengingat image dijalankan langsung di Raspberry Pi, lakukan proses build secara lokal pada Pi untuk target `sbc`:
```bash
docker compose build sbc
```
*(Catatan: Proses ini memerlukan koneksi internet aktif karena Docker akan mengunduh base image `ros:jazzy-ros-base` dan melakukan instalasi library robotika yang dibutuhkan. Proses ini memakan waktu sekitar 10-20 menit pada Raspberry Pi 4/5).*

---

## BAGIAN II: ALUR HARIAN / SETIAP UPDATE (Daily Workflow)

Ikuti alur ini ketika Anda melakukan coding di PC/Laptop dan ingin menerapkannya di drone (SBC).

### 1. Tarik Pembaruan Kode (Git Pull)
Sebelum menjalankan program di drone, selalu pastikan kode workspace Anda di Raspberry Pi sinkron dengan perubahan terbaru dari repositori:
```bash
cd ~/vtol-dev
git pull origin main
```

### 2. Bangun Ulang Image (Jika Ada Perubahan Dockerfile)
*Hanya diperlukan* jika Anda memodifikasi file `Dockerfile` atau mengubah daftar dependensi sistem. Jika hanya script python di workspace yang berubah, langkah ini bisa dilewati:
```bash
docker compose build sbc
```

### 3. Jalankan Kontainer
Nyalakan kontainer target `sbc` agar berjalan di latar belakang:
```bash
docker compose up -d sbc
```
*(Penting: pastikan tidak menggunakan flag `-t` agar syntax tidak error).*

---

## BAGIAN III: PENGUJIAN KONEKSI PIXHAWK (MAVROS)

Setelah kontainer berjalan, ikuti langkah berikut untuk menguji jembatan komunikasi antara komputer pendamping (Raspberry Pi) dengan Autopilot (Pixhawk) melalui serial port.

### 1. Jalankan Node MAVROS
1. **Masuk ke dalam kontainer yang sedang berjalan:**
   ```bash
   docker exec -it vtol_sbc bash
   ```

2. **Jalankan Node MAVROS dengan parameter FCU URL:**
   Hubungkan Pixhawk ke port USB Raspberry Pi (biasanya terbaca sebagai `/dev/ttyACM0`) dengan baudrate default Pixhawk `921600` (atau sesuaikan dengan port serial yang Anda gunakan):
   ```bash
   ros2 run mavros mavros_node --ros-args -p fcu_url:="/dev/ttyACM0:921600"
   ```

### 2. Verifikasi Konektivitas Pixhawk
1. **Buka Terminal Raspberry Pi Baru** (biarkan node MAVROS tetap berjalan di terminal pertama).
2. **Masuk kembali ke dalam kontainer:**
   ```bash
   docker exec -it vtol_sbc bash
   ```
3. **Cek State Koneksi MAVROS:**
   Jalankan perintah berikut untuk melihat status koneksi jembatan data MAVLink:
   ```bash
   ros2 topic echo /mavros/state
   ```

4. **Verifikasi Output:**
   Perhatikan bagian baris output log terminal Anda:
   ```text
   header:
     stamp:
       sec: 1718223948
       nanosec: 450912000
     frame_id: ''
   connected: True    <--- PASTIKAN BENILAI TRUE!
   armed: False
   guided: False
   mode: STABILIZE
   system_status: 3
   ```
   *Jika `connected` bernilai `True`, selamat! Program kontrol ROS2 Anda di Raspberry Pi sudah tersambung sepenuhnya dengan Autopilot Pixhawk.*
