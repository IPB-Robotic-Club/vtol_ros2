# Setup SITL For Testing

SITL (Software in the Loop) adalah software untuk menjalankan simulasi UAV, dalam kasus ini adalah drone. Semua kode sebelum ditest harus lulus uji coba di simulasi!!. Sehingga menemukan SITL yang bisa merefleksi keadaan di dunia nyata dengan semirip mungkin menjadi kewajiban.

Sayangnya hingga saat ini SITL yang berhasil saya riset hanya mencapai SITL dalam bentuk 2D yaitu ardupilot-sitl. Simulator ini hanya terbatas, tidak bisa terlalu merefleksikan terlalu nyata keadaan didunia nyata.

> [!NOTE]
> Lanjutkan riset kedepannya untuk menemukan simulasi yang lebih akurat. Saran saya riset mengenai gazebo dan cara mensimulasikan sensor seperti rangefinder, lidar dsb pada gazebo tersebut.

## Ardupilot-SITL

### Daftar Isi
- [Requirement](#requirement)
- [Panduan Instalasi (WSL/Ubuntu 24.04)](#panduan-instalasi-wslubuntu-2404)
- [Menjalankan SITL](#menjalankan-sitl)
- [Setup Lokasi Awal (Setup Home)](#setup-lokasi-awal-setup-home)
- [Memasukkan Perintah pada SITL](#memasukkan-perintah-pada-sitl)
- [Uji Coba Terbang di SITL](#uji-coba-terbang-di-sitl)
- [Konfigurasi Parameter SITL](#konfigurasi-parameter-sitl)
  - [Parameter Angin (Wind)](#parameter-angin-wind)
  - [Simulasi Sensor Rangefinder](#simulasi-sensor-rangefinder)
  - [Simulasi Sensor Optical Flow](#simulasi-sensor-optical-flow)

---

### Requirement
SITL sebaiknya diinstall di linux seperti ubuntu, wsl, dsb. Soalnya tutorial ini dilakukan di ubuntu, lebih tepatnya di wsl. Defiasi antara os harusnya tidak terlalu beda, kecuali windows. Nih installasi officialnya: [ArduPilot SITL Official Installation](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html).

---

### Panduan Instalasi (WSL/Ubuntu 24.04)
Jalankan perintah berikut di terminal WSL/Ubuntu Anda:

1. **Update package manager:**
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```
2. **Instal Git:**
   ```bash
   sudo apt install git -y
   ```
3. **Download repositori ArduPilot (ke Home Directory):**
   ```bash
   cd ~
   git clone --recursive https://github.com/ArduPilot/ardupilot.git
   cd ardupilot
   ```
4. **Instal dependensi sistem (Proses ini memakan waktu cukup lama):**
   ```bash
   Tools/environment_install/install-prereqs-ubuntu.sh -y
   ```
5. **Reload Environment Profile:**
   ```bash
   source ~/.profile
   ```
6. **Build Target SITL:**
   ```bash
   # Konfigurasi board SITL
   ./waf configure --board sitl
   # Build firmware copter
   ./waf copter
   # Lakukan build & pembersihan jika diperlukan
   ./waf
   ./waf clean
   ```
7. **Verifikasi Simulator:**
   ```bash
   sim_vehicle.py -v ArduCopter --map --console
   ```

---

### Menjalankan SITL
Untuk menjalankan simulator copter standar:
```bash
sim_vehicle.py -v ArduCopter -f quad --map --console
```
Pelajari SITL lebih lanjut di [Dokumentasi Resmi ArduPilot SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html).

---

### Setup Lokasi Awal (Setup Home)
Untuk mengubah lokasi awal drone saat simulator dijalankan:

1. Buka berkas lokasi: `~/ardupilot/Tools/autotest/location.txt`.
2. Tambahkan baris baru di bagian paling bawah dengan format berikut:
   ```text
   NAMA_LOKASI=LATITUDE,LONGITUDE,KETINGGIAN,ARAH_DRONE
   ```
   *Keterangan:* `ARAH_DRONE` bernilai `0` untuk mengarah ke utara, dan `180` untuk selatan (derajat).
   
   *Contoh (Lokasi Arlab):*
   ```text
   Arlab=-6.554734,106.723382,0,160
   ```
3. Jalankan SITL dengan parameter lokasi (`-L` atau `-l` diikuti nama lokasi):
   ```bash
   sim_vehicle.py -v ArduCopter -f quad -L Arlab --map --console
   ```

---

### Memasukkan Perintah pada SITL
Gunakan terminal utama tempat Anda menjalankan SITL untuk memasukkan perintah MAVProxy. Panduan perintah selengkapnya dapat dilihat di [MAVProxy Commands Guide](https://ardupilot.org/mavproxy/sections/commands/index.html).

---

### Uji Coba Terbang di SITL
Berikut adalah perintah manual dasar pada terminal SITL untuk menguji terbang drone setinggi 2 meter:

1. **Ubah mode ke Guided:**
   ```text
   mode guided
   ```
2. **Arming drone:**
   ```text
   arm throttle
   ```
3. **Lepas landas (Takeoff) setinggi 2 meter:**
   ```text
   takeoff 2
   ```
4. **Verifikasi:**
   Perhatikan jendela **Console**. Cek nilai pada kolom **alt** (ketinggian) apakah sudah mencapai 2 meter.
5. **Mendarat (Land):**
   ```text
   mode land
   ```

---

### Konfigurasi Parameter SITL
Anda dapat menyimpan parameter saat ini ke dalam berkas:
```text
param save nama_file
```
Dan memuatnya kembali menggunakan:
```text
param load nama_file
```
*Catatan:* Jalankan simulasi di folder `~/ardupilot` agar berkas parameter tersimpan di tempat yang konsisten. Referensi lengkap parameter dapat dilihat pada [ArduPilot Parameter List](https://ardupilot.org/copter/docs/parameters.html).

#### Parameter Angin (Wind)
Untuk membuat simulasi lebih realistis mendekati kondisi asli di lapangan:
```text
# Mengatur arah angin (dalam derajat)
param set SIM_WIND_DIR <Arah_Angin>

# Mengatur kecepatan angin (dalam m/s)
param set SIM_WIND_SPD <Kecepatan_Angin>
```
*Contoh simulasi angin di Arlab (Arah 8° dengan kecepatan 2 m/s):*
```text
param set SIM_WIND_DIR 8
param set SIM_WIND_SPD 2
```
*Tips:* Anda dapat memantau prakiraan arah dan kecepatan angin di lapangan melalui situs seperti [Windfinder](https://www.windfinder.com/).

#### Sensor Rangefinder
Untuk mensimulasikan sensor rangefinder (LiDAR/Sonar):
```text
param set SIM_SONAR_SCALE 10
param set RNGFND1_TYPE 100
param set RNGFND1_SCALING 10
param set RNGFND1_PIN 0
param set RNGFND1_MAX 50
param set RNGFND1_MIN 0
```
Setelah mengubah parameter di atas, lakukan reboot pada SITL:
```text
reboot
```
Untuk memantau data sensor secara grafis:
```text
module load graph
graph RANGEFINDER.distance
```
*Penjelasan:* SITL mendukung hingga 9 sensor rangefinder (`RNGFND1` sampai `RNGFND9`). Set `RNGFND1_TYPE` ke `0` jika ingin menonaktifkannya kembali.

#### Sensor Optical Flow
Pastikan sensor rangefinder telah menyala terlebih dahulu, kemudian aktifkan sensor optical flow dengan perintah:
```text
param set SIM_FLOW_ENABLE 1
param set FLOW_TYPE 10
reboot
```
Untuk mengecek grafis pergerakan sensor:
```text
module load graph
graph OPTICAL_FLOW.flow_comp_m_x OPTICAL_FLOW.flow_comp_m_y
```
*Sensor lainnya dapat dipelajari pada dokumentasi resmi ArduPilot.*
