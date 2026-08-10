# Panduan Analisis Log Penerbangan & Transfer Data

Dokumen ini menjelaskan langkah-langkah untuk menyalin file log hasil penerbangan dari komputer pendamping drone (**Raspberry Pi / SBC**) ke komputer lokal (**PC/WSL**), cara menggunakan skrip analisis data [workspace/analyze.py](../workspace/analyze.py) secara mandiri atau dengan bantuan AI (Antigravity), serta prosedur pengujian *vision* di darat (*dry run*).

---

## Daftar Isi
1. [Jenis File Log yang Dihasilkan](#1-jenis-file-log-yang-dihasilkan)
2. [Panduan Transfer Data dari Raspberry Pi ke PC Lokal](#2-panduan-transfer-data-dari-raspberry-pi-ke-pc-lokal)
3. [Perintah Analisis Data (analyze.py)](#3-perintah-analisis-data-analyzepy)
4. [Cara Menggunakan AI untuk Analisis Otomatis](#4-cara-menggunakan-ai-untuk-analisis-otomatis)
5. [Prosedur Uji Darat Vision (Dry Run tanpa Terbang)](#5-prosedur-uji-darat-vision-dry-run-tanpa-terbang)

---

## 1. Jenis File Log yang Dihasilkan

Setiap kali misi centering atau deteksi ArUco dijalankan di Raspberry Pi, sistem akan menghasilkan 3 file log di dalam folder `~/vtol_ros2/workspace/`:

* **`centering_data.csv`**: Mencatat telemetri kuantitatif per loop (error norm_x, norm_y, distance, altitude, yaw_error, komponen PID P/I/D, dan output channel RC1/RC2/RC3/RC4).
* **`mission_centering.log`**: Log kronologis teks yang mencatat pergeseran fase state machine (`YAW_ALIGN`, `CENTERING`), event `YAW_DRIFT`, peringatan *marker lost*, dan pemicu *failsafe*.
* **`aruco_vision.log`**: Log performa pengolahan citra per frame (status deteksi, jumlah *rejected candidates*, koordinat piksel, dan ID marker terdeteksi).

---

## 2. Panduan Transfer Data dari Raspberry Pi ke PC Lokal

Jalankan perintah ini di **Terminal PC/WSL lokal** Anda (bukan di dalam Docker dan bukan di dalam SSH Raspi):

### Menggunakan SCP (Sangat Direkomendasikan)
Gunakan perintah `scp` untuk menyalin seluruh file `.csv` dan `.log` dari Raspberry Pi ke workspace PC lokal:

```bash
# Salin file data CSV:
scp vtol@vtol.local:~/vtol_ros2/workspace/*.csv ~/vtol-dev/workspace/

# Salin file log teks & vision:
scp vtol@vtol.local:~/vtol_ros2/workspace/*.log ~/vtol-dev/workspace/
```

*Catatan:*
- Jika username atau IP Raspberry Pi berbeda, sesuaikan `vtol@vtol.local` dengan `username@IP_ADDRESS` Anda (contoh: `pi@192.168.1.100`).

### Menggunakan Rsync (Alternatif)
```bash
rsync -avz vtol@vtol.local:~/vtol_ros2/workspace/*.csv ~/vtol-dev/workspace/
rsync -avz vtol@vtol.local:~/vtol_ros2/workspace/*.log ~/vtol-dev/workspace/
```

---

## 3. Perintah Analisis Data (`analyze.py`)

Setelah file tersalin ke `~/vtol-dev/workspace/`, jalankan skrip [analyze.py](../workspace/analyze.py) dari root direktori proyek di PC lokal Anda.

### A. Menjalankan Seluruh Analisis Sekaligus
```bash
python3 workspace/analyze.py --mode all
```

### B. Analisis Korelasi Pergerakan Drone vs Input RC
Memeriksa tingkat korelasi antara error marker dengan perintah kemudi RC (Pitch, Roll, Yaw):
```bash
python3 workspace/analyze.py --mode correlation
```

### C. Analisis Performa Vision & Frame Loss
Menghitung persentase frame terdeteksi vs hilang, rata-rata *rejected candidates*, dan durasi marker hilang terlama:
```bash
python3 workspace/analyze.py --mode vision
```

### D. Analisis State Machine & PID Altitude Misi
Menampilkan konfigurasi parameter PID, timeline event misi, error rata-rata, dan statistik ketinggian:
```bash
python3 workspace/analyze.py --mode mission
```

### E. Inspeksi Detail Data Pitch, Roll, dan Yaw
Melihat baris data sampel pada rentang indeks tertentu:
```bash
# Inspeksi Pitch (Err_Y vs RC Pitch):
python3 workspace/analyze.py --mode pitch --start 100 --end 150

# Inspeksi Roll (Err_X vs RC Roll):
python3 workspace/analyze.py --mode roll --start 100 --end 150

# Inspeksi Yaw (Yaw Error vs RC Yaw):
python3 workspace/analyze.py --mode yaw --start 100 --end 150
```

### F. Menggunakan Path Custom File Log
Jika file log disimpan di nama/lokasi berbeda:
```bash
python3 workspace/analyze.py --mode all --csv /path/to/data.csv --vision-log /path/to/vision.log --mission-log /path/to/mission.log
```

---

## 4. Cara Menggunakan AI untuk Analisis Otomatis

Setelah Anda menyalin file log ke folder `~/vtol-dev/workspace/` lokal, Anda tidak perlu membaca baris log manual. Cukup beri instruksi kepada AI (Antigravity) melalui obrolan:

> *"Saya sudah transfer file log terbaru dari Raspi ke workspace lokal. Tolong analisis apa penyebab masalah penerbangan tadi."*

AI akan secara otomatis:
1. Menjalankan skrip `workspace/analyze.py`.
2. Membaca statistik korelasi, timeline kesalahan, dan frame loss.
3. Menyajikan diagnosis masalah empiris beserta solusi perbaikan pada kode/konfigurasi secara langsung.

---

## 5. Prosedur Uji Darat Vision (Dry Run tanpa Terbang)

Metode pengujian ini digunakan untuk menguji stabilitas inferensi deteksi ArUco dan tingkat *frame loss* di darat (*bench test*) tanpa harus menerbangkan drone (*disarmed* / *no propellers*).

### Metode 1: Handheld Shake & Motion Test (Uji Gerak Tangan)
1. **Jalankan Streamer Kamera di Host Raspi:**
   ```bash
   python3 pi5_streamer.py
   ```
2. **Jalankan Node Vision di Kontainer Docker Raspi:**
   ```bash
   docker exec -it vtol_sbc bash
   ros2 run vtol_vision aruco_receiver
   ```
3. **Lakukan Pengujian Fisik:**
   - Pegang marker ArUco atau gerakkan kamera Raspi di atas marker pada jarak 1 – 2 meter.
   - Goyangkan kamera / marker dengan cepat (simulasi getaran drone saat melayang) dan miringkan sudutnya.

### Metode 2: Pantau Web UI Real-Time
Buka browser di laptop yang tersambung ke WiFi/Jaringan Raspberry Pi:
- **Web Stream Video Live:** `http://vtol.local:8086` (atau `http://<ip_raspi>:8086`)
- **Status JSON Live:** `http://vtol.local:8086/status`

Di tampilan Web Stream, Anda bisa melihat garis kotak hijau pada marker secara real-time. Jika kotak hijau tetap menempel tanpa kedip-kedip saat kamera digoyangkan, deteksi sudah stabil.

### Metode 3: Pantau Log Live via Terminal
Buka terminal baru di Raspberry Pi dan jalankan perintah `tail` untuk melihat status deteksi per frame secara real-time:
```bash
tail -f ~/vtol_ros2/workspace/aruco_vision.log
```
Yang perlu diperhatikan:
- **Kolom DETECTED:** Harus bernilai `1` (terdeteksi).
- **Kolom REJECTED:** Perhatikan apakah angkanya turun jauh dari 216 ke bawah ~30–50 per frame.

### Metode 4: Analisis Persentase Frame Loss dengan `analyze.py`
Setelah menggoyangkan kamera selama 1–2 menit, matikan `aruco_receiver` (`Ctrl+C`), lalu jalankan skrip analisis:
```bash
python3 workspace/analyze.py --mode vision
```

**Target Hasil Uji Darat yang Sukses:**
- **Detected frames:** $> 70\% - 95\%$ (naik drastis dari sebelumnya 22%).
- **Average rejected markers/frame:** turun signifikan menjadi $< 40 - 50$ kandidat.
- **Longest continuous loss:** $< 100$ frame (di bawah 3 detik, aman dari timeout failsafe 5 detik).
