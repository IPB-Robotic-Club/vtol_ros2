# Walkthrough - Refaktor Menyeluruh README.md

Berkas `README.md` telah direfaktor secara menyeluruh menjadi dokumen panduan developer yang bersih, ringkas, dan fokus pada pengembangan harian (to-the-point).

## Ringkasan Perubahan

### 1. Perampingan Dokumen `README.md`
- **[README.md](../README.md)**:
  - **Penghapusan Materi Generik:** Menghapus bagian penjelasan teoretis konsep dasar (WSL2, WSLg, Docker, ROS2, MAVLink), tabel spesifikasi hardware minimum, serta panduan instalasi WSL2 dan Docker Engine langkah-demi-langkah (developer diasumsikan sudah atau dapat menginstalnya secara mandiri).
  - **Penyederhanaan Quick Start:** Menggabungkan prasyarat sistem dengan instruksi instan kloning, pembangunan kontainer, dan akses kontainer (`docker compose build` & `docker compose up` & `docker exec`).
  - **Restrukturisasi Bagian Kontrol & Kompilasi:** Cheat sheet perintah ROS 2/MAVROS, kompilasi workspace (`colcon build`), pemantauan telemetri, dan pembunuhan proses node (`pkill`) disajikan secara padat dalam satu tabel/daftar perintah.
  - **Visualisasi Volume Mount & Struktur Script:** Menyimpan ringkasan alur volume mount serta penjelasan peran masing-masing skrip di dalam package `vtol_control`.

---

## Verifikasi Build
Kompilasi ulang package dilakukan di dalam kontainer `vtol_dev` untuk menjamin integritas program tetap terjaga:
```bash
cd /home/pilot/workspace && colcon build --packages-select vtol_control
```
*Hasil: Build sukses tanpa kendala.*
