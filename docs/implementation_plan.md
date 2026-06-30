# Rencana Perbaikan Dokumentasi (Documentation Cleanup & Refactoring) - Refaktor README

Rencana ini bertujuan untuk menyederhanakan `README.md` secara menyeluruh dengan menghapus materi/panduan generik (seperti tutorial instalasi WSL2/Docker langkah demi langkah, spesifikasi hardware, dan penjelasan teori konsep dasar) dan memfokuskannya menjadi dokumentasi teknis developer yang ringkas, bersih, dan praktis.

## Proposed Changes

### Dokumentasi Repositori (Workspace Root)

---

#### [MODIFY] [README.md](../README.md)
Refaktor `README.md` agar memiliki struktur yang lebih ramping sebagai berikut:
1. **Pendahuluan:** Deskripsi singkat proyek pengembangan VTOL dengan ROS2 & Docker.
2. **Prasyarat & Setup Cepat (Quick Start):**
   - Hapus detail panduan instalasi WSL2/Docker yang bersifat generik (berisi perintah `wsl --install`, registrasi gpg key, usermod, dll.). Developer cukup diarahkan untuk menginstal WSL2 & Docker secara mandiri.
   - Satukan langkah build, up, dan exec kontainer menjadi satu alur instalasi/eksekusi cepat.
3. **Alur Kerja Harian (Daily Workflow):** Alur ringkas menyalakan docker, pull update terbaru, masuk ke container, dan mematikan docker.
4. **Koneksi SITL & Mission Planner:** Langkah penyambungan MAVROS kontainer dengan Mission Planner di Windows Host.
5. **Cheat Sheet Perintah Penting (ROS2 & MAVROS):** Rangkuman perintah kompilasi (`colcon build`), opsi menjalankan sistem (launch files), pemantauan topik telemetri, dan proses pembersihan node (`pkill`).
6. **Struktur Workspace & Penjelasan Script:**
   - Penjelasan ringkas mekanisme volume mount.
   - Daftar skrip kontrol riil pada package `vtol_control` beserta perannya.
7. **Deployment ke SBC (Raspberry Pi):** Tautan langsung ke [docs/sbc_deployment.md](docs/sbc_deployment.md).

---

## Verification Plan

### Manual Verification
- Membaca berkas `README.md` baru untuk memastikan isinya sangat ramping, to-the-point, dan tidak mengandung panduan generik/teoretis yang tidak penting bagi pengembangan sehari-hari.
