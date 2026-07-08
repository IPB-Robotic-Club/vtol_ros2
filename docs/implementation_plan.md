# Implementation Plan - Konfigurasi MAVLink FCU URL Berbasis YAML

Mengubah mekanisme pemilihan/konfigurasi FCU URL MAVLink dari berkas teks mentah (`fcu_url.txt`) menjadi berkas konfigurasi YAML (`vtol_config.yaml`) yang lebih terstruktur dan mudah dikelola. Berkas konfigurasi baru ini akan dibaca secara dinamis oleh manajemen peluncuran (`vtol_core.launch.py` dan `test_arm.launch.py`) menggunakan modul pembaca konfigurasi terpusat.

## User Review Required

> [!IMPORTANT]
> - Berkas lama `fcu_url.txt` akan dihapus dan digantikan oleh `vtol_config.yaml`.
> - Pengguna dapat dengan mudah mengganti profil koneksi aktif (`tcp` vs `serial`) hanya dengan mengubah nilai `active_profile` pada berkas `vtol_config.yaml` tanpa perlu melakukan *comment*/*uncomment* baris kode manual.

## Proposed Changes

### Konfigurasi VTOL Control

---

#### [NEW] [vtol_config.yaml](../workspace/src/vtol_control/config/vtol_config.yaml)
Membuat berkas konfigurasi baru berbasis YAML untuk mengatur profil koneksi MAVROS.

```yaml
# Konfigurasi Koneksi Autopilot VTOL
# Pilih profil aktif yang ingin digunakan untuk koneksi MAVROS (ArduPilot/PX4)

# Profil yang tersedia:
# - tcp: Simulasi SITL ArduPilot / Mission Planner (default port: 5762)
# - serial: Koneksi fisik Pixhawk ke Companion Computer (Raspberry Pi) via USB/Serial (default: /dev/ttyACM0 pada baudrate 921600)

active_profile: "tcp"

profiles:
  tcp:
    fcu_url: "tcp://127.0.0.1:5762"
  serial:
    fcu_url: "/dev/ttyACM0:921600"
```

#### [DELETE] [fcu_url.txt](../workspace/src/vtol_control/config/fcu_url.txt)
Menghapus berkas konfigurasi lama berbasis teks mentah.

---

### Kode Sumber Python & Launch Files

---

#### [NEW] [config_reader.py](../workspace/src/vtol_control/vtol_control/config_reader.py)
Membuat modul helper Python baru untuk membaca dan mem-parsing berkas konfigurasi YAML dengan aman, lengkap dengan mekanisme *fallback*.

```python
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def get_fcu_url():
    """
    Membaca berkas vtol_config.yaml untuk mendapatkan fcu_url dari profil yang aktif.
    Mendukung pembacaan langsung dari direktori source (untuk development) maupun share.
    """
    package_name = 'vtol_control'
    try:
        package_share_dir = get_package_share_directory(package_name)
    except Exception:
        # Fallback jika package belum terinstall/ter-source
        package_share_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Path ke direktori source dan installed share
    src_config_path = os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..', 'src', package_name, 'config', 'vtol_config.yaml'))
    installed_config_path = os.path.join(package_share_dir, 'config', 'vtol_config.yaml')

    # Prioritaskan berkas konfigurasi di folder source agar perubahan langsung terasa tanpa colcon build ulang
    config_file_path = src_config_path if os.path.exists(src_config_path) else installed_config_path

    # Fallback default MAVROS URL
    fcu_url = 'tcp://127.0.0.1:5762'

    if os.path.exists(config_file_path):
        try:
            with open(config_file_path, 'r') as f:
                config = yaml.safe_load(f)
                if config:
                    active_profile = config.get('active_profile', 'tcp')
                    profiles = config.get('profiles', {})
                    fcu_url = profiles.get(active_profile, {}).get('fcu_url', fcu_url)
        except Exception as e:
            # Tidak menggunakan get_logger() karena fungsi ini dipanggil sebelum inisialisasi Node ROS2
            print(f"Warning: Gagal membaca berkas konfigurasi {config_file_path}: {e}. Menggunakan default: {fcu_url}")
    else:
        print(f"Warning: Berkas konfigurasi tidak ditemukan pada {config_file_path}. Menggunakan default: {fcu_url}")

    return fcu_url
```

#### [MODIFY] [vtol_core.launch.py](../workspace/src/vtol_control/launch/vtol_core.launch.py)
Mengubah kode launch file untuk memanggil helper `get_fcu_url()` guna mendapatkan URL FCU.

#### [MODIFY] [test_arm.launch.py](../workspace/src/vtol_control/launch/test_arm.launch.py)
Mengubah kode launch file untuk memanggil helper `get_fcu_url()`.

#### [MODIFY] [setup.py](../workspace/src/vtol_control/setup.py)
Memastikan `vtol_config.yaml` dipasang dengan benar ke direktori share package saat dikompilasi.

---

### Dokumentasi

---

#### [MODIFY] [failsafe_dan_konfigurasi.md](failsafe_dan_konfigurasi.md)
Memperbarui panduan dokumentasi dengan instruksi konfigurasi berkas YAML baru serta menambahkan pranala rujukan setup SITL.

#### [NEW] [setup_sitl.md](setup_sitl.md)
Membuat berkas panduan baru `docs/setup_sitl.md` berisi tutorial lengkap instalasi, eksekusi, konfigurasi lokasi awal (home), serta sensor (rangefinder, optical flow) pada ArduPilot SITL.

#### [MODIFY] [README.md](../README.md)
Menambahkan tautan dan kotak tip petunjuk ke `docs/setup_sitl.md` pada Bagian 3 (Koneksi Autopilot SITL).

## Verification Plan

### Automated Tests
- Menjalankan `colcon build --packages-select vtol_control` di dalam kontainer `vtol_dev` untuk memastikan build sukses.

### Manual Verification
- Meluncurkan launch file dengan opsi TCP:
  `ros2 launch vtol_control vtol_core.launch.py`
  Memastikan log output menampilkan URL yang tepat dari profil `tcp`.
- Mengubah berkas `vtol_config.yaml` menjadi profil `serial` atau `custom`, lalu meluncurkan kembali launch file dan memverifikasi log output.
