# Tasks - Konfigurasi MAVLink FCU URL Berbasis YAML & Setup SITL

- [x] Membuat berkas konfigurasi [vtol_config.yaml](../workspace/src/vtol_control/config/vtol_config.yaml)
- [x] Membuat modul [config_reader.py](../workspace/src/vtol_control/vtol_control/config_reader.py) untuk parsing YAML
- [x] Menghapus berkas konfigurasi lama `workspace/src/vtol_control/config/fcu_url.txt`
- [x] Memperbarui [setup.py](../workspace/src/vtol_control/setup.py) untuk menginstal berkas konfigurasi baru
- [x] Memperbarui launch file [vtol_core.launch.py](../workspace/src/vtol_control/launch/vtol_core.launch.py)
- [x] Memperbarui launch file [test_arm.launch.py](../workspace/src/vtol_control/launch/test_arm.launch.py)
- [x] Memperbarui dokumentasi [failsafe_dan_konfigurasi.md](failsafe_dan_konfigurasi.md)
- [x] Mengompilasi workspace dengan `colcon build` dan memverifikasi build
- [x] Memverifikasi jalannya program secara manual dengan memicu launch file
- [x] Membuat berkas panduan setup SITL [setup_sitl.md](setup_sitl.md)
- [x] Memperbarui [README.md](../README.md) dengan tautan ke panduan setup SITL
- [x] Memperbarui [failsafe_dan_konfigurasi.md](failsafe_dan_konfigurasi.md) dengan tautan ke panduan setup SITL
