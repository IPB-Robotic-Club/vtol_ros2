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
    fcu_url = 'tcp://127.0.0.1:14550'

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
