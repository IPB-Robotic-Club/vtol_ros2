import os
import yaml
from ament_index_python.packages import get_package_share_directory

def get_camera_config():
    """
    Membaca berkas vision_config.yaml untuk mendapatkan konfigurasi penerimaan kamera & ArUco.
    Mendukung pembacaan langsung dari direktori source maupun share.
    """
    package_name = 'vtol_vision'
    try:
        package_share_dir = get_package_share_directory(package_name)
    except Exception:
        # Fallback jika package belum terinstall/ter-source
        package_share_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Path ke direktori source dan installed share
    src_config_path = os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..', 'src', package_name, 'config', 'vision_config.yaml'))
    installed_config_path = os.path.join(package_share_dir, 'config', 'vision_config.yaml')
    config_file_path = src_config_path if os.path.exists(src_config_path) else installed_config_path

    # Default fallback values
    default_config = {
        'udp_ip': '127.0.0.1',
        'udp_port': 5005,
        'aruco_dict': 'DICT_4X4_50',
        'show_gui': True
    }

    if os.path.exists(config_file_path):
        try:
            with open(config_file_path, 'r') as f:
                config = yaml.safe_load(f)
                if config and 'camera' in config:
                    cam_data = config['camera']
                    for key in default_config:
                        if key in cam_data:
                            default_config[key] = cam_data[key]
        except Exception as e:
            print(f"Warning: Gagal membaca konfigurasi kamera dari {config_file_path}: {e}")
    else:
        print(f"Warning: Berkas konfigurasi tidak ditemukan pada {config_file_path}. Menggunakan default.")
    
    return default_config
