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


def get_active_profile() -> str:
    """
    Membaca active_profile dari vtol_config.yaml.
    Mengembalikan 'tcp' (untuk SITL) atau 'serial' (untuk drone real).
    Digunakan sebagai penentu sumber altitude dan konfigurasi lainnya.
    """
    package_name = 'vtol_control'
    try:
        package_share_dir = get_package_share_directory(package_name)
    except Exception:
        package_share_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    src_config_path = os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..', 'src', package_name, 'config', 'vtol_config.yaml'))
    installed_config_path = os.path.join(package_share_dir, 'config', 'vtol_config.yaml')
    config_file_path = src_config_path if os.path.exists(src_config_path) else installed_config_path

    default_profile = 'tcp'

    if os.path.exists(config_file_path):
        try:
            with open(config_file_path, 'r') as f:
                config = yaml.safe_load(f)
                if config:
                    return config.get('active_profile', default_profile)
        except Exception as e:
            print(f"Warning: Gagal membaca active_profile dari {config_file_path}: {e}. Menggunakan default: {default_profile}")

    return default_profile


def get_pid_config():
    """
    Membaca parameter PID untuk misi centering dari vtol_config.yaml.
    """
    package_name = 'vtol_control'
    try:
        package_share_dir = get_package_share_directory(package_name)
    except Exception:
        package_share_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    src_config_path = os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..', 'src', package_name, 'config', 'vtol_config.yaml'))
    installed_config_path = os.path.join(package_share_dir, 'config', 'vtol_config.yaml')
    config_file_path = src_config_path if os.path.exists(src_config_path) else installed_config_path

    default_pid = {
        'kp_roll': 120.0,
        'ki_roll': 5.0,
        'kd_roll': 15.0,
        'kp_pitch': 120.0,
        'ki_pitch': 5.0,
        'kd_pitch': 15.0,
        'enable_yaw_alignment': False,
        'kp_yaw': 3.0,
        'ki_yaw': 0.1,
        'kd_yaw': 4.0,
        'max_override': 100,
        'max_yaw_override': 50,
        'max_throttle_override': 30,
        'yaw_error_threshold': 0.1,
        'error_threshold': 0.06,
        'immediate_land_threshold': 0.05,
        'exit_threshold_multiplier': 1.8,
        'centering_duration': 3.0,
        'marker_lost_timeout': 1.0,
        'deadzone_bias': 25.0,
        'deadzone_bias_yaw': 25.0,
        'control_interval': 1.0,
        'control_interval_fine': 0.8,
        'sequential_axis_mode': False,
        'flip_error_x': False,
        'flip_error_y': False,
        # Active altitude hold parameters
        'hold_altitude': True,
        'kp_altitude': 30.0,
        'max_throttle_correction': 80,
        'hover_base': 1500,
    }

    if os.path.exists(config_file_path):
        try:
            with open(config_file_path, 'r') as f:
                config = yaml.safe_load(f)
                if config and 'pid_centering' in config:
                    pid_data = config['pid_centering']
                    for key in default_pid:
                        if key in pid_data:
                            default_pid[key] = pid_data[key]
        except Exception as e:
            print(f"Warning: Gagal membaca konfigurasi PID dari {config_file_path}: {e}")
    
    return default_pid


def get_takeoff_config():
    """
    Membaca parameter takeoff dari vtol_config.yaml.
    """
    package_name = 'vtol_control'
    try:
        package_share_dir = get_package_share_directory(package_name)
    except Exception:
        package_share_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    src_config_path = os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..', 'src', package_name, 'config', 'vtol_config.yaml'))
    installed_config_path = os.path.join(package_share_dir, 'config', 'vtol_config.yaml')
    config_file_path = src_config_path if os.path.exists(src_config_path) else installed_config_path

    default_takeoff = {
        'takeoff_altitude': 1.2,
        'altitude': 1.2,
        'max_throttle_override': 30,
        'takeoff_throttle': 1530,
        'slowdown_zone': 0.2
    }

    if os.path.exists(config_file_path):
        try:
            with open(config_file_path, 'r') as f:
                config = yaml.safe_load(f)
                if config:
                    # Preferred fallback: pid_centering -> max_throttle_override
                    if 'pid_centering' in config and 'max_throttle_override' in config['pid_centering']:
                        default_takeoff['max_throttle_override'] = config['pid_centering']['max_throttle_override']
                        default_takeoff['takeoff_throttle'] = 1500 + default_takeoff['max_throttle_override']

                    # Specific takeoff block overrides
                    if 'takeoff' in config:
                        takeoff_data = config['takeoff']
                        if 'altitude' in takeoff_data:
                            default_takeoff['takeoff_altitude'] = takeoff_data['altitude']
                            default_takeoff['altitude'] = takeoff_data['altitude']
                        if 'slowdown_zone' in takeoff_data:
                            default_takeoff['slowdown_zone'] = takeoff_data['slowdown_zone']
                        if 'max_throttle_override' in takeoff_data:
                            default_takeoff['max_throttle_override'] = takeoff_data['max_throttle_override']
                            default_takeoff['takeoff_throttle'] = 1500 + takeoff_data['max_throttle_override']
                        elif 'throttle' in takeoff_data:
                            default_takeoff['takeoff_throttle'] = takeoff_data['throttle']
                            default_takeoff['max_throttle_override'] = takeoff_data['throttle'] - 1500
        except Exception as e:
            print(f"Warning: Gagal membaca konfigurasi takeoff dari {config_file_path}: {e}")
    
    return default_takeoff


def get_gcs_url():
    """
    Membaca berkas vtol_config.yaml untuk mendapatkan gcs_url dari blok mavlink_forward
    sesuai profil yang aktif. Digunakan MAVROS sebagai GCS bridge untuk forward
    telemetry MAVLink via UDP ke Ground Control Station eksternal.

    Return:
        str: gcs_url jika forwarding diaktifkan (enabled: true), atau string kosong
             jika forwarding dimatikan atau tidak dikonfigurasi.
    """
    package_name = 'vtol_control'
    try:
        package_share_dir = get_package_share_directory(package_name)
    except Exception:
        package_share_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    src_config_path = os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..', 'src', package_name, 'config', 'vtol_config.yaml'))
    installed_config_path = os.path.join(package_share_dir, 'config', 'vtol_config.yaml')
    config_file_path = src_config_path if os.path.exists(src_config_path) else installed_config_path

    if not os.path.exists(config_file_path):
        print(f"Warning: Berkas konfigurasi tidak ditemukan pada {config_file_path}. GCS forwarding dinonaktifkan.")
        return ''

    try:
        with open(config_file_path, 'r') as f:
            config = yaml.safe_load(f)

        if not config:
            return ''

        active_profile = config.get('active_profile', 'tcp')
        forward_config = config.get('mavlink_forward', {})

        if not forward_config.get('enabled', False):
            print(f"Info: MAVLink GCS forwarding dinonaktifkan untuk profil '{active_profile}'.")
            return ''

        gcs_url = forward_config.get('gcs_url', '')
        if gcs_url:
            print(f"Info: MAVLink GCS forwarding aktif via {gcs_url} (profil: {active_profile})")
        return gcs_url

    except Exception as e:
        print(f"Warning: Gagal membaca konfigurasi GCS forwarding dari {config_file_path}: {e}")
        return ''


def get_search_marker_config():
    """
    Membaca parameter untuk misi search & overshoot marker dari vtol_config.yaml.
    """
    package_name = 'vtol_control'
    try:
        package_share_dir = get_package_share_directory(package_name)
    except Exception:
        package_share_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    src_config_path = os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..', 'src', package_name, 'config', 'vtol_config.yaml'))
    installed_config_path = os.path.join(package_share_dir, 'config', 'vtol_config.yaml')
    config_file_path = src_config_path if os.path.exists(src_config_path) else installed_config_path

    default_config = {
        'target_marker_id': 2,
        'roll_override': 60,
        'pulse_duration': 0.5,
        'pause_duration': 1.0,
        'overshoot_pulse_duration': 0.5,
        'hover_duration_before': 2.0,
        'hover_duration_after': 3.0,
        'alt_correction_threshold': 0.9,
        'tilt_compensation_gain': 0.25,
        'post_detection_pulses': 2,
        'post_detection_direction': 'left',
        'post_detection_roll_override': -60,
        'post_detection_pitch_override': 0,
        'post_detection_pulse_duration': 0.5,
        'post_detection_pause_duration': 1.0,
        'servo_channel': 9,
        'servo_initial_pwm': 1900,
        'servo_drop_pwm': 1100,
        'servo_drop_duration': 2.0,
    }

    if os.path.exists(config_file_path):
        try:
            with open(config_file_path, 'r') as f:
                config = yaml.safe_load(f)
                if config and 'search_marker' in config:
                    sm_data = config['search_marker']
                    for key in default_config:
                        if key in sm_data:
                            default_config[key] = sm_data[key]
        except Exception as e:
            print(f"Warning: Gagal membaca konfigurasi search_marker dari {config_file_path}: {e}")

    return default_config

