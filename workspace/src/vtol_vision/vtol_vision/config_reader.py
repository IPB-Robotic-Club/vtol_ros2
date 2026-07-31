import os
import yaml
from ament_index_python.packages import get_package_share_directory


def get_camera_config():
    """
    Membaca vision_config.yaml dan mengembalikan konfigurasi yang sudah di-flatten
    berdasarkan active_profile (tcp / serial).

    Struktur return dict:
      active_profile   : str   — 'tcp' atau 'serial'
      camera_source    : str   — 'udp' atau 'v4l'
      # UDP-specific (hanya jika camera_source == 'udp'):
      udp_ip           : str
      udp_port         : int
      # V4L-specific (hanya jika camera_source == 'v4l'):
      device           : str
      capture_width    : int
      capture_height   : int
      capture_fps      : int
      # ArUco
      aruco_dict       : str
      marker_length    : float
      camera_matrix    : list[float]  (9 elemen)
      dist_coeffs      : list[float]  (5 elemen)
      # Stream
      stream_port      : int
    """
    package_name = 'vtol_vision'
    try:
        package_share_dir = get_package_share_directory(package_name)
    except Exception:
        package_share_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    src_config_path = os.path.abspath(
        os.path.join(package_share_dir, '..', '..', '..', '..', 'src',
                     package_name, 'config', 'vision_config.yaml')
    )
    installed_config_path = os.path.join(package_share_dir, 'config', 'vision_config.yaml')
    config_file_path = src_config_path if os.path.exists(src_config_path) else installed_config_path

    # ── Default values (digunakan jika file tidak ditemukan) ──────────────
    defaults = {
        'active_profile': 'tcp',
        # UDP defaults
        'udp_ip': '127.0.0.1',
        'udp_port': 5005,
        # V4L defaults
        'device': '/dev/video0',
        'capture_width': 640,
        'capture_height': 480,
        'capture_fps': 30,
        # ArUco defaults
        'aruco_dict': 'DICT_7X7_50',
        'marker_length': 0.20,
        'camera_matrix': [320.0, 0.0, 320.0, 0.0, 320.0, 240.0, 0.0, 0.0, 1.0],
        'dist_coeffs': [0.0, 0.0, 0.0, 0.0, 0.0],
        # Stream defaults
        'stream_port': 8086,
    }

    if not os.path.exists(config_file_path):
        print(f"Warning: Berkas konfigurasi tidak ditemukan pada {config_file_path}. Menggunakan default.")
        defaults['camera_source'] = 'udp'
        return defaults

    try:
        with open(config_file_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
    except Exception as e:
        print(f"Warning: Gagal membaca konfigurasi dari {config_file_path}: {e}")
        defaults['camera_source'] = 'udp'
        return defaults

    # ── Baca active_profile ────────────────────────────────────────────────
    active_profile = cfg.get('active_profile', defaults['active_profile'])
    defaults['active_profile'] = active_profile

    # ── Baca konfigurasi profil aktif ──────────────────────────────────────
    profiles = cfg.get('profiles', {})
    profile_cfg = profiles.get(active_profile, {})
    defaults['camera_source'] = profile_cfg.get('camera_source', 'udp')

    if defaults['camera_source'] == 'udp':
        defaults['udp_ip']   = profile_cfg.get('udp_ip',   defaults['udp_ip'])
        defaults['udp_port'] = profile_cfg.get('udp_port', defaults['udp_port'])

    # ── Baca konfigurasi ArUco ─────────────────────────────────────────────
    aruco_cfg = cfg.get('aruco', {})
    defaults['aruco_dict']    = aruco_cfg.get('aruco_dict',    defaults['aruco_dict'])
    defaults['marker_length'] = aruco_cfg.get('marker_length', defaults['marker_length'])

    # Baca camera_matrix & dist_coeffs per profil terlebih dahulu, fallback ke top-level aruco
    defaults['camera_matrix'] = profile_cfg.get('camera_matrix', aruco_cfg.get('camera_matrix', defaults['camera_matrix']))
    defaults['dist_coeffs']   = profile_cfg.get('dist_coeffs',   aruco_cfg.get('dist_coeffs',   defaults['dist_coeffs']))

    # ── Baca konfigurasi stream ────────────────────────────────────────────
    stream_cfg = cfg.get('stream', {})
    defaults['stream_port'] = stream_cfg.get('port', defaults['stream_port'])

    return defaults

