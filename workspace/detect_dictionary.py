import socket
import numpy as np
import cv2
import sys

# List of all standard dictionaries
dicts = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# Bind to UDP port 5005 to receive frames from pi5_streamer.py
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind(('127.0.0.1', 5005))
    sock.settimeout(2.0)
except Exception as e:
    print(f"Gagal mengikat socket ke port 5005: {e}")
    print("Pastikan node aruco_receiver sudah dimatikan sebelum menjalankan skrip ini!")
    sys.exit(1)

print("Mencoba mendengarkan stream UDP di port 5005...")
print("Pastikan pi5_streamer.py sedang aktif dan mengarah ke spanduk ArUco Anda!")

try:
    detected_dicts = {}
    for f_idx in range(15):
        try:
            data, _ = sock.recvfrom(65535)
            if not data:
                continue
            np_arr = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                continue
            
            # Tes pada frame asli dan frame yang di-invert secara software
            for frame_to_test, is_inverted in [(frame, False), (cv2.bitwise_not(frame), True)]:
                for dict_name, dict_id in dicts.items():
                    try:
                        # Handle old API
                        aruco_dict = cv2.aruco.Dictionary_get(dict_id)
                        aruco_params = cv2.aruco.DetectorParameters_create()
                        corners, ids, rejected = cv2.aruco.detectMarkers(frame_to_test, aruco_dict, parameters=aruco_params)
                    except AttributeError:
                        # OpenCV 4.7+ API
                        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
                        parameters = cv2.aruco.DetectorParameters()
                        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
                        corners, ids, rejected = detector.detectMarkers(frame_to_test)
                    
                    if ids is not None and len(ids) > 0:
                        for marker_id in ids.flatten():
                            key = (dict_name, int(marker_id), is_inverted)
                            detected_dicts[key] = detected_dicts.get(key, 0) + 1
        except socket.timeout:
            print("Timeout: Tidak menerima paket UDP pada port 5005. Apakah pi5_streamer.py aktif?")
            sys.exit(1)
            
    if detected_dicts:
        print("\n=== HASIL DETEKSI ===")
        for (d_name, m_id, inv), count in detected_dicts.items():
            inv_str = "Warna Terbalik (Software Inverted)" if inv else "Warna Normal"
            print(f"[SUKSES] Kamus: {d_name} | ID Marker: {m_id} | Mode: {inv_str} (Terdeteksi {count} kali)")
        print("=====================")
    else:
        print("\n[GAGAL] Spanduk ArUco tidak terdeteksi oleh kamus standar apa pun.")
        print("Saran: Cek pencahayaan, fokus kamera, atau pastikan jarak tidak terlalu jauh.")
except KeyboardInterrupt:
    pass
finally:
    sock.close()
