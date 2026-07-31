import socket
import numpy as np
import cv2
import threading
import json
import time
import os
import math
import rclpy
import yaml
from http.server import BaseHTTPRequestHandler, HTTPServer
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from vtol_vision.config_reader import get_camera_config

def rotvec2quat(rvec):
    """Convert OpenCV rvec to quaternion (x,y,z,w)."""
    R, _ = cv2.Rodrigues(rvec)
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        S = math.sqrt(trace + 1.0) * 2
        w = 0.25 * S
        x = (R[2,1] - R[1,2]) / S
        y = (R[0,2] - R[2,0]) / S
        z = (R[1,0] - R[0,1]) / S
    elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
        S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        w = (R[2,1] - R[1,2]) / S
        x = 0.25 * S
        y = (R[0,1] + R[1,0]) / S
        z = (R[0,2] + R[2,0]) / S
    elif R[1,1] > R[2,2]:
        S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        w = (R[0,2] - R[2,0]) / S
        x = (R[0,1] + R[1,0]) / S
        y = 0.25 * S
        z = (R[1,2] + R[2,1]) / S
    else:
        S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        w = (R[1,0] - R[0,1]) / S
        x = (R[0,2] + R[2,0]) / S
        y = (R[1,2] + R[2,1]) / S
        z = 0.25 * S

    # Normalize
    mag = math.sqrt(x*x + y*y + z*z + w*w)
    return (x/mag, y/mag, z/mag, w/mag)


LOG_PATH = "/home/pilot/workspace/aruco_vision.log"

# ---------------------------------------------------------------------------
# Shared frame buffer — producer (receive_loop) writes, HTTP server reads
# ---------------------------------------------------------------------------
_frame_lock = threading.Lock()
_latest_jpeg: bytes = b''
_detection_state: dict = {'detected': False, 'markers': [], 'count': 0, 'ts': 0.0}


class MjpegHandler(BaseHTTPRequestHandler):
    """Minimal HTTP handler: / → status HTML, /stream → MJPEG."""

    # Suppress default request logging to keep ROS console clean
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/stream':
            self._serve_mjpeg()
        elif self.path == '/status':
            self._serve_json_status()
        else:
            self._serve_index()

    def _serve_index(self):
        html = (
            '<!DOCTYPE html>\n'
            '<html lang="id">\n'
            '<head>\n'
            '  <meta charset="UTF-8">\n'
            '  <meta name="viewport" content="width=device-width, initial-scale=1.0">\n'
            '  <title>VTOL ArUco Vision</title>\n'
            '  <style>\n'
            '    * { box-sizing: border-box; margin: 0; padding: 0; }\n'
            '    body {\n'
            '      background: #0d0d0d;\n'
            '      color: #e0e0e0;\n'
            '      font-family: \'Segoe UI\', system-ui, sans-serif;\n'
            '      display: flex;\n'
            '      flex-direction: column;\n'
            '      align-items: center;\n'
            '      min-height: 100vh;\n'
            '      padding: 24px 16px;\n'
            '    }\n'
            '    header {\n'
            '      width: 100%;\n'
            '      max-width: 860px;\n'
            '      display: flex;\n'
            '      align-items: center;\n'
            '      gap: 12px;\n'
            '      margin-bottom: 20px;\n'
            '    }\n'
            '    .logo { font-size: 1.7rem; }\n'
            '    h1 { font-size: 1.3rem; font-weight: 600; letter-spacing: 0.03em; }\n'
            '    h1 span { color: #4fc3f7; }\n'
            '    #status-bar {\n'
            '      width: 100%;\n'
            '      max-width: 860px;\n'
            '      background: #1a1a2e;\n'
            '      border: 1px solid #2a2a4a;\n'
            '      border-radius: 10px;\n'
            '      padding: 12px 18px;\n'
            '      display: flex;\n'
            '      gap: 24px;\n'
            '      align-items: center;\n'
            '      margin-bottom: 16px;\n'
            '      flex-wrap: wrap;\n'
            '    }\n'
            '    .stat { display: flex; flex-direction: column; gap: 2px; }\n'
            '    .stat-label { font-size: 0.65rem; color: #888; text-transform: uppercase; letter-spacing: 0.08em; }\n'
            '    .stat-value { font-size: 1rem; font-weight: 600; color: #4fc3f7; }\n'
            '    #pill {\n'
            '      margin-left: auto;\n'
            '      padding: 4px 14px;\n'
            '      border-radius: 20px;\n'
            '      font-size: 0.75rem;\n'
            '      font-weight: 700;\n'
            '      letter-spacing: 0.05em;\n'
            '    }\n'
            '    .pill-detected { background: #1b5e20; color: #69f0ae; border: 1px solid #2e7d32; }\n'
            '    .pill-none     { background: #1a1a1a; color: #888;    border: 1px solid #333; }\n'
            '    #stream-wrap {\n'
            '      width: 100%;\n'
            '      max-width: 860px;\n'
            '      border-radius: 12px;\n'
            '      overflow: hidden;\n'
            '      border: 1px solid #1e1e3a;\n'
            '      background: #111;\n'
            '    }\n'
            '    #stream-wrap img { width: 100%; display: block; }\n'
            '    footer { margin-top: 18px; font-size: 0.7rem; color: #444; }\n'
            '  </style>\n'
            '</head>\n'
            '<body>\n'
            '  <header>\n'
            '    <span class="logo">&#x1F6F8;</span>\n'
            '    <h1>VTOL &mdash; <span>ArUco Vision Stream</span></h1>\n'
            '  </header>\n'
            '  <div id="status-bar">\n'
            '    <div class="stat">\n'
            '      <span class="stat-label">Frame</span>\n'
            '      <span class="stat-value" id="s-frame">&mdash;</span>\n'
            '    </div>\n'
            '    <div class="stat">\n'
            '      <span class="stat-label">Marker terdeteksi</span>\n'
            '      <span class="stat-value" id="s-count">&mdash;</span>\n'
            '    </div>\n'
            '    <div class="stat">\n'
            '      <span class="stat-label">Marker ID</span>\n'
            '      <span class="stat-value" id="s-ids">&mdash;</span>\n'
            '    </div>\n'
            '    <div class="stat">\n'
            '      <span class="stat-label">Timestamp</span>\n'
            '      <span class="stat-value" id="s-ts">&mdash;</span>\n'
            '    </div>\n'
            '    <span id="pill" class="pill-none">NO MARKER</span>\n'
            '  </div>\n'
            '  <div id="stream-wrap">\n'
            '    <img src="/stream" alt="ArUco live stream" />\n'
            '  </div>\n'
            '  <footer>VTOL Vision &bull; MJPEG stream @ /stream &bull; JSON status @ /status</footer>\n'
            '  <script>\n'
            '    async function pollStatus() {\n'
            '      try {\n'
            '        const r = await fetch(\'/status\');\n'
            '        const d = await r.json();\n'
            '        document.getElementById(\'s-frame\').textContent = d.count;\n'
            '        document.getElementById(\'s-count\').textContent = d.markers.length;\n'
            '        document.getElementById(\'s-ts\').textContent = d.ts ? new Date(d.ts * 1000).toLocaleTimeString() : \'--\';\n'
            '        const ids = d.markers.map(m => \'ID \' + m.id).join(\', \') || \'--\';\n'
            '        document.getElementById(\'s-ids\').textContent = ids;\n'
            '        const pill = document.getElementById(\'pill\');\n'
            '        if (d.detected) {\n'
            '          pill.textContent = \'DETECTED\';\n'
            '          pill.className = \'pill-detected\';\n'
            '        } else {\n'
            '          pill.textContent = \'NO MARKER\';\n'
            '          pill.className = \'pill-none\';\n'
            '        }\n'
            '      } catch (_) {}\n'
            '    }\n'
            '    setInterval(pollStatus, 500);\n'
            '    pollStatus();\n'
            '  </script>\n'
            '</body>\n'
            '</html>'
        ).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    def _serve_json_status(self):
        with _frame_lock:
            payload = json.dumps(_detection_state).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _serve_mjpeg(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=vtolframe')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        try:
            while True:
                with _frame_lock:
                    jpeg = _latest_jpeg
                if jpeg:
                    self.wfile.write(
                        b'--vtolframe\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' +
                        jpeg +
                        b'\r\n'
                    )
                    self.wfile.flush()
                time.sleep(0.033)   # ~30 fps cap
        except (BrokenPipeError, ConnectionResetError):
            pass


def _run_stream_server(port: int):
    """Blocking HTTP server — run in a daemon thread."""
    server = HTTPServer(('0.0.0.0', port), MjpegHandler)
    server.serve_forever()


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class ArucoReceiverNode(Node):
    def __init__(self):
        super().__init__('aruco_receiver')

        # Setup log file
        try:
            self.log_file = open(LOG_PATH, "w")
            self.log_file.write("=== LOG VISION ARUCO DIMULAI ===\n")
            self.log_file.write(
                f"{'TIMESTAMP':>10} | {'FRAME':>6} | {'RAW_BYTES':>10} | "
                f"{'FRAME_WxH':>11} | {'DETECTED':>8} | {'REJECTED':>8} | "
                f"{'MARKER_ID':>9} | {'CENTER_X':>9} | {'CENTER_Y':>9} | "
                f"{'NORM_EX':>8} | {'NORM_EY':>8} | {'CORNERS'}\n"
            )
            self.log_file.write("-" * 160 + "\n")
            self.log_file.flush()
        except Exception as e:
            print(f"[WARN] Gagal membuka log vision: {e}")
            self.log_file = None

        # Load configuration
        self.cam_config = get_camera_config()
        self.camera_source  = self.cam_config['camera_source']    # 'udp' atau 'v4l'
        self.active_profile = self.cam_config['active_profile']
        self.aruco_dict_name = self.cam_config['aruco_dict']
        self.stream_port    = self.cam_config['stream_port']
        self.marker_length  = self.cam_config['marker_length']

        cam_mat_list = self.cam_config['camera_matrix']
        self.camera_matrix = np.array(cam_mat_list, dtype=np.float32).reshape(3, 3)
        self.dist_coeffs   = np.array(self.cam_config['dist_coeffs'], dtype=np.float32)

        # Check if local calibration exists in workspace (hanya untuk profil raspi)
        calib_path = "/home/pilot/workspace/camera_calibration.yaml"
        if self.active_profile == 'raspi' and os.path.exists(calib_path):
            try:
                with open(calib_path, 'r') as f:
                    calib_data = yaml.safe_load(f)
                    self.camera_matrix = np.array(calib_data['camera_matrix'], dtype=np.float32).reshape(3, 3)
                    self.dist_coeffs   = np.array(calib_data['distortion_coefficients'], dtype=np.float32)
                    self.get_logger().info(f"Berhasil memuat kalibrasi kamera Raspi dari: {calib_path}")
            except Exception as e:
                self.get_logger().warn(f"Gagal memuat kalibrasi dari {calib_path}, menggunakan config bawaan: {e}")
        elif self.active_profile == 'sitl':
            self.get_logger().info(f"Menggunakan matriks kamera ideal SITL tanpa distorsi (active_profile={self.active_profile})")


        # Setup ArUco Dictionary & Parameters (OpenCV 4.6.0 API)
        dict_id = getattr(cv2.aruco, self.aruco_dict_name, cv2.aruco.DICT_7X7_50)
        self.aruco_dict   = cv2.aruco.Dictionary_get(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Optimasi parameter deteksi agar sangat stabil dan tahan pantulan cahaya/kerutan:
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 45
        self.aruco_params.adaptiveThreshWinSizeStep = 4
        self.aruco_params.adaptiveThreshConstant = 7
        
        # Toleransi ekstra terhadap kerutan spanduk (garis tepi tidak lurus sempurna akibat berkerut)
        self.aruco_params.polygonalApproxAccuracyRate = 0.05
        
        # Toleransi ekstra terhadap pantulan cahaya (memperbolehkan koreksi bit biner yang bocor/rusak karena kilauan)
        self.aruco_params.errorCorrectionRate = 0.8

        # Resolusi kamera (diupdate saat frame pertama diterima)
        self.frame_w = 640.0
        self.frame_h = 480.0

        # Setup Publishers
        self.image_pub     = self.create_publisher(Image,  '/vtol/camera/image_raw', 10)
        self.detection_pub = self.create_publisher(String, '/vtol/aruco/detection',  10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Inisialisasi camera source (selalu UDP via port 5005)
        self.sock = None
        if self.camera_source == 'udp':
            udp_ip   = self.cam_config['udp_ip']
            udp_port = self.cam_config['udp_port']
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.sock.bind((udp_ip, udp_port))
                self.get_logger().info(
                    f"[{self.active_profile}] UDP socket terikat pada {udp_ip}:{udp_port}"
                )
            except Exception as e:
                self.get_logger().error(
                    f"Gagal mengikat socket UDP ke {udp_ip}:{udp_port}: {e}"
                )
                raise e
        else:
            raise ValueError(f"camera_source '{self.camera_source}' tidak didukung. Semua profil (sitl & raspi) menggunakan 'udp'.")

        # MJPEG Web Stream Server
        stream_thread = threading.Thread(
            target=_run_stream_server, args=(self.stream_port,), daemon=True
        )
        stream_thread.start()
        self.get_logger().info(
            f"MJPEG stream server aktif -> http://0.0.0.0:{self.stream_port}  "
            f"(buka /stream untuk raw feed, / untuk dashboard)"
        )

        # Jalankan UDP receive loop thread
        self.running = True
        self.rx_thread = threading.Thread(target=self._receive_loop_udp, daemon=True)
        self.rx_thread.start()


        self.get_logger().info(
            f"ArUco Receiver Node siap. "
            f"Profile={self.active_profile}, Source={self.camera_source}, "
            f"Dict={self.aruco_dict_name}, Log={LOG_PATH}"
        )

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def write_vision_log(self, count, raw_bytes, frame, corners, ids, rejected):
        """Tulis satu baris log CSV lengkap per frame."""
        if not self.log_file:
            return

        try:
            ts = time.strftime('%H:%M:%S')
            h, w = frame.shape[:2]
            self.frame_w = float(w)
            self.frame_h = float(h)
            frame_size = f"{w}x{h}"
            n_rejected = len(rejected) if rejected is not None else 0

            if ids is not None and len(ids) > 0:
                for i, marker_id in enumerate(ids.flatten()):
                    c = corners[i][0]
                    cx = float(np.mean(c[:, 0]))
                    cy = float(np.mean(c[:, 1]))
                    norm_ex = (cx - w / 2.0) / (w / 2.0)
                    norm_ey = (cy - h / 2.0) / (h / 2.0)
                    corners_str = " ".join([f"({p[0]:.0f},{p[1]:.0f})" for p in c])
                    self.log_file.write(
                        f"{ts:>10} | {count:>6} | {raw_bytes:>10} | "
                        f"{frame_size:>11} | {'YES':>8} | {n_rejected:>8} | "
                        f"{int(marker_id):>9} | {cx:>9.2f} | {cy:>9.2f} | "
                        f"{norm_ex:>8.4f} | {norm_ey:>8.4f} | {corners_str}\n"
                    )
            else:
                self.log_file.write(
                    f"{ts:>10} | {count:>6} | {raw_bytes:>10} | "
                    f"{frame_size:>11} | {'NO':>8} | {n_rejected:>8} | "
                    f"{'---':>9} | {'---':>9} | {'---':>9} | "
                    f"{'---':>8} | {'---':>8} | ---\n"
                )
            self.log_file.flush()
        except Exception as e:
            self.get_logger().warn(f"Gagal menulis vision log: {e}")

    # ------------------------------------------------------------------
    # Shared ArUco detection — dipanggil dari kedua receive loop
    # ------------------------------------------------------------------

    def _process_frame(self, frame, count, raw_bytes):
        """Jalankan deteksi ArUco, annotasi frame, publish ROS + update MJPEG buffer."""
        global _latest_jpeg, _detection_state

        # Lakukan undistort untuk menghilangkan distorsi lensa (Opsi 1) - Aktifkan jika kalibrasi presisi
        # frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        # Konversi ke Grayscale dan gunakan CLAHE untuk meredam pantulan cahaya (glare) & bayangan
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        gray_eq = clahe.apply(gray)

        # 1. Coba deteksi normal pada gambar hasil pre-processing
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray_eq, self.aruco_dict, parameters=self.aruco_params
        )

        # 2. Coba deteksi dengan membalikkan warna frame (Inverted) pada gambar hasil pre-processing
        gray_inv = cv2.bitwise_not(gray_eq)
        corners_inv, ids_inv, rejected_inv = cv2.aruco.detectMarkers(
            gray_inv, self.aruco_dict, parameters=self.aruco_params
        )

        # 3. Gabungkan hasil deteksi dari kedua mode
        if ids_inv is not None and len(ids_inv) > 0:
            if ids is not None and len(ids) > 0:
                ids = np.concatenate((ids, ids_inv), axis=0)
                corners = corners + corners_inv
            else:
                ids = ids_inv
                corners = corners_inv

        self.write_vision_log(count, raw_bytes, frame, corners, ids, rejected)

        detections = []
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            half = self.marker_length / 2.0
            obj_pts_single = np.array([
                [-half,  half, 0],
                [ half,  half, 0],
                [ half, -half, 0],
                [-half, -half, 0],
            ], dtype=np.float32)

            for i, marker_id in enumerate(ids.flatten()):
                c = corners[i][0]
                center_x = float(np.mean(c[:, 0]))
                center_y = float(np.mean(c[:, 1]))
                img_pts  = c.astype(np.float32)

                # Gunakan IPPE_SQUARE untuk menghilangkan pose flip ambiguity.
                # Memberikan 2 kandidat solusi + reprojection error masing-masing.
                # Pilih solusi dengan reprojection error terkecil.
                try:
                    _, rvecs_sol, tvecs_sol, repro_errs = cv2.solvePnPGeneric(
                        obj_pts_single, img_pts,
                        self.camera_matrix, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    best_idx = int(np.argmin([e[0] for e in repro_errs]))
                    rvec = rvecs_sol[best_idx].flatten()
                    tvec = tvecs_sol[best_idx].flatten()
                except Exception:
                    # Fallback ke estimatePoseSingleMarkers jika solvePnP gagal
                    rvecs_fb, tvecs_fb, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[i]], self.marker_length, self.camera_matrix, self.dist_coeffs
                    )
                    rvec = rvecs_fb[0][0]
                    tvec = tvecs_fb[0][0]

                detections.append({
                    'id': int(marker_id),
                    'center': [center_x, center_y],
                    'corners': c.tolist(),
                    'pose_tvec': [float(tvec[0]), float(tvec[1]), float(tvec[2])],
                    'pose_rvec': [float(rvec[0]), float(rvec[1]), float(rvec[2])]
                })

                # Draw axes
                cv2.drawFrameAxes(
                    frame, self.camera_matrix, self.dist_coeffs,
                    rvec, tvec, self.marker_length * 0.5
                )

                # Broadcast TF
                t = TransformStamped()
                t.header.stamp    = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_link'
                t.child_frame_id  = f'aruco_marker_{int(marker_id)}'
                t.transform.translation.x = float(tvec[0])
                t.transform.translation.y = float(tvec[1])
                t.transform.translation.z = float(tvec[2])
                qx, qy, qz, qw = rotvec2quat(rvec)
                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(t)

        # HUD overlay
        self._draw_stream_overlay(frame, count, detections)

        # Update MJPEG buffer
        ok, jpeg_buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with _frame_lock:
                _latest_jpeg = jpeg_buf.tobytes()
                _detection_state = {
                    'detected': len(detections) > 0,
                    'markers': detections,
                    'count': count,
                    'ts': self.get_clock().now().nanoseconds / 1e9,
                    'frame_w': int(self.frame_w),
                    'frame_h': int(self.frame_h),
                }

        # Publish ke ROS
        detection_msg = String()
        detection_msg.data = json.dumps({
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'count': count,
            'detected': len(detections) > 0,
            'markers': detections,
            'frame_w': int(self.frame_w),
            'frame_h': int(self.frame_h)
        })
        self.detection_pub.publish(detection_msg)
        self.image_pub.publish(self.convert_cv_to_ros_image(frame))

    # ------------------------------------------------------------------
    # Receive loop: UDP (profil tcp / SITL)
    # ------------------------------------------------------------------

    def _receive_loop_udp(self):
        count = 0
        while self.running:
            try:
                self.sock.settimeout(0.5)
                data, _ = self.sock.recvfrom(65535)
                if not data:
                    continue

                count += 1
                raw_bytes = len(data)
                np_arr = np.frombuffer(data, dtype=np.uint8)
                frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is not None:
                    self._process_frame(frame, count, raw_bytes)
                else:
                    self.get_logger().warn(
                        f"[UDP Frame {count}] Gagal mendecode paket ({raw_bytes} bytes)."
                    )
                    if self.log_file:
                        self.log_file.write(
                            f"{time.strftime('%H:%M:%S'):>10} | {count:>6} | {raw_bytes:>10} | "
                            f"{'DECODE_FAIL':>11} | {'ERR':>8} | {'---':>8} | "
                            f"{'---':>9} | {'---':>9} | {'---':>9} | "
                            f"{'---':>8} | {'---':>8} | ---\n"
                        )
                        self.log_file.flush()

            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                self.get_logger().error(f"[UDP] Error: {e}")

        try:
            self.sock.close()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # HUD overlay (digambar sebelum JPEG encode, bukan untuk GUI window)
    # ------------------------------------------------------------------


    def _draw_stream_overlay(self, frame, count, detections):
        """Gambar HUD overlay di atas frame sebelum di-encode ke MJPEG."""
        h, w = frame.shape[:2]

        # ── Status bar pojok kiri atas ─────────────────────────────────
        n_markers = len(detections)
        status_color = (0, 220, 0) if n_markers > 0 else (0, 80, 220)
        status_text = f"Frame #{count}  |  Marker: {n_markers}"
        cv2.rectangle(frame, (0, 0), (w, 30), (0, 0, 0), -1)
        cv2.putText(frame, status_text, (8, 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2, cv2.LINE_AA)

        # ── Info per marker ────────────────────────────────────────────
        for det in detections:
            marker_id = det['id']
            cx, cy = det['center']
            norm_ex = (cx - w / 2.0) / (w / 2.0)
            norm_ey = (cy - h / 2.0) / (h / 2.0)

            cx_i, cy_i = int(cx), int(cy)
            cv2.circle(frame, (cx_i, cy_i), 5, (0, 255, 255), -1)

            # Menampilkan koordinat 3D di HUD jika tersedia
            x_m, y_m, z_m = 0.0, 0.0, 0.0
            if 'pose_tvec' in det:
                x_m, y_m, z_m = det['pose_tvec']

            lines = [
                (f"ID: {marker_id}", 0.9, (0, 255, 0), 2),
                (f"XYZ: ({x_m:+.2f}, {y_m:+.2f}, {z_m:+.2f}) m", 0.55, (0, 220, 255), 1),
                (f"px ({cx_i}, {cy_i})",   0.55, (255, 255, 255), 1),
            ]
            line_h = 24
            text_y = cy_i - 10 - len(lines) * line_h
            text_y = max(text_y, 35)

            for text, scale, color, thickness in lines:
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
                tx = max(4, min(cx_i - tw // 2, w - tw - 4))
                cv2.putText(frame, text, (tx + 1, text_y + 1),
                            cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 1, cv2.LINE_AA)
                cv2.putText(frame, text, (tx, text_y),
                            cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)
                text_y += line_h

            on_center = abs(norm_ex) < 0.15 and abs(norm_ey) < 0.15
            box_color = (0, 255, 0) if on_center else (0, 165, 255)
            cv2.line(frame, (cx_i, cy_i - 5), (cx_i, text_y - line_h + 5), box_color, 1, cv2.LINE_AA)

    # ------------------------------------------------------------------
    # ROS Image conversion
    # ------------------------------------------------------------------

    def convert_cv_to_ros_image(self, cv_img):
        """
        Mengonversi numpy array (BGR OpenCV) ke sensor_msgs/Image
        tanpa dependensi cv_bridge.
        """
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height = cv_img.shape[0]
        msg.width = cv_img.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = cv_img.shape[1] * 3
        msg.data = cv_img.tobytes()
        return msg

    def destroy_node(self):
        self.running = False
        # Tutup sumber kamera sesuai tipe
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        if self.capture is not None:
            try:
                self.capture.release()
            except Exception:
                pass
        if self.picam2 is not None:
            try:
                self.picam2.stop()
            except Exception:
                pass
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=2.0)
        if self.log_file:
            try:
                self.log_file.write("=== LOG VISION ARUCO BERAKHIR ===\n")
                self.log_file.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
