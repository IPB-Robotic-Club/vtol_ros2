import socket
import numpy as np
import cv2
import threading
import json
import time
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vtol_vision.config_reader import get_camera_config


LOG_PATH = "/home/pilot/workspace/aruco_vision.log"


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
        self.udp_ip = self.cam_config['udp_ip']
        self.udp_port = self.cam_config['udp_port']
        self.aruco_dict_name = self.cam_config['aruco_dict']
        self.show_gui = self.cam_config['show_gui']

        # Setup ArUco Dictionary & Parameters (OpenCV 4.6.0 API)
        dict_id = getattr(cv2.aruco, self.aruco_dict_name, cv2.aruco.DICT_4X4_50)
        self.aruco_dict = cv2.aruco.Dictionary_get(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Resolusi kamera (diupdate saat frame pertama diterima)
        self.frame_w = 640.0
        self.frame_h = 480.0

        # Setup Publishers
        self.image_pub = self.create_publisher(Image, '/vtol/camera/image_raw', 10)
        self.detection_pub = self.create_publisher(String, '/vtol/aruco/detection', 10)

        # UDP Socket Setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((self.udp_ip, self.udp_port))
            self.get_logger().info(f"Socket UDP berhasil terikat pada {self.udp_ip}:{self.udp_port}")
        except Exception as e:
            self.get_logger().error(f"Gagal mengikat socket UDP ke {self.udp_ip}:{self.udp_port}: {e}")
            raise e

        # Receiver thread flag
        self.running = True
        self.rx_thread = threading.Thread(target=self.receive_loop)
        self.rx_thread.daemon = True
        self.rx_thread.start()

        self.get_logger().info(f"ArUco Receiver Node siap. Dict={self.aruco_dict_name}, Log={LOG_PATH}")

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

    def receive_loop(self):
        count = 0
        while self.running:
            try:
                self.sock.settimeout(0.5)
                data, addr = self.sock.recvfrom(65535)
                if not data:
                    continue

                count += 1
                raw_bytes = len(data)
                np_arr = np.frombuffer(data, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is not None:
                    # Jalankan deteksi ArUco
                    corners, ids, rejected = cv2.aruco.detectMarkers(
                        frame, self.aruco_dict, parameters=self.aruco_params
                    )

                    # Tulis log vision lengkap
                    self.write_vision_log(count, raw_bytes, frame, corners, ids, rejected)

                    detections = []
                    if ids is not None:
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                        for i, marker_id in enumerate(ids.flatten()):
                            c = corners[i][0]
                            center_x = float(np.mean(c[:, 0]))
                            center_y = float(np.mean(c[:, 1]))

                            detections.append({
                                'id': int(marker_id),
                                'center': [center_x, center_y],
                                'corners': c.tolist()
                            })

                    # Publikasikan data deteksi
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

                    # Publikasikan gambar teranotasi
                    image_msg = self.convert_cv_to_ros_image(frame)
                    self.image_pub.publish(image_msg)

                    # Visualisasi GUI Lokal
                    if self.show_gui:
                        cv2.imshow("Webots ArUco Detection Stream", frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self.get_logger().info("Jendela visualisasi ditutup pengguna.")
                            break
                else:
                    self.get_logger().warn(f"[Frame {count}] Gagal mendecode paket UDP ({raw_bytes} bytes).")
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
                # Socket closed during shutdown, exit loop gracefully
                break
            except Exception as e:
                self.get_logger().error(f"Error pada loop penerima: {e}")

        try:
            self.sock.close()
        except Exception:
            pass
        if self.show_gui:
            cv2.destroyAllWindows()

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
        try:
            self.sock.close()
        except Exception:
            pass
        if self.rx_thread.is_alive():
            self.rx_thread.join()
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
