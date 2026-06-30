import socket
import numpy as np
import cv2
import threading
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vtol_vision.config_reader import get_camera_config

class ArucoReceiverNode(Node):
    def __init__(self):
        super().__init__('aruco_receiver')
        
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
        
        self.get_logger().info("ArUco Receiver Node siap dijalankan.")

    def receive_loop(self):
        count = 0
        while self.running:
            try:
                # Set timeout agar thread bisa mendeteksi pemberhentian sistem dengan responsif
                self.sock.settimeout(0.5)
                data, addr = self.sock.recvfrom(65535)
                if not data:
                    continue
                
                count += 1
                np_arr = np.frombuffer(data, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    # Jalankan deteksi ArUco
                    corners, ids, rejected = cv2.aruco.detectMarkers(
                        frame, self.aruco_dict, parameters=self.aruco_params
                    )
                    
                    detections = []
                    if ids is not None:
                        # Gambar penanda yang terdeteksi ke frame
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                        
                        # Konstruksi data deteksi
                        for i, marker_id in enumerate(ids.flatten()):
                            c = corners[i][0] # 4 sudut marker
                            center_x = float(np.mean(c[:, 0]))
                            center_y = float(np.mean(c[:, 1]))
                            
                            detections.append({
                                'id': int(marker_id),
                                'center': [center_x, center_y],
                                'corners': c.tolist()
                            })
                    
                    # Publikasikan data koordinat deteksi sebagai JSON
                    detection_msg = String()
                    detection_msg.data = json.dumps({
                        'timestamp': self.get_clock().now().nanoseconds / 1e9,
                        'count': count,
                        'detected': len(detections) > 0,
                        'markers': detections
                    })
                    self.detection_pub.publish(detection_msg)
                    
                    # Publikasikan gambar teranotasi ke ROS2
                    image_msg = self.convert_cv_to_ros_image(frame)
                    self.image_pub.publish(image_msg)
                    
                    # Visualisasi GUI Lokal (jika diaktifkan)
                    if self.show_gui:
                        cv2.imshow("Webots ArUco Detection Stream", frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self.get_logger().info("Jendela visualisasi ditutup pengguna.")
                            break
                else:
                    self.get_logger().warn("Gagal mendecode paket data gambar.")
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Error pada loop penerima: {e}")
                
        self.sock.close()
        if self.show_gui:
            cv2.destroyAllWindows()

    def convert_cv_to_ros_image(self, cv_img):
        """
        Mengonversi numpy array (matriks BGR OpenCV) secara manual ke sensor_msgs/Image
        untuk menghindari dependensi cv_bridge.
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
        if self.rx_thread.is_alive():
            self.rx_thread.join()
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
