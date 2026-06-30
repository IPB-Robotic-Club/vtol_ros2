import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from vtol_control.vtol_base import VtolBaseNode
from vtol_control.config_reader import get_pid_config
import time
import json
import math

class PIDController:
    def __init__(self, kp, ki, kd, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

    def update(self, error, current_time):
        if self.last_time is None:
            self.last_time = current_time
            self.last_error = error
            return 0.0
            
        dt = current_time - self.last_time
        if dt <= 0.0:
            return 0.0
            
        # Proportional
        p_term = self.kp * error
        
        # Integral dengan Anti-windup sederhana
        self.integral += error * dt
        max_integral = self.max_out / (self.ki if self.ki != 0 else 1.0)
        self.integral = max(min(self.integral, max_integral), -max_integral)
        i_term = self.ki * self.integral
        
        # Derivative
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative
        
        # Output total
        output = p_term + i_term + d_term
        output = max(min(output, self.max_out), -self.max_out)
        
        self.last_error = error
        self.last_time = current_time
        return output

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None


class MissionCenteringNode(VtolBaseNode):
    def __init__(self):
        # Aktifkan timer publish RC bawaan base class (10Hz)
        super().__init__('mission_centering_node', enable_rc_loop=True)
        
        # Load parameters
        self.pid_params = get_pid_config()
        self.max_override = self.pid_params['max_override']
        self.error_threshold = self.pid_params['error_threshold']
        self.centering_duration = self.pid_params['centering_duration']
        self.marker_lost_timeout = self.pid_params['marker_lost_timeout']
        self.takeoff_altitude = self.pid_params['takeoff_altitude']
        
        # Inisialisasi PID
        self.pid_roll = PIDController(
            self.pid_params['kp_roll'], self.pid_params['ki_roll'], self.pid_params['kd_roll'], self.max_override
        )
        self.pid_pitch = PIDController(
            self.pid_params['kp_pitch'], self.pid_params['ki_pitch'], self.pid_params['kd_pitch'], self.max_override
        )
        
        # State tracking
        self.last_detection_time = 0.0
        self.marker_detected = False
        self.first_detection_made = False
        self.norm_error_x = 0.0
        self.norm_error_y = 0.0
        
        self.stable_start_time = None
        self.centered = False
        
        # Berlangganan topik deteksi dari vtol_vision
        self.detection_sub = self.create_subscription(
            String,
            '/vtol/aruco/detection',
            self.detection_callback,
            10
        )
        
        self.get_logger().info("Mission Centering Node Siap!")

    def detection_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.marker_detected = data.get('detected', False)
            
            if self.marker_detected and len(data.get('markers', [])) > 0:
                self.last_detection_time = time.time()
                self.first_detection_made = True
                
                # Gunakan marker pertama yang terdeteksi
                marker = data['markers'][0]
                center_x, center_y = marker['center']
                
                # Normalisasi error terhadap resolusi kamera default Webots (640x480)
                # sehingga berkisar di rentang [-1.0, 1.0]
                W, H = 640.0, 480.0
                
                self.norm_error_x = (center_x - (W / 2.0)) / (W / 2.0)
                self.norm_error_y = (center_y - (H / 2.0)) / (H / 2.0)
            else:
                self.marker_detected = False
        except Exception as e:
            self.get_logger().error(f"Gagal melakukan parsing data deteksi: {e}")

    def execute_centering(self):
        self.get_logger().info("Menunggu marker ArUco terdeteksi...")
        
        # Takeoff ke ketinggian yang terkonfigurasi
        if not self.takeoff(target_altitude=self.takeoff_altitude):
            return
            
        # Hover sebentar untuk stabilisasi awal
        if not self.hover(duration_seconds=3.0):
            return

        self.get_logger().info("Mulai fase Centering presisi...")
        self.pid_roll.reset()
        self.pid_pitch.reset()
        
        centering_start_time = time.time()
        rate_hz = 10.0
        
        while rclpy.ok():
            current_time = time.time()
            
            # Watchdog: Proteksi keselamatan mode terbang manual
            if self.current_state.mode not in ["LOITER", "CMODE(5)"]:
                self.get_logger().warn(f"Intervensi manual terdeteksi! Mode berubah ke {self.current_state.mode}. Abort misi.")
                self.abort_flight()
                return

            # Safety watchdog: Kehilangan sinyal marker setelah terdeteksi setidaknya sekali
            if self.first_detection_made:
                if current_time - self.last_detection_time > self.marker_lost_timeout:
                    self.get_logger().warn(
                        f"Marker hilang selama lebih dari {self.marker_lost_timeout} detik! "
                        f"Menghentikan pergerakan & memicu failsafe LAND..."
                    )
                    # Kembalikan ke netral dan mendarat
                    self.rc_channels[0] = 1500
                    self.rc_channels[1] = 1500
                    self.rc_channels[2] = 1500
                    self.rc_channels[3] = 1500
                    self.publish_rc()
                    self.land()
                    return
            else:
                # Jika marker belum pernah terdeteksi sama sekali sejak fase centering dimulai,
                # batasi waktu tunggu/pencarian hingga 15.0 detik
                if current_time - centering_start_time > 15.0:
                    self.get_logger().warn("Marker tidak terdeteksi setelah 15 detik awal. Menggagalkan misi & landing...")
                    self.land()
                    return

            # Jalankan logika PID jika marker aktif terdeteksi
            if self.marker_detected:
                # Deviasi sumbu X kamera -> Koreksi Roll
                u_roll = self.pid_roll.update(self.norm_error_x, current_time)
                
                # Deviasi sumbu Y kamera -> Koreksi Pitch
                u_pitch = self.pid_pitch.update(self.norm_error_y, current_time)
                
                # Roll: (+) kanan, (-) kiri
                # Pitch: (+) mundur, (-) maju (sesuai arah visual Webots)
                self.rc_channels[0] = int(1500 + u_roll)
                self.rc_channels[1] = int(1500 + u_pitch)
                self.rc_channels[2] = 1500 # Tahan ketinggian
                self.rc_channels[3] = 1500
                
                # Evaluasi kestabilan posisi
                distance_error = math.sqrt(self.norm_error_x**2 + self.norm_error_y**2)
                
                self.get_logger().info(
                    f"Centering... Err X: {self.norm_error_x:.2f} | Y: {self.norm_error_y:.2f} | Dist: {distance_error:.2f} "
                    f"-> RC Roll: {self.rc_channels[0]} | Pitch: {self.rc_channels[1]}",
                    throttle_duration_sec=0.5
                )
                
                if distance_error <= self.error_threshold:
                    if self.stable_start_time is None:
                        self.stable_start_time = current_time
                        self.get_logger().info("Drone berada di dalam batas toleransi. Menunggu stabil...")
                    elif current_time - self.stable_start_time >= self.centering_duration:
                        self.get_logger().info(f"Target stabil selama {self.centering_duration}s tercapai. Sukses!")
                        self.centered = True
                        break
                else:
                    if self.stable_start_time is not None:
                        self.get_logger().info("Drone keluar dari batas toleransi. Mengulang waktu tunggu...")
                        self.stable_start_time = None
            else:
                # Jika marker terputus sesaat (< timeout), drone hover menanti pemulihan
                self.rc_channels[0] = 1500
                self.rc_channels[1] = 1500
                self.rc_channels[2] = 1500
                self.rc_channels[3] = 1500
                self.pid_roll.reset()
                self.pid_pitch.reset()
                self.stable_start_time = None

            # Spin ROS2
            rclpy.spin_once(self, timeout_sec=1.0/rate_hz)
            
        # Pendaratan otonom di atas marker
        if self.centered:
            self.get_logger().info("Memulai pendaratan otomatis otonom tepat di atas marker...")
            self.land()

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MissionCenteringNode()
    try:
        node.execute_centering()
    except KeyboardInterrupt:
        node.safe_exit()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
