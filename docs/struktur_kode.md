# Bedah & Struktur Kode Program `vtol_core.py`

Dokumen ini menjelaskan baris demi baris cara kerja kode program **[vtol_core.py](file:///wsl.localhost/Ubuntu-24.04/home/qois51/vtol-dev/workspace/src/vtol_control/vtol_control/vtol_core.py)**. Program ini berfungsi sebagai *core watchdog* keamanan drone, berlangganan ke data sensor MAVROS secara *silent* di latar belakang, dan menangani prosedur darurat (*failsafe*) jika terputus koneksi.

---

## 1. Bagian Impor Library (Baris 1 - 8)
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, SetMode
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
import time
```
*   **`rclpy` & `Node`**: Library utama ROS 2 untuk Python.
*   **`QoSProfile` ...**: Mengatur *Quality of Service* agar cocok dengan penerbitan data di MAVROS.
*   **`State`, `BatteryState`, `PoseStamped`**: Tipe data status koneksi, baterai, dan koordinat posisi lokal drone.
*   **`StreamRate` & `SetMode`**: Service untuk mengatur kecepatan data telemetri dan mengganti mode penerbangan drone (seperti mode `LAND`).

---

## 2. Inisialisasi Kelas Node (Baris 10 - 75)
Program dibungkus di dalam kelas `VtolCore` yang mewarisi kelas `Node` ROS 2.

```python
class VtolCore(Node):
    def __init__(self):
        super().__init__('vtol_core')
```
Mendaftarkan nama node ini sebagai **`vtol_core`**.

### Konfigurasi QoS (Quality of Service)
*   **`qos_state`**: Bertipe `RELIABLE` (aman) dan `TRANSIENT_LOCAL` (menyimpan pesan terakhir) khusus untuk status koneksi autopilot.
*   **`qos_telemetry`**: Bertipe `BEST_EFFORT` (cepat) dan `VOLATILE` khusus untuk posisi dan baterai yang nilainya terus berubah cepat.

### Pendefinisian Pelanggan (Subscription)
```python
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, self.qos_state)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, self.qos_telemetry)
        self.battery_sub = self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, self.qos_telemetry)
```
Berlangganan ke topik status, posisi, dan baterai untuk memantau wahana.

### Timer & Service Client
```python
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
```
*   **`self.timer`**: Menjalankan fungsi pemeriksaan berkala (`timer_callback`) pada frekuensi **10 Hz**.
*   **`self.set_mode_client`**: Digunakan untuk mengirim perintah ganti mode darurat (`LAND`) saat failsafe aktif.

---

## 3. Fungsi Penerima Data / Callback (Baris 77 - 98)
Setiap kali data baru dari sensor masuk, data disimpan di memori lokal kelas tanpa dicetak ke layar:

```python
    def state_callback(self, msg):
        self.current_state = msg
        self.last_state_time = time.time()
        self.mavros_heartbeat_ok = True
        self.autopilot_connected = msg.connected
```
*   **Deteksi Override Pilot**: Jika terdeteksi mode terbang berubah menjadi **`LAND`** secara manual (baik lewat remote control atau Mission Planner GCS), node menandai misi telah selesai (`self.mission_completed = True`) agar seluruh setpoint otomatis dari program luar dihentikan secara aman.

---

## 4. Prosedur Darurat / Failsafe (Baris 109 - 118)
```python
    def trigger_failsafe_land(self):
        self.get_logger().error("FAILSAFE TRIGGERED: Ordering immediate LAND!")
        if self.set_mode_client.wait_for_service(timeout_sec=0.5):
            req = SetMode.Request()
            req.custom_mode = "LAND"
            self.set_mode_client.call_async(req)
```
Mengirim perintah ganti mode ke `LAND` secara asinkron apabila terdeteksi kegagalan koneksi saat drone sedang terbang.

---

## 5. Logika Watchdog Berkala
Di dalam `timer_callback()` yang berjalan setiap 0.1 detik (10 Hz):
1.  **MAVROS Heartbeat Watchdog**: Memeriksa waktu pesan `/mavros/state` terakhir (hanya jika koneksi ke autopilot telah berhasil terjalin minimal satu kali, yaitu `connection_established = True`). Jika data terhenti $> 3.0$ detik saat drone armed (sedang terbang), failsafe `LAND` dipicu. Hal ini mencegah alarm error palsu (*false alarm*) saat MAVROS pertama kali dinyalakan dan sedang sibuk menginisialisasi plugin-pluginnya (yang memakan waktu 20-30 detik).
2.  **Autopilot Connection Watchdog**: Jika MAVROS aktif namun mendeteksi koneksi ke Pixhawk terputus (`connected: False`) saat drone armed, failsafe `LAND` dipicu.
3.  **Request Stream Rate**: Mengirimkan request streaming telemetri 10Hz sekali di awal begitu koneksi terjalin pertama kali.

---

## 6. Fungsi Utama / Main (Baris 147 - 160)
Inisialisasi ROS 2, membuat objek `VtolCore`, dan menjalankan `rclpy.spin(node)` agar callback terus berjalan. Program ini berjalan dengan log quiet di latar belakang tanpa mencetak HUD visual ke stdout terminal.
