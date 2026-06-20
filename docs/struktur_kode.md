# Bedah & Struktur Kode Program `vtol_core.py` (Basic)

Dokumen ini menjelaskan baris demi baris cara kerja kode program **[vtol_core.py](file:///wsl.localhost/Ubuntu-24.04/home/qois51/vtol-dev/workspace/src/vtol_control/vtol_control/vtol_core.py)**. Program ini berfungsi sebagai *state listener* telemetri drone dan meminta *stream rate* telemetri MAVROS saat terhubung.

---

## 1. Bagian Impor Library
```python
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate
```
*   **`rclpy` & `Node`**: Library utama ROS 2 untuk Python.
*   **`State`**: Tipe data status koneksi, mode, dan arming dari MAVROS.
*   **`StreamRate`**: Service untuk mengatur kecepatan data telemetri dari autopilot (ArduPilot/PX4).

---

## 2. Inisialisasi Kelas Node
Program dibungkus di dalam kelas `VtolCore` yang mewarisi kelas `Node` ROS 2 secara langsung.

```python
class VtolCore(Node):
    def __init__(self):
        super().__init__('vtol_core')
```
Mendaftarkan nama node ini sebagai **`vtol_core`**.

### Konfigurasi QoS (Quality of Service)
*   **`qos_state`**: Bertipe `RELIABLE` (aman) dan `TRANSIENT_LOCAL` (menyimpan pesan terakhir) khusus untuk status koneksi autopilot.

### Pendefinisian Pelanggan (Subscription) & Client
```python
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            self.qos_state
        )
        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')
```
*   Berlangganan ke topik status (`/mavros/state`) untuk memantau perubahan status drone.
*   Membuat service client untuk meminta rate pengiriman telemetri.

---

## 3. Fungsi Penerima Data / Callback
Setiap kali status baru dari MAVROS masuk, data diolah untuk mencetak perubahan log ke terminal:

```python
    def state_callback(self, msg):
```
*   **Log Perubahan Koneksi**: Mencetak pesan ketika koneksi autopilot tersambung (`Autopilot Link Connected successfully!`) atau terputus.
*   **Log Perubahan Arming**: Mencetak status arming ketika berubah dari `ARMED` ke `DISARMED` atau sebaliknya.
*   **Log Perubahan Mode Terbang**: Mencetak nama mode ketika terjadi perubahan mode penerbangan (misal ke `GUIDED`, `LAND`, `LOITER`).
*   **Request Stream Rate**: Memanggil `request_stream_rate` sekali ketika autopilot baru saja terhubung untuk memastikan MAVROS mempublikasikan telemetri lain (seperti baterai dan posisi) pada rate 10Hz.

---

## 4. Fungsi Utama / Main
Inisialisasi ROS 2, membuat objek `VtolCore`, dan menjalankan `rclpy.spin(node)` agar callback terus berjalan.
