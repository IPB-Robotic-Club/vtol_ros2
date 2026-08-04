# Panduan Setup & Uji Coba Penerimaan Kamera UDP (vtol_vision)

Dokumen ini menjelaskan langkah-langkah detail untuk mengalirkan (*stream*) kamera dari pengirim (*sender* seperti Webots, script Python lokal, atau kamera fisik) ke dalam ekosistem ROS2 di kontainer Docker (`vtol_dev`) melalui koneksi UDP port `5005`.

---

## 1. Arsitektur Aliran Video

```
┌─────────────────────────────────┐        Jaringan        ┌──────────────────────────────────┐
│      PENGIRIM (SENDER)          │         Local          │       DOCKER CONTAINER (DEV)      │
│  (Webots, Script Python, dll.)  ├───────────────────────►│  (Berjalan dengan network: host)  │
│  Membaca frame -> Encode JPEG   │       UDP:5005         │                                  │
│  -> Kirim paket UDP             │                        │  [ Node: aruco_receiver ]        │
└─────────────────────────────────┘                        │    - Bind socket ke UDP:5005     │
                                                           │    - Decode bytes -> Frame OpenCV│
                                                           │    - Deteksi ArUco               │
                                                           │    - Publish ke Topik ROS2       │
                                                           └────────────────┬─────────────────┘
                                                                            │
                                                                            ▼
                                                           ┌──────────────────────────────────┐
                                                           │           TOPIK ROS2             │
                                                           │  - /vtol/camera/image_raw        │
                                                           │  - /vtol/aruco/detection         │
                                                           └──────────────────────────────────┘
```

Karena kontainer Docker kita menggunakan **`network_mode: host`** (didefinisikan di [docker-compose.yml](file:///c:/Users/LENOVO/OneDrive/Documents/vtol_ros2/vtol_ros2/docker-compose.yml)), port jaringan kontainer terhubung langsung dengan Windows Host / WSL2. Dengan demikian, pengirim dapat mengirim data langsung ke IP Localhost (`127.0.0.1`).

---

## 2. Langkah-Langkah Setup & Deteksi Kamera

### Langkah 1: Siapkan Pengirim (UDP Stream Sender)
Jika Anda menggunakan **Webots**, pastikan modul kamera/robot Anda diatur untuk mengirimkan stream gambar terkompresi ke `127.0.0.1:5005`.

Jika Anda ingin menguji menggunakan **Kamera Fisik (Webcam)** dari Windows Host atau dari WSL, gunakan script Python contoh di bawah ini:

Save script ini di komputer Anda (misal `udp_sender.py`):
```python
import cv2
import socket
import time

# Konfigurasi UDP
UDP_IP = "127.0.0.1"  # IP penerima (Localhost/WSL IP)
UDP_PORT = 5005
MAX_DGRAM = 65507     # Batas maksimal ukuran paket UDP

# Inisialisasi Socket & Kamera
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cap = cv2.VideoCapture(0)  # 0 untuk webcam bawaan

# Set Resolusi agar ukuran paket aman
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print(f"Mengirim stream kamera ke {UDP_IP}:{UDP_PORT}...")

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
            
        # Encode frame ke format JPEG dengan kualitas 50% (untuk hemat bandwidth/ukuran paket)
        ret, encoded_img = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        if not ret:
            continue
            
        data = encoded_img.tobytes()
        size = len(data)
        
        # Kirim data jika ukuran paket di bawah batas maksimal UDP
        if size < MAX_DGRAM:
            sock.sendto(data, (UDP_IP, UDP_PORT))
        else:
            print(f"Frame terlalu besar ({size} bytes). Lewati frame.")
            
        time.sleep(1/30)  # Batasi fps ke ~30fps
except KeyboardInterrupt:
    print("\nStreaming dihentikan.")
finally:
    cap.release()
    sock.close()
```

### Langkah 2: Sesuaikan Konfigurasi Penerima (`vision_config.yaml`)
Sebelum menjalankan penerima di ROS2, pastikan berkas konfigurasi telah sesuai. 

Buka berkas konfigurasi: **[vision_config.yaml](file:///c:/Users/LENOVO/OneDrive/Documents/vtol_ros2/vtol_ros2/workspace/src/vtol_vision/config/vision_config.yaml)**

```yaml
camera:
  udp_ip: "127.0.0.1"         # Bind IP penerima (gunakan 0.0.0.0 untuk menerima dari semua IP luar)
  udp_port: 5005              # Port penerima UDP
  aruco_dict: "DICT_4X4_50"  # Dictionary ArUco yang ingin dideteksi
  show_gui: false             # Set true jika ingin menampilkan window popup OpenCV (memerlukan setup X11/GUI)
```

### Langkah 3: Bangun/Kompilasi Workspace ROS2
Jika Anda baru pertama kali menambahkan atau mengubah file di dalam package `vtol_vision`, lakukan kompilasi terlebih dahulu di dalam kontainer Docker `vtol_dev`:

1. Masuk ke kontainer Docker:
   ```bash
   docker exec -it vtol_dev bash
   ```
2. Jalankan perintah kompilasi:
   ```bash
   cd /home/pilot/workspace
   colcon build --packages-select vtol_vision
   ```
3. Lakukan refresh environment:
   ```bash
   source install/setup.bash
   ```

### Langkah 4: Jalankan Node Receiver
Di dalam kontainer Docker (`vtol_dev`), jalankan node penerima ArUco:
```bash
ros2 run vtol_vision aruco_receiver
```
**Log Sukses:**
Jika socket berhasil dibuat, terminal akan memunculkan log:
```text
[INFO] [aruco_receiver]: Socket UDP berhasil terikat pada 127.0.0.1:5005
[INFO] [aruco_receiver]: ArUco Receiver Node siap. Dict=DICT_4X4_50, Log=/home/pilot/workspace/aruco_vision.log
```

---

## 3. Langkah Pengujian & Verifikasi

Untuk memverifikasi apakah kamera telah berhasil terdeteksi dan diproses dengan benar oleh ROS2:

### Pengujian 1: Cek Pengiriman Data (Topik ROS2)
Buka terminal baru, masuk ke kontainer Docker, lalu cek apakah topik gambar raw dan deteksi ArUco mempublikasikan data:

1. **Cek Topik Gambar Raw:**
   ```bash
   ros2 topic hz /vtol/camera/image_raw
   ```
   *Jika sukses, akan muncul rate publikasi (misalnya sekitar `30.0 average rate`).*

2. **Cek Topik Deteksi ArUco:**
   ```bash
   ros2 topic echo /vtol/aruco/detection
   ```
   *Jika ada marker ArUco (dengan dictionary `DICT_4X4_50`) yang terdeteksi di kamera, topik ini akan mengeluarkan koordinat tengah marker dalam format JSON.*

### Pengujian 2: Cek Berkas Log Vision
Node penerima secara otomatis menulis riwayat deteksi ke berkas log. Anda dapat memantau isi log secara real-time:
```bash
tail -f /home/pilot/workspace/aruco_vision.log
```
**Format Log:**
* `DETECTED = YES` : Menandakan marker ArUco terdeteksi beserta ID dan koordinat ternormalisasinya.
* `DETECTED = NO` : Menandakan stream kamera terhubung tetapi tidak ada marker ArUco yang terlihat.
* `DECODE_FAIL` : Menandakan paket UDP rusak/tidak lengkap saat diterima.

### Pengujian 3: Visualisasi GUI (Opsional)
Jika Anda menggunakan Windows dengan WSLg (atau VcXsrv terinstal) dan ingin menampilkan pop-up window video OpenCV secara langsung:
1. Ubah konfigurasi `show_gui` pada **[vision_config.yaml](file:///c:/Users/LENOVO/OneDrive/Documents/vtol_ros2/vtol_ros2/workspace/src/vtol_vision/config/vision_config.yaml)** menjadi `true`.
2. Re-build/re-source workspace (jika diperlukan) lalu jalankan kembali node:
   ```bash
   ros2 run vtol_vision aruco_receiver
   ```
3. Window berjudul **"Webots ArUco Detection Stream"** akan muncul menampilkan gambar live dari feed kamera beserta kotak hijau di sekeliling marker ArUco yang terdeteksi.
