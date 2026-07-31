import socket
import cv2
import time
from picamera2 import Picamera2

# Inisialisasi Picamera2
picam = Picamera2()
config = picam.create_preview_configuration(main={"size": (640, 480)})
picam.configure(config)
picam.start()

# Setup socket UDP ke Docker (port 5005)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target_address = ('127.0.0.1', 5005)

print("Mulai streaming kamera Raspberry Pi 5 menggunakan Picamera2...")
try:
    while True:
        frame = picam.capture_array()
        ret, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 55])
        if not ret:
            continue
            
        data = encoded.tobytes()
        if len(data) < 65535:
            sock.sendto(data, target_address)
        
        time.sleep(0.03)
except KeyboardInterrupt:
    print("\nStreaming dihentikan.")
finally:
    picam.stop()
    sock.close()
