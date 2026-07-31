import urllib.request
import numpy as np
import cv2
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json

PORT = 8087
STREAM_URL = "http://localhost:8086/stream"
SAVE_DIR = "/home/vtol/vtol_ros2/calibration_images"

if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

latest_frame = None
frame_lock = threading.Lock()
running = True
img_counter = 1

# Inisialisasi counter file gambar
while os.path.exists(os.path.join(SAVE_DIR, f"calib_{img_counter:02d}.png")):
    img_counter += 1

def stream_reader():
    global latest_frame, running
    while running:
        try:
            print(f"Menghubungkan ke stream kamera di {STREAM_URL}...")
            stream = urllib.request.urlopen(STREAM_URL, timeout=5)
            bytes_data = bytes()
            while running:
                bytes_data += stream.read(4096)
                a = bytes_data.find(b'\xff\xd8')
                b = bytes_data.find(b'\xff\xd9')
                if a != -1 and b != -1:
                    jpg_data = bytes_data[a:b+2]
                    bytes_data = bytes_data[b+2:]
                    frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        with frame_lock:
                            latest_frame = frame
        except Exception as e:
            print(f"Koneksi stream gagal/putus: {e}. Menghubungkan kembali dalam 2 detik...")
            time.sleep(2)

# Jalankan thread pembaca stream
thread = threading.Thread(target=stream_reader, daemon=True)
thread.start()

class WebCaptureHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global img_counter
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            
            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>VTOL Calibration Capture Tool</title>
                <style>
                    body {
                        font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
                        background-color: #121212;
                        color: #ffffff;
                        text-align: center;
                        padding: 20px;
                    }
                    .container {
                        max-width: 800px;
                        margin: 0 auto;
                        background: #1e1e1e;
                        padding: 30px;
                        border-radius: 12px;
                        box-shadow: 0 6px 12px rgba(0,0,0,0.5);
                    }
                    h1 { color: #00bcd4; margin-bottom: 20px; }
                    img {
                        max-width: 100%;
                        border: 4px solid #333;
                        border-radius: 8px;
                        margin-bottom: 25px;
                    }
                    .btn {
                        background-color: #e91e63;
                        color: white;
                        border: none;
                        padding: 15px 40px;
                        font-size: 20px;
                        font-weight: bold;
                        border-radius: 30px;
                        cursor: pointer;
                        box-shadow: 0 4px 6px rgba(0,0,0,0.3);
                        transition: all 0.2s;
                    }
                    .btn:hover {
                        background-color: #c2185b;
                        transform: scale(1.05);
                    }
                    .btn:active {
                        transform: scale(0.95);
                    }
                    #status {
                        margin-top: 20px;
                        font-size: 18px;
                        font-weight: 500;
                        color: #4caf50;
                    }
                </style>
            </head>
            <body>
                <div class="container">
                    <h1>VTOL Camera Capture Tool</h1>
                    <img src="/local_stream" alt="Camera Stream" />
                    <br/>
                    <button class="btn" onclick="captureImage()">CAPTURE IMAGE</button>
                    <div id="status">Ready</div>
                </div>

                <script>
                    function captureImage() {
                        const statusDiv = document.getElementById('status');
                        statusDiv.innerText = "Capturing...";
                        statusDiv.style.color = "#ffeb3b";
                        
                        fetch('/capture')
                            .then(response => response.json())
                            .then(data => {
                                if (data.success) {
                                    statusDiv.innerText = "[SUKSES] Gambar disimpan: " + data.filename;
                                    statusDiv.style.color = "#4caf50";
                                } else {
                                    statusDiv.innerText = "[ERROR] Gagal: " + data.message;
                                    statusDiv.style.color = "#f44336";
                                }
                            })
                            .catch(error => {
                                statusDiv.innerText = "[ERROR] Hubungan ke server terputus.";
                                statusDiv.style.color = "#f44336";
                            });
                    }
                </script>
            </body>
            </html>
            """
            self.wfile.write(html.encode('utf-8'))
            
        elif self.path == '/local_stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while running:
                    with frame_lock:
                        if latest_frame is not None:
                            ret, jpeg = cv2.imencode('.jpg', latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                            if ret:
                                self.wfile.write(b'--frame\r\n')
                                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode('ascii'))
                                self.wfile.write(jpeg.tobytes())
                                self.wfile.write(b'\r\n')
                    time.sleep(0.04)  # ~25 FPS
            except Exception as e:
                pass
                
        elif self.path == '/capture':
            try:
                with frame_lock:
                    if latest_frame is not None:
                        filename = f"calib_{img_counter:02d}.png"
                        filepath = os.path.join(SAVE_DIR, filename)
                        cv2.imwrite(filepath, latest_frame)
                        img_counter += 1
                        
                        self.send_response(200)
                        self.send_header('Content-type', 'application/json')
                        self.end_headers()
                        self.wfile.write(json.dumps({"success": True, "filename": filename}).encode('utf-8'))
                        print(f"Captured: {filename}")
                        return
                    else:
                        raise Exception("Belum ada frame kamera di memori")
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"success": False, "message": str(e)}).encode('utf-8'))

# Menggunakan ThreadingHTTPServer untuk dukungan multi-thread
server = ThreadingHTTPServer(('0.0.0.0', PORT), WebCaptureHandler)
print(f"Web Capture UI aktif di http://localhost:{PORT}")
try:
    server.serve_forever()
except KeyboardInterrupt:
    running = False
    print("Server dihentikan.")
