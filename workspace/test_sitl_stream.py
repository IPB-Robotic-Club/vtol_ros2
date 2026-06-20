import socket
import time

print("Connecting to 127.0.0.1:5762...")
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    s.connect(("127.0.0.1", 5762))
    print("Connected successfully! Waiting for data...")
    
    start_time = time.time()
    data_received = b""
    while time.time() - start_time < 5.0:
        try:
            chunk = s.recv(1024)
            if chunk:
                data_received += chunk
                print(f"Received {len(chunk)} bytes.")
                break
        except socket.timeout:
            pass
            
    if data_received:
        print(f"SITL telemetry stream is ACTIVE! Received total {len(data_received)} bytes.")
    else:
        print("SITL telemetry stream is SILENT (No data received for 5 seconds). It is frozen or paused!")
except Exception as e:
    print(f"Connection failed: {e}")
