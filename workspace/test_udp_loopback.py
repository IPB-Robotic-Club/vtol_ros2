import socket
import threading
import time

def server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 12345))
    try:
        data, addr = sock.recvfrom(1024)
        print(f"Server received: {data.decode()} from {addr}")
        sock.sendto(b"PONG", addr)
    except Exception as e:
        print(f"Server error: {e}")

t = threading.Thread(target=server)
t.start()
time.sleep(0.5)

print("Client sending PING...")
try:
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(2.0)
    client.sendto(b"PING", ("127.0.0.1", 12345))
    data, addr = client.recvfrom(1024)
    print(f"Client received: {data.decode()} from {addr}")
except Exception as e:
    print(f"Client error: {e}")

t.join()
