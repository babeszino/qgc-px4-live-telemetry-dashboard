import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', 14551))
s.settimeout(5)
print("Listening...")
try:
    data, addr = s.recvfrom(4096)
    print(f"GOT {len(data)} bytes from {addr}")
except:
    print("NOTHING received")
s.close()