import socket
import struct

UDP_IP = "0.0.0.0" # Listen on all interfaces
UDP_PORT = 5005

# This 'ffi' string matches the C struct: 
# f = float (4 bytes), i = int (4 bytes)
# '<' means Little Endian (standard for ESP32/PC)
data_format = "<ffi" 

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for ESP32 data on port {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(12) # buffer size is 1024 bytes
    
    # Unpack the binary data
    unpacked_data = struct.unpack(data_format, data)
    
    print(f"Received from {addr}: unpacked_data={unpacked_data}")