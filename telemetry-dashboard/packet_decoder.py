import struct
from typing import List
from pydantic import BaseModel



class BallDataPacket(BaseModel):
    """Python representation of the C ball_data_packet structure."""
    timestamp: int
<<<<<<< HEAD
    magnet_current_values: List[List[float]]  # 20 x 300
    magnet_current_timestep: List[List[int]]  # 20 x 300
=======
    magnet_current_values: List[List[float]]  # 20 x 100
    magnet_current_timestep: List[List[int]]  # 20 x 100
>>>>>>> 38ad5286a888576933c39d6721e9c1c3678c4acd



def decode_ball_data_packet(raw_bytes: bytes) -> BallDataPacket:
    """
    Decode packed C struct into Python BallDataPacket.
    
    C Structure:
        typedef struct __attribute__((packed)) {
            int32_t timestamp;
<<<<<<< HEAD
            float magnet_current_values[20][300]; 
            int32_t magnet_current_timestep[20][300];
=======
            float magnet_current_values[20][100]; 
            int32_t magnet_current_timestep[20][100];
>>>>>>> 38ad5286a888576933c39d6721e9c1c3678c4acd
        } ball_data_packet;
    
    Layout:
        - 1 int32_t (4 bytes)
<<<<<<< HEAD
        - 6000 floats (24000 bytes)
        - 6000 int32_ts (24000 bytes)
        Total: 48,004 bytes
=======
        - 2000 floats (8000 bytes)
        - 2000 int32_ts (8000 bytes)
        Total: 16,004 bytes
>>>>>>> 38ad5286a888576933c39d6721e9c1c3678c4acd
    
    Args:
        raw_bytes: Raw packet bytes received over UDP
        
    Returns:
        BallDataPacket: Decoded packet
    """
<<<<<<< HEAD
    expected_size = 4 + (20 * 300 * 4) + (20 * 300 * 4)  # 48,004 bytes
=======
    expected_size = 4 + (20 * 100 * 4) + (20 * 100 * 4)  # 16,004 bytes
>>>>>>> 38ad5286a888576933c39d6721e9c1c3678c4acd
    
    if len(raw_bytes) != expected_size:
        raise ValueError(f"Expected {expected_size} bytes, got {len(raw_bytes)}")
    
    # Use little-endian format (ESP32 is little-endian)
    offset = 0
    
    # Unpack timestamp (int32_t, 4 bytes)
    timestamp = struct.unpack_from('<i', raw_bytes, offset)[0]
    offset += 4
    
<<<<<<< HEAD
    # Unpack magnet_current_values (6000 floats)
    magnet_current_values = []
    for _ in range(20):
        row = list(struct.unpack_from('<300f', raw_bytes, offset))
        magnet_current_values.append(row)
        offset += 1200  # 300 floats * 4 bytes
    
    # Unpack magnet_current_timestep (6000 int32_t)
    magnet_current_timestep = []
    for _ in range(20):
        row = list(struct.unpack_from('<300i', raw_bytes, offset))
        magnet_current_timestep.append(row)
        offset += 1200  # 300 int32_t * 4 bytes
=======
    # Unpack magnet_current_values (2000 floats)
    magnet_current_values = []
    for _ in range(20):
        row = list(struct.unpack_from('<100f', raw_bytes, offset))
        magnet_current_values.append(row)
        offset += 400  # 100 floats * 4 bytes
    
    # Unpack magnet_current_timestep (2000 int32_t)
    magnet_current_timestep = []
    for _ in range(20):
        row = list(struct.unpack_from('<100i', raw_bytes, offset))
        magnet_current_timestep.append(row)
        offset += 400  # 100 int32_t * 4 bytes
>>>>>>> 38ad5286a888576933c39d6721e9c1c3678c4acd
    
    return BallDataPacket(
        timestamp=timestamp,
        magnet_current_values=magnet_current_values,
        magnet_current_timestep=magnet_current_timestep
    )


# Example usage
if __name__ == "__main__":
    
    import socket
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 5005))
    
    print("Waiting for UDP packets on port 5005...")
    
    while True:
<<<<<<< HEAD
        data, addr = sock.recvfrom(50000)  # Large enough buffer
=======
        data, addr = sock.recvfrom(20000)  # Large enough buffer
>>>>>>> 38ad5286a888576933c39d6721e9c1c3678c4acd
        print(f"\nReceived {len(data)} bytes from {addr}")
        
        try:
            packet = decode_ball_data_packet(data)
            print(f"Timestamp: {packet.timestamp}")
            print(f"First magnet, first 5 current values: {packet.magnet_current_values[0][:5]}")
            print(f"First magnet, first 5 timesteps: {packet.magnet_current_timestep[0][:5]}")
        except ValueError as e:
            print(f"Error decoding packet: {e}")
