import struct
from typing import List
from pydantic import BaseModel



class BallDataPacket(BaseModel):
    """Python representation of the C ball_data_packet structure."""
    timestamp: int
    system_state: int
    orientation_wxyz: List[float]  # w,x,y,z
    angular_velocity_xyz: List[float]  # x,y,z
    magnet_setpoints: List[float]  # 20 elements
    magnet_current_values: List[float]  # 20 elements (single current value per magnet)
    magnet_current_timestep: List[int]  # 20 elements (timestamp for each magnet)


def decode_ball_data_packet(raw_bytes: bytes) -> BallDataPacket:
    """
    Decode packed C struct into Python BallDataPacket.
    
    Supports both new (20x1) and old (20x100) packet formats.
    
    C Structure (new):
        typedef struct __attribute__((packed)) {
            int32_t timestamp;
            uint8_t system_state;
            uint8_t reserved[3];
            float orientation_wxyz[4];
            float angular_velocity_xyz[3];
            float magnet_setpoints[20];
            float magnet_current_values[20];
            int32_t magnet_current_timestep[20];
        } ball_data_packet;

    Args:
        raw_bytes: Raw packet bytes received over UDP

    Returns:
        BallDataPacket: Decoded packet
    """
    new_size = (
        4 +  # timestamp
        4 +  # system_state + reserved
        4 * 4 +  # orientation
        4 * 3 +  # angular velocity
        4 * 20 +  # magnet_setpoints
        4 * 20 +  # magnet_current_values
        4 * 20    # magnet_current_timestep
    )
    
    old_size = (
        4 +  # timestamp
        4 +  # system_state + reserved
        4 * 4 +  # orientation
        4 * 3 +  # angular velocity
        4 * 20 +  # magnet_setpoints
        20 * 100 * 4 +  # magnet_current_values (20x100)
        20 * 100 * 4    # magnet_current_timestep (20x100)
    )

    if len(raw_bytes) == new_size:
        return _decode_new_format(raw_bytes)
    elif len(raw_bytes) == old_size:
        return _decode_old_format(raw_bytes)
    else:
        raise ValueError(f"Expected {new_size} or {old_size} bytes, got {len(raw_bytes)}")


def _decode_new_format(raw_bytes: bytes) -> BallDataPacket:
    """Decode new packet format (20x1 magnet data)."""
    offset = 0

    timestamp = struct.unpack_from('<i', raw_bytes, offset)[0]
    offset += 4

    system_state = struct.unpack_from('<B', raw_bytes, offset)[0]
    offset += 1

    # reserved 3 bytes
    offset += 3

    orientation_wxyz = list(struct.unpack_from('<4f', raw_bytes, offset))
    offset += 16

    angular_velocity_xyz = list(struct.unpack_from('<3f', raw_bytes, offset))
    offset += 12

    magnet_setpoints = list(struct.unpack_from('<20f', raw_bytes, offset))
    offset += 80

    magnet_current_values = list(struct.unpack_from('<20f', raw_bytes, offset))
    offset += 80

    magnet_current_timestep = list(struct.unpack_from('<20i', raw_bytes, offset))
    offset += 80

    return BallDataPacket(
        timestamp=timestamp,
        system_state=system_state,
        orientation_wxyz=orientation_wxyz,
        angular_velocity_xyz=angular_velocity_xyz,
        magnet_setpoints=magnet_setpoints,
        magnet_current_values=magnet_current_values,
        magnet_current_timestep=magnet_current_timestep,
    )


def _decode_old_format(raw_bytes: bytes) -> BallDataPacket:
    """Decode old packet format (20x100 magnet data), taking only the most recent value."""
    offset = 0

    timestamp = struct.unpack_from('<i', raw_bytes, offset)[0]
    offset += 4

    system_state = struct.unpack_from('<B', raw_bytes, offset)[0]
    offset += 1

    # reserved 3 bytes
    offset += 3

    orientation_wxyz = list(struct.unpack_from('<4f', raw_bytes, offset))
    offset += 16

    angular_velocity_xyz = list(struct.unpack_from('<3f', raw_bytes, offset))
    offset += 12

    magnet_setpoints = list(struct.unpack_from('<20f', raw_bytes, offset))
    offset += 80

    # Old format: 20x100 arrays, take only the last value from each magnet's 100 samples
    magnet_current_values = []
    for _ in range(20):
        row = list(struct.unpack_from('<100f', raw_bytes, offset))
        magnet_current_values.append(row[-1])  # Take most recent (last) value
        offset += 400

    magnet_current_timestep = []
    for _ in range(20):
        row = list(struct.unpack_from('<100i', raw_bytes, offset))
        magnet_current_timestep.append(row[-1])  # Take most recent (last) timestep
        offset += 400

    return BallDataPacket(
        timestamp=timestamp,
        system_state=system_state,
        orientation_wxyz=orientation_wxyz,
        angular_velocity_xyz=angular_velocity_xyz,
        magnet_setpoints=magnet_setpoints,
        magnet_current_values=magnet_current_values,
        magnet_current_timestep=magnet_current_timestep,
    )


if __name__ == "__main__":
    
    import socket
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 5005))
    
    print("Waiting for UDP packets on port 5005...")
    print("(Supports both new 20x1 and old 20x100 packet formats)")
    
    while True:
        data, addr = sock.recvfrom(20000)  # Large enough buffer
        print(f"\nReceived {len(data)} bytes from {addr}")
        
        try:
            packet = decode_ball_data_packet(data)
            print(f"Timestamp: {packet.timestamp}")
            print(f"Magnet currents (first 5): {packet.magnet_current_values[:5]}")
            print(f"Magnet timesteps (first 5): {packet.magnet_current_timestep[:5]}")
        except ValueError as e:
            print(f"Error decoding packet: {e}")
