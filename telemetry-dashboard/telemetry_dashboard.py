import socket
import struct
import threading
import time
from typing import Optional
from packet_decoder import decode_ball_data_packet

# Consider ESP disconnected if no telemetry received for this many seconds
CONNECTION_TIMEOUT_S = 3.0

class BallControllerDashboard:
    def __init__(self, esp_ip: str = "192.168.4.1"):
        """
        Initialize dashboard client
        
        Args:
            esp_ip: IP address of ESP32 (default is AP mode IP)
        """
        self.esp_ip = esp_ip
        self.telemetry_port = 5005
        self.command_port = 5006
        
        # Receiver socket for telemetry
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx_sock.bind(("0.0.0.0", self.telemetry_port))
        
        # Sender socket for commands
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.running = False
        self.latest_telemetry = None
        self.sequence_number = 0
        self._last_telemetry_time: Optional[float] = None
        
    def _receive_telemetry_thread(self):
        """Background thread to receive telemetry data"""
        while self.running:
            try:
                data, addr = self.rx_sock.recvfrom(20000)
                # Mark as connected whenever we receive any data from the ESP
                self._last_telemetry_time = time.monotonic()
                try:
                    packet = decode_ball_data_packet(data)
                    self.latest_telemetry = packet
                except ValueError as e:
                    print(f"[ERROR] Decode failed ({len(data)} bytes): {e}")
            except socket.timeout:
                pass
            except Exception as e:
                print(f"[ERROR] Receive error: {e}")
    
    def start(self):
        """Start receiving telemetry"""
        self.running = True
        self.rx_sock.settimeout(1.0)
        
        rx_thread = threading.Thread(target=self._receive_telemetry_thread, daemon=True)
        rx_thread.start()
        print(f"Connected to ESP32 at {self.esp_ip}")
        return self
    
    @property
    def is_connected(self) -> bool:
        """True if telemetry packets have been received recently."""
        if self._last_telemetry_time is None:
            return False
        return (time.monotonic() - self._last_telemetry_time) < CONNECTION_TIMEOUT_S

    def stop(self):
        """Stop the dashboard"""
        self.running = False
        time.sleep(0.5)
    
    def _send_command(self, command_type: int, x: float = 0, y: float = 0, z: float = 1):
        """
        Send command to ESP32
        
        Args:
            command_type: 0=direction, 1=calibrate, 2=emergency_stop, 3=start_running
            x, y, z: Direction vector components
        """
        self.sequence_number += 1
        
        # Pack command: uint8 + 3xfloat + uint32
        cmd_packet = struct.pack('<BfffI',
            command_type,
            x, y, z,
            self.sequence_number
        )
        
        try:
            self.tx_sock.sendto(cmd_packet, (self.esp_ip, self.command_port))
            print(f"[COMMAND] Sent command type {command_type}: ({x:.2f}, {y:.2f}, {z:.2f})")
        except Exception as e:
            print(f"[ERROR] Failed to send command: {e}")
    
    def set_direction(self, x: float, y: float, z: float):
        """Set ideal direction for ball control"""
        self._send_command(command_type=0, x=x, y=y, z=z)
    
    def calibrate(self):
        """Send calibration command"""
        self._send_command(command_type=1)
    
    def stop_running(self):
        """Stop running and return to standby"""
        self._send_command(command_type=2)
    
    def start_running(self):
        """Start/resume running"""
        self._send_command(command_type=3)
    
    def get_telemetry(self):
        """Get latest telemetry data"""
        return self.latest_telemetry


# Example usage
if __name__ == "__main__":
    # Connect to ESP32
    dashboard = BallControllerDashboard(esp_ip="192.168.4.1").start()
    
    try:
        # Calibrate first
        print("\n1. Sending calibration command...")
        dashboard.calibrate()
        time.sleep(2)
        
        # Start running
        print("\n2. Starting system...")
        dashboard.start_running()
        time.sleep(2)
        
        # Send steering commands
        print("\n3. Sending steering commands...")
        dashboard.set_direction(1.0, 0.0, 0.0)  # Roll forward
        time.sleep(2)
        
        dashboard.set_direction(0.0, 1.0, 0.0)  # Roll left
        time.sleep(2)
        
        dashboard.set_direction(0.0, 0.0, 1.0)  # Neutral
        time.sleep(2)
        
        # Check telemetry
        print("\n4. Latest telemetry:")
        telem = dashboard.get_telemetry()
        if telem:
            print(f"   Timestamp: {telem.timestamp}ms")
            print(f"   Magnet 1 current values: {telem.magnet_current_values[0][:5]}")
        
        # Emergency stop
        print("\n5. Emergency stop...")
        dashboard.emergency_stop()
        time.sleep(1)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        dashboard.stop()