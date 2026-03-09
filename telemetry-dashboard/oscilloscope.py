import socket
import threading
import time
from collections import deque
from typing import List, Tuple
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from packet_decoder import decode_ball_data_packet


class MagnetDataBuffer:
    """Maintains a rolling buffer of magnet current data with 2-second history."""
    
    def __init__(self, time_window_ms: float = 6000.0):
        """
        Args:
            time_window_ms: Time window to display in milliseconds (default 2000ms = 2 seconds)
        """
        self.time_window_ms = time_window_ms
        # One deque per magnet to store (timestamp_ms, current_value) tuples
        self.magnet_buffers = [deque(maxlen=600000) for _ in range(20)]
        self.lock = threading.Lock()
    
    def add_packet(self, packet) -> None:
        """Add packet data to buffers, filtering out invalid timesteps."""
        with self.lock:
            for magnet_index in range(20):
                for sample_idx in range(300):
                    # Treat timestep == 0 as a non-value and render it as zero current.
                    timestep = packet.magnet_current_timestep[magnet_index][sample_idx]
                    if timestep == 0:
                        continue
                    
                    current = packet.magnet_current_values[magnet_index][sample_idx]
                    self.magnet_buffers[magnet_index].append((timestep, current))
            
        with open(f"data/{packet.timestamp}.txt", "w") as f:
            for magnet_index in range(20):
                for sample_idx in range(300):
                    timestep = packet.magnet_current_timestep[magnet_index][sample_idx]
                    current = packet.magnet_current_values[magnet_index][sample_idx]
                    f.write(f"{magnet_index},{timestep},{current}\n")
    
    def get_visible_data(self, current_time_ms: int) -> List[Tuple[List[int], List[float]]]:
        """
        Get data visible in current 2-second window.
        
        Args:
            current_time_ms: Current reference time in milliseconds
            
        Returns:
            List of (times, values) tuples for each magnet
        """
        with self.lock:
            visible_data = []
            time_start = current_time_ms - self.time_window_ms
            
            for magnet_idx in range(20):
                times = []
                values = []
                
                for timestamp_ms, current_val in self.magnet_buffers[magnet_idx]:
                    if time_start <= timestamp_ms <= current_time_ms:
                        times.append(timestamp_ms - time_start)  # Relative time
                        values.append(current_val)
                
                visible_data.append((times, values))
            
            return visible_data


class MagnetOscilloscope:
    """Real-time oscilloscope display for magnet currents."""
    
    def __init__(self, udp_port: int = 5005, time_window_ms: float = 2000.0, update_interval_ms: int = 20):
        self.udp_port = udp_port
        self.buffer = MagnetDataBuffer(time_window_ms=time_window_ms)
        self.socket = None
        self.is_running = False
        self.last_packet_time = None
        self.update_interval_ms = update_interval_ms
        
        # Create figure with 4x5 subplots (20 magnets)
        self.fig, self.axes = plt.subplots(4, 5, figsize=(16, 12))
        self.axes = self.axes.flatten()
        self.fig.suptitle('Magnet Current Oscilloscope (2s Window)', fontsize=16)
        
        # Configure each subplot
        for idx, ax in enumerate(self.axes):
            ax.set_title(f'Magnet {idx + 1}')
            ax.set_xlabel('Time (ms)')
            ax.set_ylabel('Current (A)')
            ax.grid(True, alpha=0.3)
            ax.set_xlim(0, time_window_ms)
            ax.set_ylim(-0.5, 5.0)  # Adjust as needed for your current range
        
        # Store line objects for animation
        self.lines = [ax.plot([], [], 'b-', linewidth=1.5)[0] for ax in self.axes]
    
    def start_udp_receiver(self):
        """Start receiving UDP packets in background thread."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", self.udp_port))
        self.socket.settimeout(1.0)  # 1 second timeout
        self.is_running = True
        
        thread = threading.Thread(target=self._receiver_loop, daemon=True)
        thread.start()
        print(f"UDP receiver started on port {self.udp_port}")
    
    def _receiver_loop(self):
        """Background thread that receives and decodes UDP packets."""
        while self.is_running:
            try:
                data, addr = self.socket.recvfrom(50000)
                print(f"Received packet from {addr}, size={len(data)} bytes")
                try:
                    packet = decode_ball_data_packet(data)
                    self.buffer.add_packet(packet)

                    # Keep plotting in the same time domain as packet sample timesteps.
                    self.last_packet_time = packet.timestamp
                except ValueError as e:
                    print(f"Decode error: {e}")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Receiver error: {e}")
    
    def _update_plot(self, frame):
        """Animation callback to update plot."""
        if self.last_packet_time is None:
            return self.lines
        
        # Get current visible data
        visible_data = self.buffer.get_visible_data(self.last_packet_time)
        
        # Update each subplot
        for magnet_idx, (times, values) in enumerate(visible_data):
            if times:
                self.lines[magnet_idx].set_data(times, values)
                # Auto-scale Y if needed
                if values:
                    min_val = min(values)
                    max_val = max(values)
                    margin = (max_val - min_val) * 0.1 if max_val > min_val else 0.5
                    self.axes[magnet_idx].set_ylim(-1, 14)
            else:
                self.lines[magnet_idx].set_data([], [])
        
        return self.lines
    
    def run(self):
        """Start the oscilloscope display."""
        self.start_udp_receiver()
        
        # Create animation
        ani = animation.FuncAnimation(
            self.fig,
            self._update_plot,
            interval=self.update_interval_ms,
            blit=False,
            cache_frame_data=False
        )
        
        plt.tight_layout()
        plt.show()
        
        # Cleanup
        self.is_running = False
        if self.socket:
            self.socket.close()


if __name__ == "__main__":
    oscilloscope = MagnetOscilloscope(udp_port=5005, time_window_ms=6000.0, update_interval_ms=1)
    oscilloscope.run()
