import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import math
from telemetry_dashboard import BallControllerDashboard
from ota_upload import upload_firmware

class DashboardGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Ball Controller Telemetry Dashboard")
        self.root.geometry("1000x700")
        self.root.configure(bg="#1e1e1e")
        
        self.dashboard = BallControllerDashboard(esp_ip="192.168.4.1")
        self.dashboard.start()
        
        # System state tracking (driven by telemetry, not button presses)
        self.system_state = "Disconnected"
        self._prev_telemetry_text = ""
        
        # Joystick state
        self.joystick_x = 0.0
        self.joystick_y = 0.0

        # Simulated ball model state (relative to calibrated joystick)
        self.ball_position = [0.0, 0.0]
        self.ball_velocity = [0.0, 0.0]
        self.calibration_offset_deg = 0.0  # degrees of correction

        # Magnet data storage
        self.magnet_currents = [0.0] * 20
        self.magnet_setpoints = [0.0] * 20
        
        self.setup_ui()
        self.update_telemetry()
        
    def setup_ui(self):
        """Create the GUI elements"""
        
        # Title
        title_label = tk.Label(self.root, text="Ball Controller Dashboard", 
                               font=("Arial", 20, "bold"), bg="#1e1e1e", fg="#00ff00")
        title_label.pack(pady=10)
        
        # Main container
        main_frame = tk.Frame(self.root, bg="#1e1e1e")
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left side - Joystick
        left_frame = tk.Frame(main_frame, bg="#1e1e1e")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10)
        
        # Joystick label
        joystick_label = tk.Label(left_frame, text="Joystick Input", 
                                  font=("Arial", 14, "bold"), bg="#1e1e1e", fg="#00ff00")
        joystick_label.pack(pady=10)
        
        # Joystick canvas (click/drag to control)
        self.joystick_canvas = tk.Canvas(left_frame, width=300, height=300, 
                                         bg="#2a2a2a", highlightthickness=2, highlightbackground="#00ff00")
        self.joystick_canvas.pack(pady=10)
        self.joystick_canvas.bind("<Button-1>", self.on_joystick_click)
        self.joystick_canvas.bind("<B1-Motion>", self.on_joystick_drag)
        self.joystick_canvas.bind("<ButtonRelease-1>", self.on_joystick_release)
        
        # Draw joystick circles
        self.draw_joystick()
        
        # Joystick value display
        self.joystick_value_label = tk.Label(left_frame, 
                                             text="X: 0.00  Y: 0.00", 
                                             font=("Arial", 12), bg="#1e1e1e", fg="#00ff00")
        self.joystick_value_label.pack(pady=5)
        
        # Ball model canvas
        self.ball_canvas = tk.Canvas(left_frame, width=300, height=300,
                                     bg="#111111", highlightthickness=2, highlightbackground="#00ff00")
        self.ball_canvas.pack(pady=10)
        self.draw_ball_model()

        self.ball_info_label = tk.Label(left_frame, text="Position: (0.00, 0.00) Velocity: (0.00, 0.00)",
                                        font=("Arial", 11), bg="#1e1e1e", fg="#00ff00")
        self.ball_info_label.pack(pady=5)

        # Keyboard hint
        hint_label = tk.Label(left_frame, text="Click and drag to move joystick\nOr use WASD keys", 
                             font=("Arial", 10), bg="#1e1e1e", fg="#888888", justify=tk.CENTER)
        hint_label.pack(pady=5)
        
        # Right side - Controls and status
        right_frame = tk.Frame(main_frame, bg="#1e1e1e")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10)
        
        # Status frame
        status_frame = tk.LabelFrame(right_frame, text="Status", 
                                     font=("Arial", 12, "bold"), bg="#2a2a2a", fg="#00ff00")
        status_frame.pack(fill=tk.X, pady=10)
        
        self.connection_label = tk.Label(status_frame, text="\u25cf Disconnected",
                                        font=("Arial", 11, "bold"), bg="#2a2a2a", fg="#ff4444", justify=tk.LEFT)
        self.connection_label.pack(padx=10, pady=5)

        self.status_label = tk.Label(status_frame, text="Waiting for ESP32 telemetry...", 
                                     font=("Arial", 10), bg="#2a2a2a", fg="#888888", justify=tk.LEFT)
        self.status_label.pack(padx=10, pady=5)

        self.state_label = tk.Label(status_frame, text="State: Disconnected",
                                    font=("Arial", 10, "bold"), bg="#2a2a2a", fg="#888888", justify=tk.LEFT)
        self.state_label.pack(padx=10, pady=5)
        
        # Control buttons frame
        button_frame = tk.LabelFrame(right_frame, text="Controls", 
                                     font=("Arial", 12, "bold"), bg="#2a2a2a", fg="#00ff00")
        button_frame.pack(fill=tk.X, pady=10)
        
        # Calibrate button (green)
        self.calibrate_btn = tk.Button(button_frame, text="CALIBRATE", 
                                       command=self.on_calibrate,
                                       font=("Arial", 12, "bold"), 
                                       bg="#00aa00", fg="black", width=20, height=2)
        self.calibrate_btn.pack(padx=10, pady=5)
        
        # Start button
        self.start_btn = tk.Button(button_frame, text="START", 
                                   command=self.on_start,
                                   font=("Arial", 12, "bold"), 
                                   bg="#0088ff", fg="white", width=20, height=2)
        self.start_btn.pack(padx=10, pady=5)
        
        # Send direction button
        self.send_direction_btn = tk.Button(button_frame, text="SEND DIRECTION", 
                                           command=self.on_send_direction,
                                           font=("Arial", 12, "bold"), 
                                           bg="#ffaa00", fg="black", width=20, height=2)
        self.send_direction_btn.pack(padx=10, pady=5)
        
        # Stop running button (red)
        self.stop_btn = tk.Button(button_frame, text="STOP RUNNING", 
                                       command=self.on_stop_running,
                                       font=("Arial", 14, "bold"), 
                                       bg="#ff0000", fg="white", width=20, height=2)
        self.stop_btn.pack(padx=10, pady=10)
        
        # OTA flash button
        self.ota_btn = tk.Button(button_frame, text="FLASH FIRMWARE (OTA)",
                                 command=self.on_ota_upload,
                                 font=("Arial", 11, "bold"),
                                 bg="#8800cc", fg="white", width=20, height=2)
        self.ota_btn.pack(padx=10, pady=5)
        
        # Magnet currents frame
        magnet_frame = tk.LabelFrame(right_frame, text="Magnets", 
                                      font=("Arial", 12, "bold"), bg="#2a2a2a", fg="#00ff00")
        magnet_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        self.magnet_list = ttk.Treeview(magnet_frame, columns=("magnet", "current", "setpoint"),
                                         show="headings", height=10)
        self.magnet_list.heading("magnet", text="#")
        self.magnet_list.heading("current", text="Current")
        self.magnet_list.heading("setpoint", text="Setpoint")
        self.magnet_list.column("magnet", width=40, anchor="center")
        self.magnet_list.column("current", width=80, anchor="center")
        self.magnet_list.column("setpoint", width=80, anchor="center")

        mag_scroll = ttk.Scrollbar(magnet_frame, orient="vertical", command=self.magnet_list.yview)
        self.magnet_list.configure(yscrollcommand=mag_scroll.set)
        self.magnet_list.pack(side=tk.LEFT, padx=(10, 0), pady=10, fill=tk.BOTH, expand=True)
        mag_scroll.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 10), pady=10)

        for i in range(20):
            self.magnet_list.insert("", "end", iid=f"mag{i}", values=(i + 1, "0.00", "0.00"))

        # Telemetry frame
        telemetry_frame = tk.LabelFrame(right_frame, text="Telemetry", 
                                        font=("Arial", 12, "bold"), bg="#2a2a2a", fg="#00ff00")
        telemetry_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Telemetry text display
        self.telemetry_text = tk.Text(telemetry_frame, height=10, width=40,
                                      font=("Courier", 9), bg="#1a1a1a", fg="#00ff00",
                                      highlightthickness=0)
        self.telemetry_text.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
        self.telemetry_text.config(state=tk.DISABLED)  # Read-only
        
        # Bind keyboard controls
        self.root.bind("<w>", self.on_key_w)
        self.root.bind("<a>", self.on_key_a)
        self.root.bind("<s>", self.on_key_s)
        self.root.bind("<d>", self.on_key_d)
        self.root.bind("<KeyRelease>", self.on_key_release)
    
    def draw_joystick(self):
        """Draw the joystick background"""
        self.joystick_canvas.delete("all")
        
        # Draw border
        self.joystick_canvas.create_rectangle(10, 10, 290, 290, outline="#00ff00", width=2)
        
        # Draw crosshairs
        self.joystick_canvas.create_line(150, 10, 150, 290, fill="#444444", width=1)
        self.joystick_canvas.create_line(10, 150, 290, 150, fill="#444444", width=1)
        
        # Draw center circle
        self.joystick_canvas.create_oval(140, 140, 160, 160, outline="#666666", width=1)
        
        # Calculate joystick position
        center_x, center_y = 150, 150
        radius = 120
        
        stick_x = center_x + self.joystick_x * radius
        stick_y = center_y - self.joystick_y * radius  # Inverted Y
        
        # Clamp to circle
        dx = stick_x - center_x
        dy = stick_y - center_y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > radius:
            stick_x = center_x + (dx / dist) * radius
            stick_y = center_y + (dy / dist) * radius
        
        # Draw stick
        self.joystick_canvas.create_line(center_x, center_y, stick_x, stick_y, 
                                         fill="#00ff00", width=3)
        
        # Draw stick ball
        ball_size = 15
        self.joystick_canvas.create_oval(stick_x - ball_size, stick_y - ball_size,
                                         stick_x + ball_size, stick_y + ball_size,
                                         fill="#00ff00", outline="#00aa00", width=2)

    def draw_ball_model(self):
        """Draw 2D ball position/velocity model"""
        self.ball_canvas.delete("all")

        # center
        cx, cy = 150, 150
        self.ball_canvas.create_oval(cx-120, cy-120, cx+120, cy+120, outline="#00ff00", width=2)
        self.ball_canvas.create_line(cx, cy-120, cx, cy+120, fill="#444444")
        self.ball_canvas.create_line(cx-120, cy, cx+120, cy, fill="#444444")

        # draw reference joystick vector
        jsx = cx + self.joystick_x * 110
        jsy = cy - self.joystick_y * 110
        self.ball_canvas.create_line(cx, cy, jsx, jsy, fill="#00aa00", width=2, arrow=tk.LAST)

        # apply calibration offset rotation to joystick vector
        angle = math.radians(self.calibration_offset_deg)
        c, s = math.cos(angle), math.sin(angle)
        rx = self.joystick_x * c - self.joystick_y * s
        ry = self.joystick_x * s + self.joystick_y * c
        bx = cx + rx * 110
        by = cy - ry * 110
        self.ball_canvas.create_line(cx, cy, bx, by, fill="#ffdd00", width=2, dash=(4,2), arrow=tk.LAST)

        # update position from simulated state
        posx = cx + self.ball_position[0] * 100
        posy = cy - self.ball_position[1] * 100
        self.ball_canvas.create_oval(posx-8, posy-8, posx+8, posy+8, fill="#ff0000")

        # velocity vector
        vtx = posx + self.ball_velocity[0] * 100
        vty = posy - self.ball_velocity[1] * 100
        self.ball_canvas.create_line(posx, posy, vtx, vty, fill="#00ffff", width=2)
    
    def on_joystick_click(self, event):
        """Handle joystick click"""
        self.update_joystick_position(event.x, event.y)
    
    def on_joystick_drag(self, event):
        """Handle joystick drag"""
        self.update_joystick_position(event.x, event.y)
    
    def on_joystick_release(self, event):
        """Handle joystick release - reset to center"""
        self.joystick_x = 0.0
        self.joystick_y = 0.0
        self.draw_joystick()
        self.joystick_value_label.config(text="X: 0.00  Y: 0.00")
    
    def update_joystick_position(self, canvas_x, canvas_y):
        """Update joystick position from canvas coordinates"""
        center_x, center_y = 150, 150
        radius = 120
        
        # Calculate relative position
        dx = canvas_x - center_x
        dy = center_y - canvas_y  # Inverted Y
        
        # Normalize to unit circle
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > 0:
            self.joystick_x = min(1.0, dx / radius)
            self.joystick_y = min(1.0, dy / radius)
            
            # Clamp to unit circle
            mag = math.sqrt(self.joystick_x**2 + self.joystick_y**2)
            if mag > 1.0:
                self.joystick_x /= mag
                self.joystick_y /= mag
        
        self.draw_joystick()
        self.joystick_value_label.config(text=f"X: {self.joystick_x:.2f}  Y: {self.joystick_y:.2f}")
    
    def on_key_w(self, event):
        """Handle W key (up)"""
        self.joystick_y = 1.0
        self.draw_joystick()
        self.joystick_value_label.config(text=f"X: {self.joystick_x:.2f}  Y: {self.joystick_y:.2f}")
    
    def on_key_a(self, event):
        """Handle A key (left)"""
        self.joystick_x = -1.0
        self.draw_joystick()
        self.joystick_value_label.config(text=f"X: {self.joystick_x:.2f}  Y: {self.joystick_y:.2f}")
    
    def on_key_s(self, event):
        """Handle S key (down)"""
        self.joystick_y = -1.0
        self.draw_joystick()
        self.joystick_value_label.config(text=f"X: {self.joystick_x:.2f}  Y: {self.joystick_y:.2f}")
    
    def on_key_d(self, event):
        """Handle D key (right)"""
        self.joystick_x = 1.0
        self.draw_joystick()
        self.joystick_value_label.config(text=f"X: {self.joystick_x:.2f}  Y: {self.joystick_y:.2f}")
    
    def on_key_release(self, event):
        """Reset joystick on key release"""
        if event.keysym in ['w', 'a', 's', 'd']:
            self.joystick_x = 0.0
            self.joystick_y = 0.0
            self.draw_joystick()
            self.joystick_value_label.config(text="X: 0.00  Y: 0.00")
    
    def _check_connected(self, action_name: str) -> bool:
        """Return True if ESP32 is connected, else show warning."""
        if not self.dashboard.is_connected:
            messagebox.showwarning("Not Connected",
                                   f"Cannot {action_name}: no telemetry from ESP32.\n"
                                   "Make sure you are connected to the ESP32 WiFi network.")
            return False
        return True

    def on_calibrate(self):
        """Calibrate button pressed"""
        if not self._check_connected("calibrate"):
            return
        self.dashboard.calibrate()
        self.update_status("Calibration command sent")

    def on_start(self):
        """Start button pressed"""
        if not self._check_connected("start"):
            return
        self.dashboard.start_running()
        self.update_status("Start command sent")

    def on_send_direction(self):
        """Send current joystick direction"""
        if not self._check_connected("send direction"):
            return
        # Normalize to unit vector (or zero if center)
        magnitude = math.sqrt(self.joystick_x**2 + self.joystick_y**2)
        
        if magnitude > 0.01:  # Deadzone
            x = self.joystick_x / magnitude
            y = self.joystick_y / magnitude
        else:
            x = 0.0
            y = 0.0

        # Setpoint for magnets: approximate as normalized target magnitude
        for i in range(20):
            self.magnet_setpoints[i] = magnitude * 1.0  # placeholder scale

        self.dashboard.set_direction(x, y, 0.0)
        self.update_status(f"Sent direction: ({x:.2f}, {y:.2f})")

        # Simulate physics in running mode to reflect on ball model
        if self.system_state == "Running":
            self.ball_velocity[0] = x * 0.5  # scaled velocity
            self.ball_velocity[1] = y * 0.5

    def on_stop_running(self):
        """Stop running button pressed - always allowed even if disconnected"""
        self.dashboard.stop_running()
        self.ball_velocity = [0.0, 0.0]
        self.ball_position = [0.0, 0.0]
        self.update_status("Stop running sent")

    def on_ota_upload(self):
        """Open file dialog and upload firmware via OTA"""
        if not self._check_connected("flash firmware"):
            return

        if self.system_state not in ("Standby", "Connection"):
            messagebox.showwarning("Wrong State",
                                   f"OTA flash is only safe in Standby state.\n"
                                   f"Current state: {self.system_state}")
            return

        firmware_path = filedialog.askopenfilename(
            title="Select firmware binary",
            filetypes=[("Binary files", "*.bin"), ("All files", "*.*")],
            initialdir=".."
        )
        if not firmware_path:
            return

        if not messagebox.askyesno("Confirm OTA Flash",
                                    f"Flash firmware:\n{firmware_path}\n\n"
                                    "The ESP32 will reboot after flashing.\nContinue?"):
            return

        self.ota_btn.config(state=tk.DISABLED, text="FLASHING...")
        self.update_status("OTA upload in progress...")

        def do_upload():
            success = upload_firmware(firmware_path, self.dashboard.esp_ip)
            self.root.after(0, lambda: self._ota_complete(success))

        threading.Thread(target=do_upload, daemon=True).start()

    def _ota_complete(self, success):
        """Called on main thread when OTA upload finishes"""
        self.ota_btn.config(state=tk.NORMAL, text="FLASH FIRMWARE (OTA)")
        if success:
            self.update_status("OTA flash successful! ESP32 is rebooting...")
            messagebox.showinfo("OTA Success", "Firmware flashed successfully.\nESP32 is rebooting.")
        else:
            self.update_status("OTA flash failed")
            messagebox.showerror("OTA Failed", "Firmware upload failed.\nCheck console for details.")

    def update_status(self, message):
        """Update status label"""
        self.status_label.config(text=message)
    
    def update_telemetry(self):
        """Update telemetry display"""

        # --- Connection indicator ---
        connected = self.dashboard.is_connected
        if connected:
            self.connection_label.config(text="\u25cf Connected", fg="#00ff00")
        else:
            self.connection_label.config(text="\u25cf Disconnected", fg="#ff4444")
            if self.system_state != "Disconnected":
                self.system_state = "Disconnected"
                self.state_label.config(text="State: Disconnected", fg="#888888")
                self.status_label.config(text="Waiting for ESP32 telemetry...")

        # Animate ball physics (relative to joystick + calibration offset)
        if self.system_state == "Running":
            # simple discrete integration
            self.ball_position[0] += self.ball_velocity[0] * 0.05
            self.ball_position[1] += self.ball_velocity[1] * 0.05
            # clamp to bounding area [-1,+1]
            self.ball_position[0] = max(-1.0, min(1.0, self.ball_position[0]))
            self.ball_position[1] = max(-1.0, min(1.0, self.ball_position[1]))

        self.draw_ball_model()
        self.ball_info_label.config(text=f"Position: ({self.ball_position[0]:.2f}, {self.ball_position[1]:.2f}) ")

        if self.dashboard.latest_telemetry and connected:
            telem = self.dashboard.latest_telemetry

            # Update state from ESP telemetry (authoritative source)
            state_map = {
                0: "Connection",
                1: "Standby",
                2: "Calibration",
                3: "Running",
                4: "Testing"
            }
            self.system_state = state_map.get(telem.system_state, "Unknown")

            # update magnet data from latest values
            # magnet_current_values is 20x100, use most recent value from each row
            current_snapshot = [row[-1] if row else 0.0 for row in telem.magnet_current_values]
            self.magnet_currents = current_snapshot

            # Update setpoints directly from telemetry
            self.magnet_setpoints = list(telem.magnet_setpoints)

            # Update lists
            for i in range(20):
                cur = self.magnet_currents[i]
                setp = self.magnet_setpoints[i]
                self.magnet_list.set(f"mag{i}", "current", f"{cur:.2f}")
                self.magnet_list.set(f"mag{i}", "setpoint", f"{setp:.2f}")

            # Orientation as yaw/pitch/roll approximate from quaternion
            w, x, y, z = telem.orientation_wxyz
            # yaw (z-axis rotation)
            yaw = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))))
            roll = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))

            ax, ay, az = telem.angular_velocity_xyz

            # Telemetry basic summaries
            avg_current = sum(self.magnet_currents) / len(self.magnet_currents)
            text_content = (
                f"State: {self.system_state}\n"
                f"Timestamp: {telem.timestamp} ms\n"
                f"Magnet avg current: {avg_current:.2f}\n"
                f"Orientation (wxyz): {w:.3f}, {x:.3f}, {y:.3f}, {z:.3f}\n"
                f"Euler (deg): roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}\n"
                f"Angular vel (rad/s): x={ax:.2f}, y={ay:.2f}, z={az:.2f}\n"
                f"Last joystick: X={self.joystick_x:.2f}, Y={self.joystick_y:.2f}\n"
                f"Calib offset: {self.calibration_offset_deg:.1f} deg\n"
            )

            # Only update the text widget if content actually changed,
            # to avoid resetting the scroll position on every tick.
            if text_content != self._prev_telemetry_text:
                self._prev_telemetry_text = text_content
                self.telemetry_text.config(state=tk.NORMAL)
                self.telemetry_text.delete("1.0", tk.END)
                self.telemetry_text.insert("1.0", text_content)
                self.telemetry_text.config(state=tk.DISABLED)

        self.state_label.config(text=f"State: {self.system_state}",
                                fg="#00ffff" if connected else "#888888")

        # Schedule next update
        self.root.after(100, self.update_telemetry)
    
    def on_closing(self):
        """Handle window closing"""
        self.dashboard.stop()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    gui = DashboardGUI(root)
    root.protocol("WM_DELETE_WINDOW", gui.on_closing)
    root.mainloop()
