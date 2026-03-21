import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import ctypes
import threading
import math
from telemetry_dashboard import BallControllerDashboard
from ota_upload import upload_firmware


def configure_windows_dpi_awareness():
    """Request per-monitor DPI awareness on Windows for sharper text rendering."""
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(2)
    except Exception:
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except Exception:
            pass

class DashboardGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Ball Controller Telemetry Dashboard")
        self.root.geometry("1180x860")
        self.root.configure(bg="#10131a")
        self.root.minsize(1100, 800)

        self.colors = {
            "bg": "#10131a",
            "panel": "#171c26",
            "panel_alt": "#1d2431",
            "panel_edge": "#2b3445",
            "text": "#eef2ff",
            "muted": "#94a3b8",
            "accent": "#5eead4",
            "accent_2": "#60a5fa",
            "success": "#22c55e",
            "warning": "#f59e0b",
            "danger": "#ef4444",
            "purple": "#a78bfa",
        }
        
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

        # Telemetry-driven model state
        self.orientation_wxyz = [1.0, 0.0, 0.0, 0.0]
        self.angular_velocity_xyz = [0.0, 0.0, 0.0]
        self.ball_pos_map = [0.0, 0.0]
        self.ball_vel_map = [0.0, 0.0]
        self.map_box_limit = 8.0
        self.physics_dt = 0.1
        self.show_magnets_3d = tk.BooleanVar(value=False)
        self.show_magnet_ids_3d = tk.BooleanVar(value=False)

        # Mirrors esp-controller-idf/src/core/magnet_config.h positions.
        self.model_magnet_positions = [
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (-1.0, 0.0, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 0.0, -1.0),
            (1.0, 1.0, 0.0),
            (-1.0, -1.0, 0.0),
            (1.0, 0.0, 1.0),
            (-1.0, 0.0, -1.0),
            (0.0, 1.0, 1.0),
            (0.0, -1.0, -1.0),
            (1.0, -1.0, 0.0),
            (-1.0, 1.0, 0.0),
            (1.0, 0.0, -1.0),
            (-1.0, 0.0, 1.0),
            (0.5, 0.5, 0.5),
            (-0.5, -0.5, -0.5),
            (0.5, -0.5, 0.5),
            (-0.5, 0.5, -0.5),
        ]

        # Magnet data storage
        self.magnet_currents = [0.0] * 20
        self.magnet_setpoints = [0.0] * 20
        self.magnet_history = [[] for _ in range(20)]
        self.magnet_setpoint_history = [[] for _ in range(20)]
        self.magnet_display_mode = "instant"
        self.magnet_grid_padding = 8
        self.magnet_grid_cols = 5
        self.magnet_grid_rows = 4
        self.magnet_scale_max_amps = 15.0
        
        self.setup_ui()
        self.update_action_button_visibility()
        self.update_telemetry()

    def configure_styles(self):
        """Configure ttk styling for a more cohesive look."""
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure("Dashboard.Treeview",
                        background=self.colors["panel"],
                        fieldbackground=self.colors["panel"],
                        foreground=self.colors["text"],
                        rowheight=26,
                        borderwidth=0,
                        relief="flat")
        style.configure("Dashboard.Treeview.Heading",
                        background=self.colors["panel_alt"],
                        foreground=self.colors["text"],
                        relief="flat",
                        font=("Segoe UI", 10, "bold"))
        style.map("Dashboard.Treeview",
                  background=[("selected", self.colors["accent_2"])],
                  foreground=[("selected", "#ffffff")])
        style.configure("Dashboard.Vertical.TScrollbar",
                        troughcolor=self.colors["panel"],
                        background=self.colors["panel_alt"],
                        arrowcolor=self.colors["text"],
                        bordercolor=self.colors["panel_edge"],
                        lightcolor=self.colors["panel_edge"],
                        darkcolor=self.colors["panel_edge"])
        
    def setup_ui(self):
        """Create the GUI elements"""
        self.configure_styles()
        
        # Header
        header = tk.Frame(self.root, bg=self.colors["bg"])
        header.pack(fill=tk.X, padx=18, pady=(16, 8))

        title_label = tk.Label(header, text="Ball Controller Dashboard",
                               font=("Segoe UI", 22, "bold"), bg=self.colors["bg"], fg=self.colors["text"])
        title_label.pack(anchor="w")

        subtitle_row = tk.Frame(header, bg=self.colors["bg"])
        subtitle_row.pack(fill=tk.X, pady=(4, 0))

        self.connection_label = tk.Label(subtitle_row, text="● Disconnected",
                                         font=("Segoe UI", 10, "bold"),
                                         bg=self.colors["bg"], fg=self.colors["danger"])
        self.connection_label.pack(side=tk.RIGHT)
        
        # Main container with draggable divider (left/right resizable)
        main_pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL,
                       bg=self.colors["bg"], sashwidth=8,
                       sashrelief=tk.FLAT, bd=0)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=18, pady=10)

        # Left side - Joystick
        left_frame = tk.Frame(main_pane, bg=self.colors["bg"])

        # Overview card
        overview_card = tk.Frame(left_frame, bg=self.colors["panel"], highlightbackground=self.colors["panel_edge"], highlightthickness=1)
        overview_card.pack(fill=tk.X, pady=(0, 12))

        overview_inner = tk.Frame(overview_card, bg=self.colors["panel"])
        overview_inner.pack(fill=tk.X, padx=14, pady=12)

        self.state_label = tk.Label(overview_inner, text="Standby",
                                    font=("Segoe UI", 18, "bold"), bg=self.colors["panel"], fg=self.colors["accent"])
        self.state_label.pack(anchor="w")

        self.status_label = tk.Label(overview_inner, text="Waiting for ESP32 telemetry...",
                                     font=("Segoe UI", 10), bg=self.colors["panel"], fg=self.colors["muted"], justify=tk.LEFT)
        self.status_label.pack(anchor="w", pady=(4, 0))

        self.mode_hint_label = tk.Label(overview_inner, text="",
                                        font=("Segoe UI", 10, "italic"), bg=self.colors["panel"], fg=self.colors["accent_2"])
        self.mode_hint_label.pack(anchor="w", pady=(6, 0))
        
        # Joystick label
        joystick_label = tk.Label(left_frame, text="Joystick Input", 
                                  font=("Segoe UI", 14, "bold"), bg=self.colors["bg"], fg=self.colors["text"])
        joystick_label.pack(pady=10)
        
        # Joystick canvas (click/drag to control)
        self.joystick_canvas = tk.Canvas(left_frame, width=300, height=300, 
                                         bg="#0b1220", highlightthickness=1, highlightbackground=self.colors["panel_edge"])
        self.joystick_canvas.pack(pady=10)
        self.joystick_canvas.bind("<Button-1>", self.on_joystick_click)
        self.joystick_canvas.bind("<B1-Motion>", self.on_joystick_drag)
        self.joystick_canvas.bind("<ButtonRelease-1>", self.on_joystick_release)
        
        # Draw joystick circles
        self.draw_joystick()
        
        # Joystick value display
        self.joystick_value_label = tk.Label(left_frame, 
                                             text="X: 0.00  Y: 0.00", 
                             font=("Segoe UI", 12), bg=self.colors["bg"], fg=self.colors["accent"])
        self.joystick_value_label.pack(pady=5)
        
        # Ball model canvas (left: top-down map, right: 3D orientation)
        self.ball_canvas = tk.Canvas(left_frame, width=560, height=280,
                         bg="#0a0f18", highlightthickness=1, highlightbackground=self.colors["panel_edge"])
        self.ball_canvas.pack(pady=10)
        self.draw_ball_model()

        self.ball_info_label = tk.Label(left_frame, text="Position: (0.00, 0.00) Velocity: (0.00, 0.00)",
                        font=("Segoe UI", 11), bg=self.colors["bg"], fg=self.colors["text"])
        self.ball_info_label.pack(pady=5)

        model_options_row = tk.Frame(left_frame, bg=self.colors["bg"])
        model_options_row.pack(fill=tk.X, pady=(2, 4))

        self.show_magnets_toggle = tk.Checkbutton(
            model_options_row,
            text="Show magnets on 3D model",
            variable=self.show_magnets_3d,
            command=self.draw_ball_model,
            bg=self.colors["bg"],
            fg=self.colors["muted"],
            activebackground=self.colors["bg"],
            activeforeground=self.colors["text"],
            selectcolor=self.colors["panel"],
            font=("Segoe UI", 9),
        )
        self.show_magnets_toggle.pack(side=tk.LEFT)

        self.show_magnet_ids_toggle = tk.Checkbutton(
            model_options_row,
            text="Show IDs",
            variable=self.show_magnet_ids_3d,
            command=self.draw_ball_model,
            bg=self.colors["bg"],
            fg=self.colors["muted"],
            activebackground=self.colors["bg"],
            activeforeground=self.colors["text"],
            selectcolor=self.colors["panel"],
            font=("Segoe UI", 9),
        )
        self.show_magnet_ids_toggle.pack(side=tk.LEFT, padx=(12, 0))

        # Keyboard hint
        hint_label = tk.Label(left_frame, text="Click and drag to move joystick\nOr use WASD keys", 
                     font=("Segoe UI", 10), bg=self.colors["bg"], fg=self.colors["muted"], justify=tk.CENTER)
        hint_label.pack(pady=5)
        
        # Right side - Controls and status
        right_frame = tk.Frame(main_pane, bg=self.colors["bg"])

        main_pane.add(left_frame, minsize=480)
        main_pane.add(right_frame, minsize=420)
        
        # Top right block (status + controls)
        top_right_frame = tk.Frame(right_frame, bg=self.colors["bg"])
        top_right_frame.pack(fill=tk.X)

        # Status frame
        status_frame = tk.LabelFrame(top_right_frame, text="Status",
                         font=("Segoe UI", 12, "bold"), bg=self.colors["panel"], fg=self.colors["text"])
        status_frame.pack(fill=tk.X, pady=10)
        
        status_inner = tk.Frame(status_frame, bg=self.colors["panel"])
        status_inner.pack(fill=tk.X, padx=10, pady=8)

        self.connection_label.pack_forget()
        self.connection_label = tk.Label(status_inner, text="● Disconnected",
                         font=("Segoe UI", 11, "bold"), bg=self.colors["panel"], fg=self.colors["danger"], justify=tk.LEFT)
        self.connection_label.pack(anchor="w")

        self.status_label.pack_forget()
        self.status_label = tk.Label(status_inner, text="Waiting for ESP32 telemetry...",
                         font=("Segoe UI", 10), bg=self.colors["panel"], fg=self.colors["muted"], justify=tk.LEFT)
        self.status_label.pack(anchor="w", pady=(4, 0))

        self.state_label.pack_forget()
        self.state_label = tk.Label(status_inner, text="Standby",
                        font=("Segoe UI", 15, "bold"), bg=self.colors["panel"], fg=self.colors["accent"], justify=tk.LEFT)
        self.state_label.pack(anchor="w", pady=(8, 0))
        
        # Control buttons frame
        button_frame = tk.LabelFrame(top_right_frame, text="Controls",
                         font=("Segoe UI", 12, "bold"), bg=self.colors["panel"], fg=self.colors["text"])
        button_frame.pack(fill=tk.X, pady=10)

        button_inner = tk.Frame(button_frame, bg=self.colors["panel"])
        button_inner.pack(fill=tk.X, padx=10, pady=10)
        
        # Calibrate button (green)
        self.calibrate_btn = tk.Button(button_inner, text="CALIBRATE", 
                                       command=self.on_calibrate,
                           font=("Segoe UI", 12, "bold"), 
                           bg=self.colors["success"], fg="#081018", width=20, height=2,
                           activebackground="#4ade80", activeforeground="#081018",
                           relief="flat", highlightthickness=0)
        
        # Start button
        self.start_btn = tk.Button(button_inner, text="START", 
                                   command=self.on_start,
                       font=("Segoe UI", 12, "bold"), 
                       bg=self.colors["accent_2"], fg="white", width=20, height=2,
                       activebackground="#93c5fd", activeforeground="#081018",
                       relief="flat", highlightthickness=0)
        
        # Send direction button
        self.send_direction_btn = tk.Button(button_inner, text="SEND DIRECTION", 
                                           command=self.on_send_direction,
                           font=("Segoe UI", 12, "bold"), 
                           bg=self.colors["warning"], fg="#121212", width=20, height=2,
                           activebackground="#fbbf24", activeforeground="#121212",
                           relief="flat", highlightthickness=0)
        
        # Emergency stop button (red)
        self.stop_btn = tk.Button(button_inner, text="EMERGENCY STOP", 
                                       command=self.on_emergency_stop,
                           font=("Segoe UI", 14, "bold"), 
                           bg=self.colors["danger"], fg="white", width=20, height=2,
                           activebackground="#fb7185", activeforeground="white",
                           relief="flat", highlightthickness=0)
        
        # OTA flash button
        self.ota_btn = tk.Button(button_inner, text="FLASH FIRMWARE (OTA)",
                                 command=self.on_ota_upload,
                     font=("Segoe UI", 11, "bold"),
                     bg=self.colors["purple"], fg="white", width=20, height=2,
                     activebackground="#c4b5fd", activeforeground="#081018",
                     relief="flat", highlightthickness=0)

        # Resizable right-side lower panes (magnets / telemetry)
        right_data_pane = tk.PanedWindow(right_frame, orient=tk.VERTICAL,
                         bg=self.colors["bg"], sashwidth=8,
                         sashrelief=tk.FLAT, bd=0)
        right_data_pane.pack(fill=tk.BOTH, expand=True, pady=(2, 0))

        magnet_panel = tk.Frame(right_data_pane, bg=self.colors["bg"])

        # Magnet display mode controls
        magnet_mode_row = tk.Frame(magnet_panel, bg=self.colors["bg"])
        magnet_mode_row.pack(fill=tk.X, pady=(0, 4))

        magnet_mode_label = tk.Label(magnet_mode_row, text="Magnet View",
                                     font=("Segoe UI", 11, "bold"), bg=self.colors["bg"], fg=self.colors["muted"])
        magnet_mode_label.pack(side=tk.LEFT, padx=(2, 8))

        self.instant_mode_btn = tk.Button(magnet_mode_row, text="Instant",
                                          command=lambda: self.set_magnet_display_mode("instant"),
                                          font=("Segoe UI", 10, "bold"), bg=self.colors["accent_2"], fg="white",
                                          relief="flat", highlightthickness=0, padx=12, pady=4)
        self.instant_mode_btn.pack(side=tk.LEFT, padx=(0, 6))

        self.graph_mode_btn = tk.Button(magnet_mode_row, text="Graph (2s)",
                                        command=lambda: self.set_magnet_display_mode("graph"),
                                        font=("Segoe UI", 10, "bold"), bg=self.colors["panel_alt"], fg=self.colors["text"],
                                        relief="flat", highlightthickness=0, padx=12, pady=4)
        self.graph_mode_btn.pack(side=tk.LEFT)
        
        # Magnet visualization frame
        magnet_frame = tk.LabelFrame(magnet_panel, text="Magnets",
                                     font=("Segoe UI", 12, "bold"), bg=self.colors["panel"], fg=self.colors["text"])
        magnet_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        magnet_canvas_wrap = tk.Frame(magnet_frame, bg=self.colors["panel"])
        magnet_canvas_wrap.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.magnet_canvas = tk.Canvas(magnet_canvas_wrap, width=520, height=390,
                           bg="#0d1117", highlightthickness=0)
        self.magnet_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.magnet_scroll = ttk.Scrollbar(magnet_canvas_wrap, orient="vertical",
                           command=self.magnet_canvas.yview,
                           style="Dashboard.Vertical.TScrollbar")
        self.magnet_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.magnet_canvas.configure(yscrollcommand=self.magnet_scroll.set)
        self.magnet_canvas.bind("<Configure>", lambda _e: self.draw_magnet_grid())

        # Telemetry frame
        telemetry_frame = tk.LabelFrame(right_data_pane, text="Telemetry",
                        font=("Segoe UI", 12, "bold"), bg=self.colors["panel"], fg=self.colors["text"])

        # Telemetry text display
        self.telemetry_text = tk.Text(telemetry_frame, height=10, width=40,
                          font=("Consolas", 9), bg="#0d1117", fg=self.colors["accent"],
                          highlightthickness=0)
        self.telemetry_text.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
        self.telemetry_text.config(state=tk.DISABLED)  # Read-only

        right_data_pane.add(magnet_panel, minsize=220)
        right_data_pane.add(telemetry_frame, minsize=140)
        
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
        self.joystick_canvas.create_rectangle(10, 10, 290, 290, outline=self.colors["panel_edge"], width=2)
        
        # Draw crosshairs
        self.joystick_canvas.create_line(150, 10, 150, 290, fill="#334155", width=1)
        self.joystick_canvas.create_line(10, 150, 290, 150, fill="#334155", width=1)
        
        # Draw center circle
        self.joystick_canvas.create_oval(140, 140, 160, 160, outline="#64748b", width=1)
        
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
                                         fill=self.colors["accent"], width=3)
        
        # Draw stick ball
        ball_size = 15
        self.joystick_canvas.create_oval(stick_x - ball_size, stick_y - ball_size,
                                         stick_x + ball_size, stick_y + ball_size,
                                         fill=self.colors["accent"], outline="#14b8a6", width=2)

    def draw_ball_model(self):
        """Draw top-down position view and 3D orientation view."""
        self.ball_canvas.delete("all")

        w = max(1, int(self.ball_canvas.winfo_width()))
        h = max(1, int(self.ball_canvas.winfo_height()))
        mid = w // 2

        self._draw_top_down_panel(10, 10, mid - 10, h - 10)
        self._draw_orientation_panel(mid + 10, 10, w - 10, h - 10)

    def _draw_top_down_panel(self, x0, y0, x1, y1):
        """Left panel: global-frame top-down room map with estimated position and velocity."""
        c = self.ball_canvas
        c.create_rectangle(x0, y0, x1, y1, outline=self.colors["panel_edge"], width=1)
        c.create_text(x0 + 8, y0 + 8, anchor="nw", fill=self.colors["muted"],
                      font=("Segoe UI", 9), text="Top-down position (global frame)")

        pad = 22
        map_left = x0 + pad
        map_top = y0 + pad
        map_right = x1 - pad
        map_bottom = y1 - pad
        c.create_rectangle(map_left, map_top, map_right, map_bottom, outline="#334155", width=1)

        def world_to_canvas(px, py):
            sx = (px / self.map_box_limit)
            sy = (py / self.map_box_limit)
            cx = (map_left + map_right) * 0.5 + sx * (map_right - map_left) * 0.5
            cy = (map_top + map_bottom) * 0.5 - sy * (map_bottom - map_top) * 0.5
            return cx, cy

        # center cross
        cx, cy = world_to_canvas(0.0, 0.0)
        c.create_line(cx, map_top, cx, map_bottom, fill="#243041", dash=(2, 2))
        c.create_line(map_left, cy, map_right, cy, fill="#243041", dash=(2, 2))

        # Global +X / +Y reference in map corner
        ref_x0 = map_left + 16
        ref_y0 = map_bottom - 16
        c.create_line(ref_x0, ref_y0, ref_x0 + 24, ref_y0, fill="#e2e8f0", width=2, arrow=tk.LAST)
        c.create_line(ref_x0, ref_y0, ref_x0, ref_y0 - 24, fill="#e2e8f0", width=2, arrow=tk.LAST)
        c.create_text(ref_x0 + 28, ref_y0, text="+X", anchor="w", fill="#cbd5e1", font=("Segoe UI", 8, "bold"))
        c.create_text(ref_x0, ref_y0 - 28, text="+Y", anchor="s", fill="#cbd5e1", font=("Segoe UI", 8, "bold"))

        bx, by = world_to_canvas(self.ball_pos_map[0], self.ball_pos_map[1])
        c.create_oval(bx - 8, by - 8, bx + 8, by + 8, fill=self.colors["danger"], outline="")

        # Velocity vector (estimated from gyro)
        vel_scale = 12.0
        vx = self.ball_vel_map[0]
        vy = self.ball_vel_map[1]
        c.create_line(bx, by, bx + vx * vel_scale, by - vy * vel_scale,
                      fill=self.colors["accent_2"], width=2, arrow=tk.LAST)

        # Desired direction vector from joystick
        c.create_line(bx, by, bx + self.joystick_x * 30, by - self.joystick_y * 30,
                      fill=self.colors["warning"], width=2, arrow=tk.LAST)

        # Ball body +X projection in global top-down (from IMU quaternion)
        rot = self._quat_to_rot(self.orientation_wxyz)
        body_x_world = self._mat_vec(rot, [1.0, 0.0, 0.0])
        c.create_line(bx, by, bx + body_x_world[0] * 22, by - body_x_world[1] * 22,
                  fill="#f472b6", width=2, arrow=tk.LAST)

    def _quat_to_rot(self, q):
        """Quaternion (w,x,y,z) to 3x3 rotation matrix."""
        w, x, y, z = q
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ]

    def _mat_vec(self, m, v):
        return [
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
        ]

    def _normalize3(self, v):
        n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        if n < 1e-6:
            return [0.0, 0.0, 0.0]
        return [v[0] / n, v[1] / n, v[2] / n]

    def _draw_orientation_panel(self, x0, y0, x1, y1):
        """Right panel: 3D sphere with axes rotated by IMU quaternion."""
        c = self.ball_canvas
        c.create_rectangle(x0, y0, x1, y1, outline=self.colors["panel_edge"], width=1)
        c.create_text(x0 + 8, y0 + 8, anchor="nw", fill=self.colors["muted"],
                      font=("Segoe UI", 9), text="3D orientation (IMU, oblique view)")

        cx = (x0 + x1) * 0.5
        cy = (y0 + y1) * 0.55
        radius = min((x1 - x0), (y1 - y0)) * 0.33

        # Sphere outline
        c.create_oval(cx - radius, cy - radius, cx + radius, cy + radius,
                      outline="#334155", width=2)
        c.create_oval(cx - radius, cy - radius * 0.35, cx + radius, cy + radius * 0.35,
                      outline="#223047", width=1)

        rot = self._quat_to_rot(self.orientation_wxyz)

        def proj(v):
            # Simple perspective-like projection for readability
            px = cx + (v[0] + 0.35 * v[2]) * radius
            py = cy - (v[1] - 0.15 * v[2]) * radius
            return px, py

        axes = [
            ([1.0, 0.0, 0.0], "#ef4444"),
            ([0.0, 1.0, 0.0], "#22c55e"),
            ([0.0, 0.0, 1.0], "#60a5fa"),
        ]

        for axis_vec, color in axes:
            rv = self._mat_vec(rot, axis_vec)
            ex, ey = proj(rv)
            c.create_line(cx, cy, ex, ey, fill=color, width=3, arrow=tk.LAST)

        # A marker on the rotated +Z direction to show spin orientation clearly
        pole = self._mat_vec(rot, [0.0, 0.0, 1.0])
        px, py = proj(pole)
        c.create_oval(px - 5, py - 5, px + 5, py + 5, fill="#93c5fd", outline="")

        if self.show_magnets_3d.get():
            for idx, original in enumerate(self.model_magnet_positions):
                body_vec = self._normalize3(original)
                world_vec = self._mat_vec(rot, body_vec)
                mx, my = proj(world_vec)

                in_front = world_vec[2] >= 0.0
                marker_color = "#f97316" if in_front else "#475569"
                marker_r = 4 if in_front else 3
                c.create_oval(mx - marker_r, my - marker_r, mx + marker_r, my + marker_r,
                              fill=marker_color, outline="")

                if self.show_magnet_ids_3d.get() and in_front:
                    c.create_text(mx + 6, my - 6, text=str(idx + 1),
                                  anchor="sw", fill="#fde68a", font=("Segoe UI", 8, "bold"))

    def set_magnet_display_mode(self, mode):
        """Switch between instant and graph magnet display modes."""
        if mode not in ("instant", "graph"):
            return
        self.magnet_display_mode = mode
        self.instant_mode_btn.config(bg=self.colors["accent_2"] if mode == "instant" else self.colors["panel_alt"])
        self.instant_mode_btn.config(fg="white" if mode == "instant" else self.colors["text"])
        self.graph_mode_btn.config(bg=self.colors["accent_2"] if mode == "graph" else self.colors["panel_alt"])
        self.graph_mode_btn.config(fg="white" if mode == "graph" else self.colors["text"])
        self.draw_magnet_grid()

    def _clamp_amps(self, value):
        return max(0.0, min(self.magnet_scale_max_amps, float(value)))

    def draw_magnet_grid(self):
        """Draw the 5x2 magnet grid using the current display mode."""
        canvas = self.magnet_canvas
        canvas.delete("all")

        width = max(1, int(canvas.winfo_width()))
        height = max(1, int(canvas.winfo_height()))
        cols = self.magnet_grid_cols
        rows = self.magnet_grid_rows
        pad = self.magnet_grid_padding

        header_h = 26
        usable_w = max(80, width - pad * (cols + 1))
        cell_w = usable_w / cols

        fit_cell_h = (height - header_h - pad * (rows + 1)) / rows
        min_cell_h = 88
        cell_h = max(min_cell_h, fit_cell_h)

        content_h = header_h + pad + rows * (cell_h + pad)
        canvas.configure(scrollregion=(0, 0, width, content_h))

        canvas.create_text(12, 10, anchor="nw",
                           fill=self.colors["muted"], font=("Segoe UI", 9),
                           text=f"{self.magnet_display_mode.capitalize()} mode  |  Scale: 0A to {self.magnet_scale_max_amps:.0f}A")

        for index in range(20):
            row = index // cols
            col = index % cols
            x0 = pad + col * (cell_w + pad)
            y0 = header_h + pad + row * (cell_h + pad)
            x1 = x0 + cell_w
            y1 = y0 + cell_h
            self._draw_magnet_cell(canvas, index, x0, y0, x1, y1)

    def _draw_magnet_cell(self, canvas, index, x0, y0, x1, y1):
        current = self._clamp_amps(self.magnet_currents[index])
        setpoint = self._clamp_amps(self.magnet_setpoints[index])
        height = y1 - y0
        width = x1 - x0

        # Base frame and labels
        canvas.create_rectangle(x0, y0, x1, y1, outline=self.colors["panel_edge"], width=1, fill="#0d1117")
        canvas.create_text(x0 + 6, y0 + 5, anchor="nw", text=f"M{index + 1}",
                           fill=self.colors["text"], font=("Segoe UI", 9, "bold"))
        canvas.create_text(x1 - 6, y0 + 5, anchor="ne", text=f"{current:.1f}A",
                           fill=self.colors["accent_2"], font=("Segoe UI", 8, "bold"))

        graph_top = y0 + 22
        graph_bottom = y1 - 8
        graph_left = x0 + 8
        graph_right = x1 - 8

        # 0A to 15A vertical scale markers
        canvas.create_line(graph_left, graph_bottom, graph_left, graph_top, fill="#334155", width=1)
        for tick_amps in (0, 5, 10, 15):
            tick_y = graph_bottom - ((tick_amps / self.magnet_scale_max_amps) * (graph_bottom - graph_top))
            canvas.create_line(graph_left - 3, tick_y, graph_left, tick_y, fill="#475569", width=1)

        if self.magnet_display_mode == "instant":
            fill_height = (current / self.magnet_scale_max_amps) * (graph_bottom - graph_top)
            fill_top = graph_bottom - fill_height
            canvas.create_rectangle(graph_left, fill_top, graph_right, graph_bottom,
                                    outline="", fill=self.colors["accent_2"])
            set_y = graph_bottom - ((setpoint / self.magnet_scale_max_amps) * (graph_bottom - graph_top))
            canvas.create_line(graph_left, set_y, graph_right, set_y, fill=self.colors["warning"], width=3)
            canvas.create_text(x0 + width / 2, y1 - 4, text=f"S {setpoint:.1f}A",
                               fill=self.colors["warning"], font=("Segoe UI", 8, "bold"), anchor="s")
        else:
            latest_timestamp = getattr(self, "latest_telemetry_timestamp", None)
            samples = []
            setpoint_samples = []
            if latest_timestamp is not None:
                cutoff = latest_timestamp - 2000
                samples = [value for sample_time, value in self.magnet_history[index] if sample_time >= cutoff]
                setpoint_samples = [
                    value for sample_time, value in self.magnet_setpoint_history[index] if sample_time >= cutoff
                ]
            if not samples:
                samples = [current]
            if not setpoint_samples:
                setpoint_samples = [setpoint]

            sample_count = len(samples)
            step = (graph_right - graph_left) / max(1, sample_count - 1)
            points = []
            for sample_index, sample in enumerate(samples):
                sample_value = self._clamp_amps(sample)
                px = graph_left + sample_index * step
                py = graph_bottom - ((sample_value / self.magnet_scale_max_amps) * (graph_bottom - graph_top))
                points.append((px, py))

            fill_points = [(graph_left, graph_bottom)] + points + [(graph_right, graph_bottom)]
            flat_points = [coordinate for point in fill_points for coordinate in point]
            canvas.create_polygon(*flat_points, fill=self.colors["accent_2"], outline="")
            if len(points) > 1:
                line_points = [coordinate for point in points for coordinate in point]
                canvas.create_line(*line_points, fill="#93c5fd", width=2, smooth=True)

            set_count = len(setpoint_samples)
            set_step = (graph_right - graph_left) / max(1, set_count - 1)
            set_points = []
            for sample_index, sample in enumerate(setpoint_samples):
                sample_value = self._clamp_amps(sample)
                px = graph_left + sample_index * set_step
                py = graph_bottom - ((sample_value / self.magnet_scale_max_amps) * (graph_bottom - graph_top))
                set_points.append((px, py))
            if len(set_points) > 1:
                line_points = [coordinate for point in set_points for coordinate in point]
                canvas.create_line(*line_points, fill=self.colors["warning"], width=2, smooth=True)
            else:
                set_y = graph_bottom - ((setpoint / self.magnet_scale_max_amps) * (graph_bottom - graph_top))
                canvas.create_line(graph_left, set_y, graph_right, set_y, fill=self.colors["warning"], width=2)

            canvas.create_text(x0 + width / 2, y1 - 4, text=f"S {setpoint:.1f}A",
                               fill=self.colors["warning"], font=("Segoe UI", 8, "bold"), anchor="s")

    def _refresh_magnet_history(self, telemetry):
        """Store the latest current history for graph mode."""
        latest_timestamp = telemetry.timestamp
        for magnet_index in range(20):
            row = telemetry.magnet_current_values[magnet_index]
            if row:
                history = self.magnet_history[magnet_index]
                for sample in row:
                    history.append((latest_timestamp, self._clamp_amps(sample)))
                cutoff = latest_timestamp - 4000
                while history and history[0][0] < cutoff:
                    history.pop(0)

            setpoint_history = self.magnet_setpoint_history[magnet_index]
            setpoint_history.append((latest_timestamp, self._clamp_amps(telemetry.magnet_setpoints[magnet_index])))
            cutoff = latest_timestamp - 4000
            while setpoint_history and setpoint_history[0][0] < cutoff:
                setpoint_history.pop(0)
    
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

    def update_action_button_visibility(self):
        """Show only actions that are valid for the current telemetry state."""
        connected = self.dashboard.is_connected

        can_calibrate = connected and self.system_state == "Standby"
        can_start = connected and self.system_state == "Calibration"
        can_send_direction = connected and self.system_state == "Calibration"
        can_flash = connected and self.system_state == "Standby"
        can_emergency = connected

        button_specs = [
            (self.calibrate_btn, can_calibrate, {"padx": 10, "pady": 5}),
            (self.start_btn, can_start, {"padx": 10, "pady": 5}),
            (self.send_direction_btn, can_send_direction, {"padx": 10, "pady": 5}),
            (self.emergency_btn, can_emergency, {"padx": 10, "pady": 10}),
            (self.ota_btn, can_flash, {"padx": 10, "pady": 5}),
        ]

        for button, _, _ in button_specs:
            if button.winfo_ismapped():
                button.pack_forget()

        for button, visible, pack_kwargs in button_specs:
            if visible:
                button.pack(**pack_kwargs)

        if connected:
            if self.system_state == "Standby":
                self.mode_hint_label.config(text="Ready for calibration or firmware update.", fg=self.colors["warning"])
            elif self.system_state == "Calibration":
                self.mode_hint_label.config(text="Calibration active: start and send direction are available.", fg=self.colors["accent_2"])
            elif self.system_state == "Running":
                self.mode_hint_label.config(text="Running: use emergency stop if needed.", fg=self.colors["success"])
            else:
                self.mode_hint_label.config(text="Connected, waiting for valid state.", fg=self.colors["muted"])
        else:
            self.mode_hint_label.config(text="Connect to the ESP32 WiFi network to begin.", fg=self.colors["muted"])

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

        # Keep legacy local state in sync with telemetry-driven map model for labels.
        self.ball_position[0] = self.ball_pos_map[0] / self.map_box_limit
        self.ball_position[1] = self.ball_pos_map[1] / self.map_box_limit
        self.ball_velocity[0] = self.ball_vel_map[0]
        self.ball_velocity[1] = self.ball_vel_map[1]

        self.draw_ball_model()
        self.ball_info_label.config(
            text=f"Pos: ({self.ball_pos_map[0]:.2f}, {self.ball_pos_map[1]:.2f})  "
                 f"Vel: ({self.ball_vel_map[0]:.2f}, {self.ball_vel_map[1]:.2f})"
        )

        self.draw_magnet_grid()

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
            self.latest_telemetry_timestamp = telem.timestamp
            self._refresh_magnet_history(telem)

            # Orientation as yaw/pitch/roll approximate from quaternion
            w, x, y, z = telem.orientation_wxyz
            self.orientation_wxyz = [w, x, y, z]
            # yaw (z-axis rotation)
            yaw = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))))
            roll = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))

            ax, ay, az = telem.angular_velocity_xyz
            self.angular_velocity_xyz = [ax, ay, az]

            # Estimate top-down linear velocity from body-frame angular velocity and
            # convert it to world frame for a true global map view.
            vel_body = [ay, -ax, 0.0]
            rot = self._quat_to_rot(self.orientation_wxyz)
            vel_world = self._mat_vec(rot, vel_body)
            self.ball_vel_map[0] = 0.85 * self.ball_vel_map[0] + 0.15 * vel_world[0]
            self.ball_vel_map[1] = 0.85 * self.ball_vel_map[1] + 0.15 * vel_world[1]

            self.ball_pos_map[0] += self.ball_vel_map[0] * self.physics_dt
            self.ball_pos_map[1] += self.ball_vel_map[1] * self.physics_dt

            # Clamp map position to virtual room bounds.
            self.ball_pos_map[0] = max(-self.map_box_limit, min(self.map_box_limit, self.ball_pos_map[0]))
            self.ball_pos_map[1] = max(-self.map_box_limit, min(self.map_box_limit, self.ball_pos_map[1]))

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

            self.draw_magnet_grid()

        self.state_label.config(text=f"State: {self.system_state}",
                                fg="#00ffff" if connected else "#888888")
        self.update_action_button_visibility()

        # Schedule next update
        self.root.after(100, self.update_telemetry)
    
    def on_closing(self):
        """Handle window closing"""
        self.dashboard.stop()
        self.root.destroy()

if __name__ == "__main__":
    configure_windows_dpi_awareness()
    root = tk.Tk()
    try:
        scaling = root.winfo_fpixels("1i") / 72.0
        root.tk.call("tk", "scaling", scaling)
    except Exception:
        pass
    gui = DashboardGUI(root)
    root.protocol("WM_DELETE_WINDOW", gui.on_closing)
    root.mainloop()
