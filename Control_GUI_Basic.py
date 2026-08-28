import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import logging
import math
from collections import deque

# matplotlib embedding
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import Trajectory_controller as controller

# Map constants from controller module (kept for readability)
IDLE = controller.IDLE
CLOSE_LOOP_CONTROL = controller.CLOSED_LOOP_CONTROL

logging.basicConfig(level=logging.INFO) #GAY

logger = logging.getLogger("ControlGUI")

# Combined update interval (GUI + plot)
UPDATE_INTERVAL_MS = 50

class ControlGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Control GUI")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # Display-only dummy controller (or real, depending on underlying module)
        self.controller = controller.ODriveThread()
        self.controller.start()

        self.control_elms = {
            "Target": (self.controller.start_pos, "deg"), 
            "Max velocity": (60, "deg/s"), 
            # "Max jerk": (),
            "Kp": (self.controller.Kp, None), 
            "Kd": (self.controller.Kd, None), 
            "Control bandwidth": (self.controller.ctrl_bandwidth, None), 
            "Encoder bandwidth": (self.controller.enc_bandwidth, None)
        }

        self.load_params = {
            "External load": (self.controller.ext_load, "kg"), 
            "Load position": (self.controller.hanger_distance, "m"), 
            "Coulomb friction": (self.controller.coul_friction, "Nm"), 
            "Viscous friction": (self.controller.visc_friction, "Nm/rad"),
            "Torque limit": (self.controller.max_torque, "Nm")
        }

        self.max_jerk = 0

        # UI state
        self.plotting = True

        # --- THÊM MỚI: Biến lưu khớp đang được chọn để hiển thị ---
        # 0 = HIP (Hông), 1 = KNEE (Gối), 2 = ANKLE (Cổ chân)
        self.selected_joint = tk.IntVar(value=0)

        # --- THÊM MỚI: Placeholder cho UART Manager (được inject từ main.py) ---
        # Nếu main.py inject uart vào, đồ thị sẽ đọc dữ liệu từ ESP32 telemetry.
        # Nếu không có (chạy standalone), hoạt động y hệt như cũ (đọc từ ODrive trực tiếp).
        self.uart = None

        # Build UI
        self._build_ui()
        # Start periodic combined updates
        self.after(UPDATE_INTERVAL_MS, self._update)

    def _build_ui(self):
        # Main frame layout: left plots, right controls
        main = ttk.Frame(self, padding=6)
        main.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right = ttk.Frame(main, width=360)
        right.pack(side=tk.RIGHT, fill=tk.Y)

        # Plots: 3 stacked subplots (Position, Velocity, Torque)
        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax_pos = self.fig.add_subplot(411)
        self.ax_vel = self.fig.add_subplot(412)
        self.ax_acc = self.fig.add_subplot(413)
        self.ax_tor = self.fig.add_subplot(414)
        self.fig.tight_layout(pad=2.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Action buttons panel=============================================================================================================================================
        # 1. Status Label      
        top_right = ttk.Frame(right, padding=6)
        top_right.pack(side=tk.TOP, fill=tk.X)

        for c in range(3): top_right.columnconfigure(c, weight=1)
        for r in range(4): top_right.rowconfigure(r, weight=1)
        
        # 1. Status Label
        self.status_label = tk.Label(top_right, text="Ready", relief="ridge", bg="lightgreen")
        self.status_label.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        # NÚT MỚI: Calib button (Nằm ở giữa)
        self.btn_calib = tk.Button(top_right, text="Calib", bg="lightblue", relief="raised", command=self._on_calib)
        self.btn_calib.grid(row=0, column=1, sticky="nsew", padx=2, pady=2)

        # 2. Offset button
        self.btn_offset = tk.Button(top_right, text="Offset", bg="tomato", relief="raised", command=self._on_offset)
        self.btn_offset.grid(row=0, column=2, sticky="nsew", padx=2, pady=2)
        # 3. IDLE/Close loop toggle (mode)
        self.btn_mode = tk.Button(top_right, text="Close Loop", bg="lightgreen", relief="raised", command=self._on_mode_tog)
        self.btn_mode.grid(row=1, column=0, sticky="nsew", padx=2, pady=2)
        # 4. Stop/Continue plotting
        self.btn_plot = tk.Button(top_right, text="Stop plotting", relief="raised", command=self._on_toggle_plot)
        self.btn_plot.grid(row=1, column=1, sticky="nsew", padx=2, pady=2)
        # 5. Reset
        self.btn_reset = tk.Button(top_right, text="Reset", relief="raised", command=self._on_reset)
        self.btn_reset.grid(row=2, column=0, sticky="nsew", padx=2, pady=2)
        # 6. EStop
        self.btn_estop = tk.Button(top_right, text="ESTOP", bg="red", fg="white", relief="raised", command=self._on_estop)
        self.btn_estop.grid(row=2, column=1, sticky="nsew", padx=2, pady=2)

        # ==============================================================================================
        # THÊM MỚI: THANH CHỌN KHỚP (Radio Buttons) - Row 3
        # ==============================================================================================
        joint_frame = ttk.LabelFrame(top_right, text="Khop hien thi")
        joint_frame.grid(row=3, column=0, columnspan=3, sticky="nsew", padx=2, pady=4)

        JOINT_LABELS = [
            ("Hip (Hong)",     0, "#FF6B6B"),   # Đỏ
            ("Knee (Goi)",     1, "#4ECDC4"),   # Xanh ngọc
            ("Ankle (Co chan)",2, "#45B7D1"),   # Xanh dương
        ]
        for col_idx, (label, val, color) in enumerate(JOINT_LABELS):
            rb = tk.Radiobutton(
                joint_frame,
                text=label,
                variable=self.selected_joint,
                value=val,
                command=self._on_joint_select,  # Gọi callback khi bấm chọn
                bg=color,
                fg="white",
                selectcolor="#333333",
                activebackground=color,
                font=("Helvetica", 8, "bold"),
                relief="raised",
                indicatoron=0,   # Kiểu nút bấm đặc (không phải tròn)
                padx=4, pady=3
            )
            rb.grid(row=0, column=col_idx, sticky="nsew", padx=2, pady=2)
            joint_frame.columnconfigure(col_idx, weight=1)

        # Move controls==================================================================================================================================================
        control_frame = ttk.LabelFrame(right, text="Control", padding=8)
        control_frame.pack(padx=6, pady=8, fill=tk.X)

        control_grid = ttk.Frame(control_frame)
        control_grid.pack(fill=tk.X)

        ttk.Label(control_grid, text="Position (deg):").grid(row=0, column=0, sticky=tk.W)
        self.entry_pos = ttk.Entry(control_grid, width=12, state="readonly")
        self.entry_pos.grid(row=0, column=1, padx=4, pady=2)

        self.control_panel = []
        
        for i, (key, (val, unit)) in enumerate(self.control_elms.items()):
            row_idx = i + 1
            label_text = f"{key} ({unit}):" if unit else f"{key}:"
            ttk.Label(control_grid, text=label_text).grid(row=row_idx, column=0, sticky=tk.W, pady=1)
            v = tk.StringVar(value=f"{val:.2f}")
            entry = ttk.Entry(control_grid, textvariable=v, width=12) 
            entry.grid(row=row_idx, column=1, padx=4, pady=2) 
            self.control_panel.append([entry, v])

        # Move button
        self.btn_move = ttk.Button(control_frame, text="Move", command=self._on_move, state="disable")
        self.btn_move.pack(pady=(10, 0), fill=tk.X)

        # Filter settings==================================================================================================================================================
        filter_frame = ttk.LabelFrame(right, text="Filter", padding=8)
        filter_frame.pack(padx=6, pady=8, fill=tk.X)

        filter_grid = ttk.Frame(filter_frame)
        filter_grid.pack(fill=tk.X)

        ttk.Label(filter_grid, text="Window size:").grid(row=0, column=0, sticky=tk.W)
        self.window_size_var = tk.StringVar(value=str(self.controller.window_size))
        self.entry_window_size = ttk.Entry(filter_grid, textvariable=self.window_size_var, width=12)
        self.entry_window_size.grid(row=0, column=1, padx=4, pady=2)

        ttk.Label(filter_grid, text="Poly order:").grid(row=1, column=0, sticky=tk.W)
        self.poly_order_var = tk.StringVar(value=str(self.controller.poly_order))
        self.entry_poly_order = ttk.Entry(filter_grid, textvariable=self.poly_order_var, width=12)
        self.entry_poly_order.grid(row=1, column=1, padx=4, pady=2)

        # Apply button
        self.btn_apply_filter = ttk.Button(filter_frame, text="Apply", command=self._on_apply_filter, state="disable")
        self.btn_apply_filter.pack(pady=(10, 0), fill=tk.X)

        # Error display===================================================================================================================================================
        error_frame = ttk.LabelFrame(right, text="Error", padding=8)
        error_frame.pack(padx=6, pady=8, fill=tk.X)

        ttk.Label(error_frame, text="Position error (deg):").grid(row=0, column=0, sticky=tk.W)
        self.entry_pos_error = ttk.Entry(error_frame, width=12, state="readonly")
        self.entry_pos_error.grid(row=0, column=1, padx=4, pady=2)

        ttk.Label(error_frame, text="Velocity error (deg/s):").grid(row=1, column=0, sticky=tk.W)
        self.entry_vel_error = ttk.Entry(error_frame, width=12, state="readonly")
        self.entry_vel_error.grid(row=1, column=1, padx=4, pady=2)

        ttk.Label(error_frame, text="Max jerk (deg/s^3):").grid(row=2, column=0, sticky=tk.W)
        self.max_jerk_var = tk.StringVar(value="0")
        self.entry_max_jerk = ttk.Entry(error_frame, textvariable=self.max_jerk_var, width=12, state="readonly")
        self.entry_max_jerk.grid(row=2, column=1, padx=4, pady=2)

        # Send parameters block=========================================================================================================================================
        param_frame = ttk.LabelFrame(right, text="Parameters", padding=8)
        param_frame.pack(padx=6, pady=8, fill=tk.X)

        param_grid = ttk.Frame(param_frame)
        param_grid.pack(fill=tk.X)

        self.param_panel = []
        for i, (key, (val, unit)) in enumerate(self.load_params.items()):
            row_idx = i + 1
            ttk.Label(param_grid, text=f"{key} ({unit}):" if unit else f"{key}:").grid(row=row_idx, column=0, sticky=tk.W, pady=1)
            
            v = tk.StringVar(value=f"{val:.3f}")
            entry = ttk.Entry(param_grid, textvariable=v, width=12)
            entry.grid(row=row_idx, column=1, padx=4, pady=2) 
            self.param_panel.append([entry, v])

        self.btn_send_param = ttk.Button(param_frame, text="Send", command=self._on_send_parameters, state="disable")
        self.btn_send_param.pack(pady=(10, 0), fill=tk.X)

        # Status area at bottom==========================================================================================================================================
        status_frame = ttk.Frame(right, padding=6)
        status_frame.pack(side=tk.BOTTOM, fill=tk.X)

        self.status_text = tk.StringVar(value="Status: starting...")
        ttk.Label(status_frame, textvariable=self.status_text, relief=tk.RIDGE).pack(fill=tk.X)

        # Initialize some states
        self.send_enable(enabled=False)

        # Plot data lines (empty initially)===========================================================================================================================
        self._pos_line, = self.ax_pos.plot([], [], label="q (deg)")
        self._pos_set_line, = self.ax_pos.plot([], [], label="q_d (deg)", linestyle="--")
        self._vel_line, = self.ax_vel.plot([], [], label="qdot (deg/s)")
        self._vel_set_line, = self.ax_vel.plot([], [], label="qdot_d (deg/s)", linestyle="--")
        self._acc_line, = self.ax_acc.plot([], [], label="qddot (deg/s^2)")
        self._acc_set_line, = self.ax_acc.plot([], [], label="qddot_d (deg/s^2)", linestyle="--")
        self._tor_line, = self.ax_tor.plot([], [], label="Torque (Nm)")
        for ax in (self.ax_pos, self.ax_vel, self.ax_acc, self.ax_tor):
            ax.grid(True)
            ax.legend(loc="upper right")

        # Keep a reference to last plotted time origin to keep plots stable
        self._last_t0 = None

    # ---------------------------------------------------------------------------
    # THÊM MỚI: Callback khi người dùng bấm chọn khớp
    # ---------------------------------------------------------------------------
    def _on_joint_select(self):
        """Được gọi mỗi khi người dùng bấm Radio Button chọn khớp khác."""
        joint_names = ["Hip (Hong)", "Knee (Goi)", "Ankle (Co chan)"]
        selected = self.selected_joint.get()
        self.status_text.set(f"Status: Dang xem khop {joint_names[selected]}")
        # Reset trục thời gian để đồ thị vẽ lại từ đầu khi đổi khớp
        self._last_t0 = None

    # ---------------------------------------------------------------------------
    # GUI callbacks
    # ---------------------------------------------------------------------------
    def _on_calib(self):
        try:
            self.controller.run_calibration()
            messagebox.showinfo("Calibration", "Đang tiến hành Calib. Đợi motor kêu tít tít và xoay xong mới bấm thao tác khác nhé!")
        except Exception:
            logger.exception("Calibration error")
    def _on_offset(self):
        try:
            self.controller.set_offset()
            # set color & enabled
            if getattr(self.controller, "isOffset", False):
                self.btn_offset.configure(state="disabled", bg="lightgreen")
            else:
                self.btn_offset.configure(state="normal", bg="tomato")
        except Exception:
            logger.exception("Offset error")

    def _set_target_entry_enabled(self, enable):
        if enable:
            self.control_panel[0][0].configure(state="normal")
        else:
            self.control_panel[0][0].delete(0, tk.END)
            self.control_panel[0][0].configure(state="disabled")

    def _on_toggle_plot(self):
        self.plotting = not self.plotting
        if self.plotting:
            self.btn_plot.config(text="Stop plotting")
        else:
            self.btn_plot.config(text="Continue plotting")

    def _on_estop(self):
        try:
            self.controller.emergency_stop()
            self.btn_estop.config(state="disabled")
        except Exception:
            logger.exception("EStop error")

    def _on_reset(self):
        try:
            self.controller.reset()
            # re-enable estop button after reset
            self.btn_estop.config(state="normal")
        except Exception:
            logger.exception("Reset error")

    def _on_move(self):
        self.max_jerk = 0
        try:
            elms = []
            for entry, var in self.control_panel:
                s = var.get().strip()
                if s == "":
                    elms.append(0.0)
                else:
                    elms.append(float(s))
            if len(elms) < len(self.load_params):
                messagebox.showwarning("Control", "Please fill all control elements")
                return

            self.controller.update_ctrlElms(*elms)
            self.status_text.set("Status: control input sent")
        except Exception:
            logger.exception("Send control input error")
            messagebox.showerror("Error", "Failed to send control input to controller")

    def _on_send_parameters(self):
        try:
            params = []
            for entry, var in self.param_panel:
                s = var.get().strip()
                if s == "":
                    params.append(0.0)
                else:
                    params.append(float(s))
            if len(params) < len(self.load_params):
                messagebox.showwarning("Parameters", "Please fill all parameter fields")
                return

            self.controller.update_loadParms(*params)
            self.status_text.set("Status: parameters sent")
        except Exception:
            logger.exception("Send parameters error")
            messagebox.showerror("Error", "Failed to send parameters to controller")

    def _on_apply_filter(self):
        try:
            window_size = int(self.window_size_var.get().strip())
            poly_order = int(self.poly_order_var.get().strip())

            # Validate filter parameters
            if window_size <= 0:
                messagebox.showwarning("Filter", "Window size must be positive")
                return
            if window_size % 2 == 0:
                messagebox.showwarning("Filter", "Window size must be odd")
                return
            if poly_order >= window_size:
                messagebox.showwarning("Filter", "Poly order must be less than window size")
                return
            if poly_order < 0:
                messagebox.showwarning("Filter", "Poly order must be non-negative")
                return

            # Update controller filter parameters
            with self.controller.data_lock:
                self.controller.window_size = window_size
                self.controller.poly_order = poly_order
                self.controller.velFilBuf = deque(maxlen=window_size)
                self.controller.timeFilBuf = deque(maxlen=window_size)

            self.status_text.set("Status: filter settings updated")
        except ValueError:
            messagebox.showerror("Error", "Please enter valid integers for window size and poly order")
        except Exception:
            logger.exception("Apply filter error")
            messagebox.showerror("Error", "Failed to apply filter settings")

    def _on_mode_tog(self):
        # Toggle closed loop / IDLE on controller
        try:
            state = self.controller.get_state()
            if state == IDLE:
                # try to enter closed loop
                self.controller.enter_closed_loop()
                self.btn_mode.config(text="IDLE", bg="yellow")
                self.send_enable(False)
                self.move_enable(True)
            else:
                self.controller.return_IDLE()
                self.btn_mode.config(text="Close Loop", bg="lightgreen")
                self.send_enable(True)
                self.move_enable(False)  
                if state == None:
                    messagebox.showwarning("Cảnh báo", "Không xác định trạng thái")              

        except Exception:
            logger.exception("Mode toggle error")

    # ---------------------------------------------------------------------------
    # Combined update: poll controller state, update GUI widgets, update plots
    # ---------------------------------------------------------------------------
    def _update(self):
        try:
            # Build a state dict from controller attributes (safe access)
            try:
                connected = bool(getattr(self.controller, "connected", False))
                closed_loop = bool(getattr(self.controller, "closed_loop_control", False))
                is_offset = bool(getattr(self.controller, "isOffset", False))
                estop = bool(getattr(self.controller, "_estop_event", threading.Event()).is_set())
                error = bool(getattr(self.controller, "error", False))
            except Exception:
                connected = closed_loop = is_offset = estop = error = False

            # Update status label (color + text)
            if estop:
                self.status_label.config(text="ESTOP", background="red")
            elif error:
                self.status_label.config(text="ERROR", background="orange")
            elif connected:
                self.status_label.config(text="Connected", background="lightgreen")
            else:
                self.status_label.config(text="Disconnected", background="lightgrey")

            # Update mode button text
            if closed_loop:
                self.btn_mode.config(text="IDLE", bg="yellow")
            else:
                self.btn_mode.config(text="Close Loop", bg="lightgreen")

            # Enable/disable buttons depending on mode and offset
            self.send_enable(enabled=(not closed_loop))
            self.move_enable(enabled=(is_offset and closed_loop))
            self.btn_apply_filter.configure(state="normal" if connected else "disabled")

            # Enable/disable target entry depending on offset
            if is_offset:
                self._set_target_entry_enabled(True)
                self.btn_offset.configure(state="disabled", bg="lightgreen")
            else:
                self._set_target_entry_enabled(False)
                self.btn_offset.configure(state="normal", bg="tomato")

            # Get data for plotting and display
            # =======================================================================
            # THÊM MỚI: Nếu có UART (inject từ main.py), đọc telemetry của khớp
            # đang được chọn từ ESP32. Nếu không có, đọc từ ODrive trực tiếp như cũ.
            # =======================================================================
            data = []
            joint_idx = self.selected_joint.get()  # 0=HIP, 1=KNEE, 2=ANKLE

            try:
                if self.uart is not None and self.uart.is_connected():
                    # --- Chế độ UART: Đọc telemetry từ ESP32 ---
                    # Lấy bản sao an toàn của toàn bộ telemetry
                    tel = self.uart.telemetry
                    if tel.last_updated_time > 0:
                        j = tel.joints[joint_idx]
                        t_now = tel.last_updated_time
                        # Chuyển sang tuple giống format của ODriveThread.get_data()
                        # để các dòng vẽ đồ thị bên dưới không cần sửa gì!
                        # Format: (time, pos, vel, acc, pos_set, vel_set, acc_set, jerk, torque)
                        data = [(t_now, j.actual_pos, j.actual_vel, j.actual_acc,
                                 j.set_pos, j.set_vel, j.set_acc, 0.0, j.torque_set)]
                else:
                    # --- Chế độ ODrive trực tiếp (legacy): Đọc từ controller ---
                    data = self.controller.get_data()
            except Exception:
                data = []

            if data:
                # data is list of tuples: (time, pos, vel, pos_set, vel_set, tor)
                times, pos_vals, vel_vals, acc_vals, pos_set_vals, vel_set_vals, acc_set_vals, _, tor_vals = zip(*data)
                # normalize time origin to the first sample in the list (keeps plotting stable)
                t0 = times[0] if self._last_t0 is None else self._last_t0
                # If a big discontinuity, reset origin to current first sample
                if self._last_t0 is None or (times[-1] - t0) > 30.0:
                    t0 = times[0]
                    self._last_t0 = t0
                times_rel = [t - t0 for t in times]

                # Update plot lines if plotting enabled
                if self.plotting:
                    self._pos_line.set_data(times_rel, pos_vals)
                    self._pos_set_line.set_data(times_rel, pos_set_vals)
                    self._vel_line.set_data(times_rel, vel_vals)
                    self._vel_set_line.set_data(times_rel, vel_set_vals)
                    self._acc_line.set_data(times_rel, acc_vals)
                    self._acc_set_line.set_data(times_rel, acc_set_vals)
                    self._tor_line.set_data(times_rel, tor_vals)

                    # autoscale each axis
                    def autoscale(ax, x, y):
                        if not x or not y:
                            return
                        ax.relim()
                        ax.autoscale_view()
                    autoscale(self.ax_pos, times_rel, pos_vals + pos_set_vals if isinstance(pos_vals, tuple) else pos_vals)
                    autoscale(self.ax_vel, times_rel, vel_vals + vel_set_vals if isinstance(vel_vals, tuple) else vel_vals)
                    autoscale(self.ax_acc, times_rel, acc_vals + acc_set_vals if isinstance(vel_vals, tuple) else acc_vals)
                    autoscale(self.ax_tor, times_rel, tor_vals)

                    self.ax_pos.set_ylabel("deg")
                    self.ax_vel.set_ylabel("deg/s")
                    self.ax_acc.set_ylabel("deg/s^2")
                    self.ax_tor.set_ylabel("Nm")
                    self.ax_tor.set_xlabel("Time (s)")
                    self.canvas.draw_idle()

                # Update displayed numeric values from the latest sample
                _, last_pos, last_vel, _, last_pos_set, last_vel_set, _, last_jerk, _ = data[-1]
                
                # Show current position
                try:
                    self.entry_pos.config(state="normal")
                    self.entry_pos.delete(0, tk.END)
                    self.entry_pos.insert(0, f"{last_pos:.3f}")
                    self.entry_pos.config(state="readonly")
                except Exception:
                    pass

                # Update errors (setpoint minus actual)
                pos_err = last_pos_set - last_pos
                vel_err = last_vel_set - last_vel
                
                try:
                    self.entry_pos_error.config(state="normal")
                    self.entry_pos_error.delete(0, tk.END)
                    self.entry_pos_error.insert(0, f"{pos_err:.3f}")
                    self.entry_pos_error.config(state="readonly")

                    self.entry_vel_error.config(state="normal")
                    self.entry_vel_error.delete(0, tk.END)
                    self.entry_vel_error.insert(0, f"{vel_err:.3f}")
                    self.entry_vel_error.config(state="readonly")

                    if self.max_jerk < abs(last_jerk):
                        self.max_jerk = abs(last_jerk)
                        self.max_jerk_var.set(f"{self.max_jerk:.3f}")  


                except Exception:
                    pass

            else:
                pass

            # Update status text minimally
            if connected:
                self.status_text.set("Status: running")
            else:
                self.status_text.set("Status: controller unavailable")

        except Exception:
            logger.exception("Update loop error")

        # schedule next update
        self.after(UPDATE_INTERVAL_MS, self._update)

    def move_enable(self, enabled):
        """Enable/disable Move button based on offset and idle state"""
        btn_state = "normal" if enabled else "disabled"
        self.btn_move.configure(state=btn_state)

    def send_enable(self, enabled):
        """Enable/disable Send button based on idle state"""
        btn_state = "normal" if enabled else "disabled"
        self.btn_send_param.configure(state=btn_state)

    def _on_close(self):
        # Confirm close
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            try:
                # Attempt graceful shutdown of controller thread
                try:
                    # prefer stop() which exists in the provided controller
                    self.controller.stop()
                    # give it a moment and join
                    if hasattr(self.controller, "join"):
                        self.controller.join(timeout=2.0)
                except Exception:
                    logger.exception("Error shutting down controller")
            finally:
                self.destroy()

if __name__ == "__main__":
    app = ControlGUI()
    app.mainloop()