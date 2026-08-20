#!/usr/bin/env python3
"""
sine_plot_gui.py - Giao diện đồ thị thời gian thực hiển thị sóng Sin từ ESP32
Chạy trên Raspberry Pi 5 để nhận dữ liệu UART và vẽ đồ thị cuộn.

Cách chạy:
    python3 sine_plot_gui.py                  (Tự động tìm cổng)
    python3 sine_plot_gui.py /dev/ttyUSB0     (Chỉ định cổng)
"""

import sys
import time
import threading
from collections import deque

# Import uart_handler (cùng thư mục)
from uart_handler import ESP32UARTManager, RobotTelemetry, JointTelemetry

# ============================================================================
# THƯ VIỆN ĐỒ HỌA
# ============================================================================
try:
    import tkinter as tk
except ImportError:
    print("[!] Chua cai tkinter. Chay: sudo apt install python3-tk")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("TkAgg")  # Backend cho Tkinter
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
except ImportError:
    print("[!] Chua cai matplotlib. Chay: pip3 install matplotlib")
    sys.exit(1)


# ============================================================================
# CẤU HÌNH
# ============================================================================
MAX_POINTS = 200       # Số điểm hiển thị trên đồ thị (4 giây @ 50Hz)
UPDATE_MS  = 50        # Cập nhật giao diện mỗi 50ms (20 FPS)

JOINT_NAMES  = ["Hip (Hong)", "Knee (Goi)", "Ankle (Co chan)"]
JOINT_COLORS = ["#FF6B6B", "#4ECDC4", "#45B7D1"]  # Đỏ, Xanh ngọc, Xanh dương


# ============================================================================
# LỚP GIAO DIỆN CHÍNH
# ============================================================================
class SinePlotApp:
    def __init__(self, uart_mgr: ESP32UARTManager):
        self.uart_mgr = uart_mgr

        # Bộ đệm vòng (Ring Buffer) lưu 200 điểm gần nhất cho mỗi khớp
        self.time_buf = deque(maxlen=MAX_POINTS)
        self.pos_buf  = [deque(maxlen=MAX_POINTS) for _ in range(3)]  # 3 khớp
        self.set_buf  = [deque(maxlen=MAX_POINTS) for _ in range(3)]  # Setpoint

        self.t_start = time.time()
        self.packet_count = 0

        # --- Tạo cửa sổ Tkinter ---
        self.root = tk.Tk()
        self.root.title("ESP32 -> Pi5 | UART Sine Wave Monitor")
        self.root.configure(bg="#1a1a2e")

        # --- Tiêu đề ---
        title = tk.Label(
            self.root,
            text="DO THI SONG SIN TU ESP32 (UART 921600 baud)",
            font=("Helvetica", 14, "bold"),
            fg="#e94560", bg="#1a1a2e"
        )
        title.pack(pady=5)

        # --- Thông tin kết nối ---
        self.info_var = tk.StringVar(value="Dang cho du lieu...")
        info_label = tk.Label(
            self.root, textvariable=self.info_var,
            font=("Courier", 10), fg="#0f3460", bg="#e0e0e0",
            relief="sunken", padx=10, pady=3
        )
        info_label.pack(fill="x", padx=10, pady=2)

        # --- Matplotlib Figure (3 subplot xếp dọc) ---
        self.fig = Figure(figsize=(9, 6), dpi=100, facecolor="#16213e")
        self.axes = []
        for i in range(3):
            ax = self.fig.add_subplot(3, 1, i + 1)
            ax.set_facecolor("#0f3460")
            ax.set_title(JOINT_NAMES[i], color="white", fontsize=10, pad=3)
            ax.set_ylabel("Goc (do)", color="white", fontsize=8)
            ax.tick_params(colors="white", labelsize=7)
            ax.set_xlim(0, MAX_POINTS)
            ax.set_ylim(-60, 60)
            ax.grid(True, alpha=0.2, color="white")
            ax.spines['bottom'].set_color('#444')
            ax.spines['left'].set_color('#444')
            self.axes.append(ax)

        self.axes[2].set_xlabel("Thoi gian (mau)", color="white", fontsize=8)
        self.fig.tight_layout(pad=2.0)

        # Vẽ canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=5)

        # Khởi tạo các đường vẽ (line objects)
        self.lines_actual = []
        self.lines_set = []
        for i in range(3):
            line_act, = self.axes[i].plot([], [], color=JOINT_COLORS[i], linewidth=1.5, label="Actual")
            line_set, = self.axes[i].plot([], [], color="yellow", linewidth=1, linestyle="--", alpha=0.6, label="Setpoint")
            self.lines_actual.append(line_act)
            self.lines_set.append(line_set)
            self.axes[i].legend(loc="upper right", fontsize=7, facecolor="#16213e", edgecolor="#444", labelcolor="white")

        # --- Nút điều khiển ---
        btn_frame = tk.Frame(self.root, bg="#1a1a2e")
        btn_frame.pack(fill="x", padx=10, pady=5)

        self.btn_clear = tk.Button(
            btn_frame, text="Xoa Do Thi", command=self._clear_plot,
            bg="#e94560", fg="white", font=("Helvetica", 10, "bold"),
            relief="flat", padx=15, pady=5
        )
        self.btn_clear.pack(side="left", padx=5)

        self.btn_quit = tk.Button(
            btn_frame, text="Thoat", command=self._quit,
            bg="#533483", fg="white", font=("Helvetica", 10, "bold"),
            relief="flat", padx=15, pady=5
        )
        self.btn_quit.pack(side="right", padx=5)

        # --- Đăng ký callback nhận telemetry ---
        self.uart_mgr.register_telemetry_callback(self._on_telemetry)

        # --- Bắt đầu vòng lặp cập nhật đồ thị ---
        self.root.after(UPDATE_MS, self._update_plot)

        # --- Bắt sự kiện đóng cửa sổ ---
        self.root.protocol("WM_DELETE_WINDOW", self._quit)

    def _on_telemetry(self, data: RobotTelemetry):
        """Callback được gọi từ luồng RX khi nhận 1 gói telemetry"""
        self.packet_count += 1
        t_now = time.time() - self.t_start
        self.time_buf.append(t_now)

        for i in range(3):
            j = data.joints[i]
            self.pos_buf[i].append(j.actual_pos)
            self.set_buf[i].append(j.set_pos)

    def _update_plot(self):
        """Cập nhật đồ thị từ buffer (chạy trên Main Thread của Tkinter)"""
        n = len(self.time_buf)
        if n > 1:
            x_data = list(range(n))  # Trục X = số thứ tự mẫu

            for i in range(3):
                self.lines_actual[i].set_data(x_data, list(self.pos_buf[i]))
                self.lines_set[i].set_data(x_data, list(self.set_buf[i]))
                self.axes[i].set_xlim(0, max(n, MAX_POINTS))

            self.canvas.draw_idle()

            # Cập nhật thông tin
            self.info_var.set(
                f"Cong: {self.uart_mgr.port} | "
                f"Packets: {self.uart_mgr.packets_received} OK, "
                f"{self.uart_mgr.packets_crc_error} CRC err | "
                f"FPS: ~{1000 // UPDATE_MS}"
            )

        # Lặp lại sau UPDATE_MS
        self.root.after(UPDATE_MS, self._update_plot)

    def _clear_plot(self):
        """Xóa toàn bộ dữ liệu đồ thị"""
        self.time_buf.clear()
        for i in range(3):
            self.pos_buf[i].clear()
            self.set_buf[i].clear()
        self.t_start = time.time()

    def _quit(self):
        """Dọn dẹp và thoát"""
        self.uart_mgr.disconnect()
        self.root.quit()
        self.root.destroy()

    def run(self):
        """Bắt đầu vòng lặp Tkinter"""
        self.root.mainloop()


# ============================================================================
# ĐIỂM BẮT ĐẦU CHƯƠNG TRÌNH
# ============================================================================
if __name__ == "__main__":
    print("=" * 50)
    print("  GIAO DIEN DO THI SONG SIN - ESP32 UART")
    print("=" * 50)

    # Chọn cổng
    port = sys.argv[1] if len(sys.argv) > 1 else None

    # Khởi tạo UART
    uart_mgr = ESP32UARTManager()

    if port is None:
        port = uart_mgr.auto_detect_port()

    print(f"[*] Dang ket noi cong: {port}")
    success = uart_mgr.connect(port)

    if not success:
        print(f"[!] Khong the ket noi {port}!")
        print("    Thu lai voi: python3 sine_plot_gui.py /dev/ttyUSB0")
        sys.exit(1)

    print(f"[OK] Da ket noi {port} @ 921600 baud")
    print("[*] Dang mo giao dien do thi...")

    # Mở giao diện
    app = SinePlotApp(uart_mgr)
    app.run()
