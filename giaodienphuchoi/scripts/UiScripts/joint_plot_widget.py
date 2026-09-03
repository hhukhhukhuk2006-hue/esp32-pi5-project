"""joint_plot_widget.py — Widget đồ thị 3x3 realtime dùng pyqtgraph.

Tính năng:
    - Ma trận 9 đồ thị: 3 hàng (Pos/Vel/Acc) × 3 cột (Hip/Knee/Ankle).
    - Mỗi ô có 2 đường: Actual (nét liền) và Setpoint (nét đứt vàng).
    - Lazy Rendering: Chỉ vẽ khi widget đang hiện, tự TẮT khi ẩn → Pi 5 không bị lag.
    - Click đúp vào bất kỳ ô nào → Phóng to toàn màn hình. Click đúp lần nữa → Thu nhỏ.

Cách nhúng vào DevMainScreen (mainscreen.py):
    from UiScripts.joint_plot_widget import JointPlotWidget

    # Tạo và nhúng widget vào frame_control của màn hình KTV:
    self.plot_widget = JointPlotWidget(parent=self.ui.frame_control)
    self.plot_widget.setGeometry(0, 45, 840, 605)
    self.plot_widget.hide()  # Ẩn ban đầu

    # Nút toggle giữa giao diện điều khiển và đồ thị:
    self.btn_toggle_plot.clicked.connect(self._toggle_plot_view)

Cách cấp dữ liệu:
    # Gọi mỗi lần nhận được gói tin UART từ ESP32 (trong data_receive()):
    self.plot_widget.push_data(
        pos_actual=[hip_pos, knee_pos, ankle_pos],   # đơn vị: degree
        vel_actual=[hip_vel, knee_vel, ankle_vel],   # đơn vị: deg/s
        acc_actual=[hip_acc, knee_acc, ankle_acc],   # đơn vị: deg/s²
        pos_set   =[hip_ps,  knee_ps,  ankle_ps ],   # Setpoint pos
        vel_set   =[hip_vs,  knee_vs,  ankle_vs ],   # Setpoint vel
        acc_set   =[hip_as,  knee_as,  ankle_as ],   # Setpoint acc
    )
"""

from __future__ import annotations
from collections import deque

import pyqtgraph as pg
from PyQt5.QtCore  import QTimer, Qt
from PyQt5.QtWidgets import (QWidget, QGridLayout, QSizePolicy,
                              QPushButton, QHBoxLayout, QVBoxLayout, QLabel)
from PyQt5.QtGui   import QFont

from UiScripts.session_summary_dialog import SessionDataTracker, SessionSummaryDialog

# ─────────────────────────────────────────────────────────────────────────────
#  Cấu hình hiển thị
# ─────────────────────────────────────────────────────────────────────────────
BUFFER_SIZE  = 300          # Số mẫu lưu trong bộ đệm vòng (≈ 6 giây @ 50 Hz)
UPDATE_MS    = 50           # Chu kỳ vẽ lại đồ thị (ms) — 20 FPS là đủ mượt

# Tên 3 cột (khớp)
JOINT_NAMES = ["Hip (Hông)", "Knee (Gối)", "Ankle (Cổ chân)"]

# Màu đường Actual theo từng khớp
JOINT_COLORS = ["#FF6B6B", "#4ECDC4", "#45B7D1"]   # Đỏ / Xanh ngọc / Xanh dương

# Màu đường Setpoint (dùng chung cho cả 3 khớp)
SETPOINT_COLOR = "#FFD700"  # Vàng gold

# Tên 3 hàng (loại dữ liệu)
ROW_LABELS  = ["Vị trí (°)", "Vận tốc (°/s)", "Gia tốc (°/s²)"]

# Giới hạn trục Y mặc định cho từng hàng [min, max]
Y_RANGES = [
    (-120.0, 120.0),    # Pos  (độ)
    (-200.0, 200.0),    # Vel  (độ/s)
    (-500.0, 500.0),    # Acc  (độ/s²)
]

# Nền sáng cho pyqtgraph để hợp với giao diện KTV
pg.setConfigOption("background", "#FFFFFF")   # Nền trắng
pg.setConfigOption("foreground", "#000000")   # Chữ đen


# ─────────────────────────────────────────────────────────────────────────────
#  JointPlotWidget — Widget chính
# ─────────────────────────────────────────────────────────────────────────────
class JointPlotWidget(QWidget):
    """Widget ma trận 3×3 đồ thị realtime với Lazy Rendering + click đúp phóng to."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # ── Bộ đệm vòng (Ring Buffers) ─────────────────────────────────────
        # Cấu trúc: buffers[row][col] = deque
        # row: 0=Pos, 1=Vel, 2=Acc   |   col: 0=Hip, 1=Knee, 2=Ankle
        self.buf_actual = [[deque(maxlen=BUFFER_SIZE) for _ in range(3)] for _ in range(3)]
        self.buf_set    = [[deque(maxlen=BUFFER_SIZE) for _ in range(3)] for _ in range(3)]

        # ── Trạng thái phóng to ────────────────────────────────────────────
        # None = đang ở chế độ 3×3. (row, col) = ô đang được phóng to.
        self._zoomed: tuple[int, int] | None = None

        # ── Tracker tổng kết phiên đo ──────────────────────────────────────
        self._tracker = SessionDataTracker()
        self._was_visible: bool = False   # Để phân biệt hide lúc khởi động vs. bấm tắt

        # ── Build layout ───────────────────────────────────────────────────
        self._build_ui()

        # ── Timer Lazy Rendering ───────────────────────────────────────────
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_plots)
        # Timer KHÔNG start ngay — chỉ start khi widget hiện (showEvent)

    # =========================================================================
    #  BUILD GIAO DIỆN
    # =========================================================================
    def _build_ui(self):
        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(4, 4, 4, 4)
        root_layout.setSpacing(4)

        # ── Thanh tiêu đề + nút thu nhỏ (chỉ hiện khi phóng to) ───────────
        header = QHBoxLayout()
        self._lbl_title = QLabel("📈  Đồ thị 3 Khớp  |  Thời gian thực")
        self._lbl_title.setStyleSheet("color:#e0e0e0; font-weight:bold; font-size:12px;")
        header.addWidget(self._lbl_title)
        header.addStretch()

        self._btn_shrink = QPushButton("✕ Thu nhỏ")
        self._btn_shrink.setFixedSize(100, 28)
        self._btn_shrink.setStyleSheet(
            "QPushButton{background:#e94560;color:white;border-radius:6px;font-size:11px;}"
            "QPushButton:hover{background:#c73652;}"
        )
        self._btn_shrink.clicked.connect(self._exit_zoom)
        self._btn_shrink.hide()   # Ẩn ban đầu, hiện khi phóng to
        header.addWidget(self._btn_shrink)
        root_layout.addLayout(header)

        # ── Grid 3×3 ───────────────────────────────────────────────────────
        self._grid_widget = QWidget()
        self._grid = QGridLayout(self._grid_widget)
        self._grid.setSpacing(3)
        root_layout.addWidget(self._grid_widget, stretch=1)

        # Lưu tham chiếu đến từng PlotWidget và PlotDataItem
        self._plot_widgets: list[list[pg.PlotWidget]] = []
        self._curves_actual: list[list[pg.PlotDataItem]] = []
        self._curves_set:    list[list[pg.PlotDataItem]] = []

        for row in range(3):
            pw_row, ca_row, cs_row = [], [], []
            for col in range(3):
                pw = self._create_plot(row, col)
                self._grid.addWidget(pw, row, col)
                pw_row.append(pw)

                # Đường Actual (nét liền)
                ca = pw.plot(pen=pg.mkPen(JOINT_COLORS[col], width=2))
                # Đường Setpoint (nét đứt vàng)
                cs = pw.plot(pen=pg.mkPen(SETPOINT_COLOR, width=1.5,
                                          style=Qt.DashLine))
                ca_row.append(ca)
                cs_row.append(cs)

            self._plot_widgets.append(pw_row)
            self._curves_actual.append(ca_row)
            self._curves_set.append(cs_row)

        # ── OVERLAY WIDGET cho chế độ phóng to ──────────────────────────────────
        # Không đụng vào grid layout — overlay chỉ phủ lên trên bằng setGeometry.
        # Nhờ đó grid gốc KHOONG BAO GIờ bị thay đổi → thu nhỏ luôn chuẩn.
        self._zoom_overlay = pg.PlotWidget(parent=self._grid_widget)
        self._zoom_overlay.setBackground("#F5F5F5")  # Nền sáng cho overlay
        self._zoom_overlay.showGrid(x=True, y=True, alpha=0.3)
        self._zoom_overlay.hide()   # Ẩn ban đầu

        # 2 đường vẽ trên overlay (Actual + Setpoint)
        self._zoom_curve_actual = self._zoom_overlay.plot(
            pen=pg.mkPen("#000000", width=3))   # Đen đậm, sẽ đổi màu khi mở
        self._zoom_curve_set    = self._zoom_overlay.plot(
            pen=pg.mkPen(SETPOINT_COLOR, width=2, style=Qt.DashLine))

        # Click đúp lên overlay → đóng phóng to
        self._zoom_overlay.mouseDoubleClickEvent = lambda ev: self._exit_zoom()

    def _create_plot(self, row: int, col: int) -> pg.PlotWidget:
        """Tạo 1 ô đồ thị với nhãn trục và sự kiện click đúp."""
        pw = pg.PlotWidget()
        pw.setBackground("#FFFFFF")
        pw.showGrid(x=True, y=True, alpha=0.3)
        pw.setYRange(*Y_RANGES[row], padding=0)
        pw.setXRange(0, BUFFER_SIZE, padding=0)
        pw.getAxis("bottom").setStyle(showValues=False)     # Ẩn số trục X (chỉ cần hàng dưới cùng)

        # Nhãn hàng bên trái (chỉ cột đầu tiên)
        if col == 0:
            pw.setLabel("left", ROW_LABELS[row],
                        color="#000000", size="9pt")
        else:
            pw.setLabel("left", "")

        # Tiêu đề cột (chỉ hàng đầu tiên)
        if row == 0:
            pw.setTitle(JOINT_NAMES[col],
                        color=JOINT_COLORS[col], size="10pt")

        # Hiện số trục X ở hàng cuối cùng
        if row == 2:
            pw.setLabel("bottom", "Mẫu", color="#000000", size="8pt")
            pw.getAxis("bottom").setStyle(showValues=True)

        # Sự kiện click đúp → phóng to / thu nhỏ
        pw.mouseDoubleClickEvent = lambda ev, r=row, c=col: self._on_double_click(r, c)

        return pw

    # =========================================================================
    #  LAZY RENDERING — Tự bật/tắt timer khi widget hiện/ẩn
    # =========================================================================
    def showEvent(self, event):
        """Bật timer và bắt đầu phiên đo mới khi Tab Đồ thị được mở."""
        super().showEvent(event)
        self._timer.start(UPDATE_MS)
        self._was_visible = True
        self._is_recording = True
        self._tracker.reset()

    def hideEvent(self, event):
        """Tắt timer và DừNG phiên đo khi quay lại màn hình Điều Khiển.
        Luôn hiện popup tóm tắt (có data → bảng đầy đủ / không có data → thông báo).
        """
        super().hideEvent(event)
        self._timer.stop()
        self._is_recording = False
        if self._was_visible:
            self._show_summary()
        self._was_visible = False

    # =========================================================================
    #  NHẬN DỮ LIỆU TỪ BÊN NGOÀI (gọi từ data_receive() trong mainscreen.py)
    # =========================================================================
    def push_data(self,
                  pos_actual: list[float], vel_actual: list[float], acc_actual: list[float],
                  pos_set:    list[float], vel_set:    list[float], acc_set:    list[float],
                  torque:     list[float] | None = None):
        """
        Đẩy 1 mẫu dữ liệu mới vào bộ đệm cho cả 3 khớp.

        Args:
            pos_actual: [hip, knee, ankle] — Góc thực tế (độ)
            vel_actual: [hip, knee, ankle] — Vận tốc thực tế (độ/s)
            acc_actual: [hip, knee, ankle] — Gia tốc thực tế (độ/s²)
            pos_set:    [hip, knee, ankle] — Setpoint vị trí
            vel_set:    [hip, knee, ankle] — Setpoint vận tốc
            acc_set:    [hip, knee, ankle] — Setpoint gia tốc
            torque:     [hip, knee, ankle] — Mô-men xoắn (Nm), tùy chọn
        """
        actual_rows = [pos_actual, vel_actual, acc_actual]
        set_rows    = [pos_set,    vel_set,    acc_set]

        for row in range(3):
            for col in range(3):
                self.buf_actual[row][col].append(actual_rows[row][col])
                self.buf_set[row][col].append(set_rows[row][col])

        # ── Ghi vào tracker phiên đo ──────────────────────────────────────
        self._tracker.record(
            pos_actual, vel_actual, acc_actual,
            pos_set,    vel_set,    acc_set,
            torque=torque,
        )

    # =========================================================================
    #  VẼ LẠI ĐỒ THỊ (gọi theo chu kỳ bởi QTimer)
    # =========================================================================
    def _refresh_plots(self):
        """Vẽ lại toàn bộ đồ thị từ dữ liệu trong bộ đệm."""
        # — 1. Cập nhật lưới 3×3 (luôn chạy, dù overlay đang hiện) —
        for row in range(3):
            for col in range(3):
                y_actual = list(self.buf_actual[row][col])
                y_set    = list(self.buf_set[row][col])
                n = len(y_actual)
                if n == 0:
                    continue
                x = list(range(n))
                self._curves_actual[row][col].setData(x, y_actual)
                self._curves_set[row][col].setData(x, y_set)

        # — 2. Cập nhật overlay nếu đang phóng to —
        if self._zoomed is not None:
            z_row, z_col = self._zoomed
            y_actual = list(self.buf_actual[z_row][z_col])
            y_set    = list(self.buf_set[z_row][z_col])
            n = len(y_actual)
            if n > 0:
                x = list(range(n))
                self._zoom_curve_actual.setData(x, y_actual)
                self._zoom_curve_set.setData(x, y_set)

    # =========================================================================
    #  PHÓNG TO / THU NHỎ (Click đúp chuột)
    # =========================================================================
    def _on_double_click(self, row: int, col: int):
        """Xử lý sự kiện click đúp vào 1 ô đồ thị."""
        if self._zoomed is None:
            self._enter_zoom(row, col)
        else:
            self._exit_zoom()

    def _enter_zoom(self, zoom_row: int, zoom_col: int):
        """
        Phóng to ô (zoom_row, zoom_col) bằng overlay widget.

        Chiến lược: KHOONG đụng vào grid gốc.
        Overlay phủ lên toàn bộ _grid_widget bằng setGeometry.
        Khi ẩn overlay → grid gốc hiện ra nguyên vẹn.
        """
        self._zoomed = (zoom_row, zoom_col)

        # Cấu hình overlay theo ô được chọn
        color = JOINT_COLORS[zoom_col]
        title = f"{ROW_LABELS[zoom_row]}  —  {JOINT_NAMES[zoom_col]}"
        self._zoom_overlay.setTitle(title, color=color, size="13pt")
        self._zoom_overlay.setLabel("left",   ROW_LABELS[zoom_row],
                                    color="#000000", size="11pt")
        self._zoom_overlay.setLabel("bottom", "Mẫu",
                                    color="#000000", size="10pt")
        self._zoom_overlay.getAxis("bottom").setStyle(showValues=True)
        self._zoom_overlay.setYRange(*Y_RANGES[zoom_row], padding=0.05)
        self._zoom_overlay.setXRange(0, BUFFER_SIZE, padding=0)

        # Đổi màu đường Actual theo cột (khớp)
        self._zoom_curve_actual.setPen(pg.mkPen(color, width=3))

        # Khởi động với dữ liệu hiện có
        y_a = list(self.buf_actual[zoom_row][zoom_col])
        y_s = list(self.buf_set[zoom_row][zoom_col])
        if y_a:
            x = list(range(len(y_a)))
            self._zoom_curve_actual.setData(x, y_a)
            self._zoom_curve_set.setData(x, y_s)

        # Phủ overlay lên toàn bộ _grid_widget
        self._zoom_overlay.setGeometry(self._grid_widget.rect())
        self._zoom_overlay.raise_()    # Lên trên cùng
        self._zoom_overlay.show()

        # Cập nhật thanh tiêu đề
        self._btn_shrink.show()
        self._lbl_title.setText(
            f"📈  {JOINT_NAMES[zoom_col]}  |  {ROW_LABELS[zoom_row]}  "
            f"— Double-click để thu nhỏ"
        )

    def _exit_zoom(self):
        """
        Thu nhỏ về chế độ 3×3.

        Chỉ ẩn overlay — grid gốc vẫn nguyên vẹn, không cần khôi phục gì cả.
        """
        if self._zoomed is None:
            return
        self._zoomed = None
        self._zoom_overlay.hide()     # Ẩn overlay → grid 3×3 lộ ra
        self._btn_shrink.hide()
        self._lbl_title.setText("📈  Đồ thị 3 Khớp  |  Thời gian thực")

    # =========================================================================
    #  POPUP TÓM TẮT PHIÊN ĐO
    # =========================================================================
    def _show_summary(self):
        """Hiển thị popup tóm tắt phiên đo, rồi reset tracker cho lần chạy tiếp."""
        dlg = SessionSummaryDialog(self._tracker, parent=self.window())
        dlg.exec()          # Modal — chờ người dùng đóng
        self._tracker.reset()   # Reset để phiên chạy tiếp được tính mới

    def resizeEvent(self, event):
        """Khi widget bị resize: cập nhật kích thước overlay cho khớp."""
        super().resizeEvent(event)
        if self._zoomed is not None:
            self._zoom_overlay.setGeometry(self._grid_widget.rect())

