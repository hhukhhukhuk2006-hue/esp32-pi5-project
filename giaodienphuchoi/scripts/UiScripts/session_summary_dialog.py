"""session_summary_dialog.py — Popup tổng kết phiên đo sau khi tắt đồ thị.

Hiển thị:
    - Bảng 5 mẫu đầu tiên và 5 mẫu cuối cùng nhận được từ ESP32.
    - Khối thống kê tóm tắt: Max Torque, Max Tracking Error, tổng gói tin.
    - Nút xuất CSV.
"""

from __future__ import annotations

import csv
import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QTableWidget, QTableWidgetItem,
    QHeaderView, QFrame, QFileDialog, QWidget,
    QAbstractItemView, QSizePolicy,
)

# =============================================================================
#  CẤU TRÚC LƯU MỘT MẪU DỮ LIỆU
# =============================================================================
@dataclass
class DataSample:
    """Một mẫu gồm 3 khớp x (pos/vel/acc actual+set + torque)."""
    timestamp:  float       = 0.0
    pos_actual: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    vel_actual: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    acc_actual: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    torque:     List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pos_set:    List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    vel_set:    List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    acc_set:    List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


# =============================================================================
#  TRACKER — Nhẹ, không tốn RAM
# =============================================================================
class SessionDataTracker:
    """
    Thu thập dữ liệu phiên đo: 5 mẫu đầu + 5 mẫu cuối.
    Tính max metrics trực tiếp — không lưu toàn bộ lịch sử.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.first_5: List[DataSample] = []
        self.last_5:  deque            = deque(maxlen=5)
        self.total_packets:      int   = 0
        self.start_time: Optional[float] = None
        self.end_time:   Optional[float] = None
        self.max_tracking_error: List[float] = [0.0, 0.0, 0.0]
        self.max_torque:         List[float] = [0.0, 0.0, 0.0]
        self.min_torque:         List[float] = [0.0, 0.0, 0.0]

    def record(self,
               pos_actual, vel_actual, acc_actual,
               pos_set,    vel_set,    acc_set,
               torque=None):
        now = time.time()
        if self.start_time is None:
            self.start_time = now
        self.end_time = now
        self.total_packets += 1

        if torque is None:
            torque = [0.0, 0.0, 0.0]

        sample = DataSample(
            timestamp=now,
            pos_actual=list(pos_actual), vel_actual=list(vel_actual),
            acc_actual=list(acc_actual), torque=list(torque),
            pos_set=list(pos_set),       vel_set=list(vel_set),
            acc_set=list(acc_set),
        )

        if len(self.first_5) < 5:
            self.first_5.append(sample)
        self.last_5.append(sample)

        for i in range(3):
            err = abs(pos_set[i] - pos_actual[i])
            self.max_tracking_error[i] = max(self.max_tracking_error[i], err)
            self.max_torque[i] = max(self.max_torque[i], torque[i])
            self.min_torque[i] = min(self.min_torque[i], torque[i])

    @property
    def duration_sec(self) -> float:
        if self.start_time and self.end_time:
            return self.end_time - self.start_time
        return 0.0

    def has_data(self) -> bool:
        return self.total_packets > 0


# =============================================================================
#  DIALOG POPUP
# =============================================================================
_JOINT_NAMES = ["Hip (Hông)", "Knee (Gối)", "Ankle (Cổ chân)"]
_COL_HEADERS = [
    "Mẫu #", "Khớp",
    "Pos Act (°)", "Pos Set (°)", "Err Pos (°)",
    "Vel Act (°/s)", "Vel Set (°/s)",
    "Acc Act (°/s²)", "Acc Set (°/s²)",
    "Torque (Nm)",
]
_COLOR_FIRST = QColor("#E8F5E9")   # Xanh lá nhạt — 5 mẫu đầu
_COLOR_LAST  = QColor("#FFF3E0")   # Cam nhạt      — 5 mẫu cuối


class SessionSummaryDialog(QDialog):
    """Popup: Bảng 5 đầu + 5 cuối + thống kê tóm tắt."""

    def __init__(self, tracker: SessionDataTracker, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._tracker = tracker
        self._all_rows = []
        self.setWindowTitle("Tóm tắt Phiên Đo — 5 Đầu & 5 Cuối")
        self.setMinimumSize(1050, 620)
        self.setModal(True)
        self._build_ui()
        self._populate()

    # ─────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        self.setStyleSheet("""
            QDialog { background:#F7F9FC; font-family:'Segoe UI',sans-serif; }
            QLabel#title  { font-size:15px; font-weight:bold; color:#1A237E; }
            QLabel#section{ font-size:12px; font-weight:bold; color:#37474F; padding:4px 0; }
            QTableWidget  { background:#FFF; border:1px solid #CFD8DC;
                            border-radius:6px; gridline-color:#ECEFF1; font-size:11px; }
            QHeaderView::section { background:#1A237E; color:white; font-weight:bold;
                                   font-size:11px; padding:5px 4px; border:none; }
            QPushButton#btn_export { background:#1565C0; color:white; font-weight:bold;
                                     font-size:12px; border-radius:7px; padding:7px 20px; }
            QPushButton#btn_export:hover { background:#0D47A1; }
            QPushButton#btn_close  { background:#CFD8DC; color:#263238; font-weight:bold;
                                     font-size:12px; border-radius:7px; padding:7px 20px; }
            QPushButton#btn_close:hover  { background:#B0BEC5; }
            QFrame#stat_card { background:#FFF; border:1px solid #CFD8DC;
                               border-radius:8px; }
        """)

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(10)

        # Tiêu đề
        self._lbl_title = QLabel("Tóm tắt Phiên Đo")
        self._lbl_title.setObjectName("title")
        root.addWidget(self._lbl_title)

        # Legend màu
        leg = QHBoxLayout()
        leg.setSpacing(14)
        for color, txt in [(_COLOR_FIRST, "  5 mẫu ĐẦU (Start-up)  "),
                            (_COLOR_LAST,  "  5 mẫu CUỐI (Braking)  ")]:
            lb = QLabel(txt)
            lb.setStyleSheet(f"background:{color.name()};padding:3px 10px;"
                             "border-radius:4px;font-size:11px;font-weight:bold;")
            leg.addWidget(lb)
        leg.addStretch()
        root.addLayout(leg)

        # Bảng dữ liệu
        lbl_t = QLabel("Dữ liệu các mẫu:")
        lbl_t.setObjectName("section")
        root.addWidget(lbl_t)

        self._table = QTableWidget()
        self._table.setColumnCount(len(_COL_HEADERS))
        self._table.setHorizontalHeaderLabels(_COL_HEADERS)
        hh = self._table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.Stretch)
        hh.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self._table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._table.verticalHeader().setVisible(False)
        root.addWidget(self._table, stretch=1)

        # Thống kê
        lbl_s = QLabel("Thống kê phiên đo:")
        lbl_s.setObjectName("section")
        root.addWidget(lbl_s)

        stat_row = QHBoxLayout()
        stat_row.setSpacing(8)
        self._sc_packets  = self._stat_card("Tổng gói tin",              "—")
        self._sc_duration = self._stat_card("Thời lượng",                "—")
        self._sc_e0       = self._stat_card("Max Err Hip (°)",            "—")
        self._sc_e1       = self._stat_card("Max Err Knee (°)",           "—")
        self._sc_e2       = self._stat_card("Max Err Ankle (°)",          "—")
        self._sc_t0       = self._stat_card("Torque Hip  Min|Max (Nm)",   "—")
        self._sc_t1       = self._stat_card("Torque Knee  Min|Max (Nm)",  "—")
        self._sc_t2       = self._stat_card("Torque Ankle  Min|Max (Nm)", "—")
        for c in [self._sc_packets, self._sc_duration,
                  self._sc_e0, self._sc_e1, self._sc_e2,
                  self._sc_t0, self._sc_t1, self._sc_t2]:
            stat_row.addWidget(c)
        root.addLayout(stat_row)

        # Nút
        btn_row = QHBoxLayout()
        btn_row.addStretch()

        self._btn_export = QPushButton("  Xuất CSV")
        self._btn_export.setObjectName("btn_export")
        self._btn_export.setFixedHeight(36)
        self._btn_export.clicked.connect(self._export_csv)
        btn_row.addWidget(self._btn_export)

        self._btn_close = QPushButton("  Đóng")
        self._btn_close.setObjectName("btn_close")
        self._btn_close.setFixedHeight(36)
        self._btn_close.clicked.connect(self.accept)
        btn_row.addWidget(self._btn_close)

        root.addLayout(btn_row)

    def _stat_card(self, label: str, value: str) -> QFrame:
        card = QFrame()
        card.setObjectName("stat_card")
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        lay = QVBoxLayout(card)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(2)
        lb = QLabel(label)
        lb.setStyleSheet("color:#607D8B;font-size:10px;")
        lb.setAlignment(Qt.AlignCenter)
        vl = QLabel(value)
        vl.setStyleSheet("color:#1A237E;font-size:13px;font-weight:bold;")
        vl.setAlignment(Qt.AlignCenter)
        lay.addWidget(lb)
        lay.addWidget(vl)
        card._val = vl          # type: ignore[attr-defined]
        return card

    # ─────────────────────────────────────────────────────────────────────
    def _populate(self):
        t = self._tracker
        if not t.has_data():
            # Vẫn hiện popup đầy đủ, nhưng bảng chỉ có 1 dòng thông báo
            self._lbl_title.setText("Tóm tắt Phiên Đo  —  Chưa nhận được dữ liệu từ ESP32")
            self._btn_export.setEnabled(False)
            self._table.setRowCount(1)
            msg = QTableWidgetItem(
                "⚠️  Chưa nhận được gói tin nào từ ESP32 trong phiên đo này. "
                "Kiểm tra kết nối UART hoặc bấm Run trước khi xem đồ thị."
            )
            msg.setTextAlignment(Qt.AlignCenter)
            msg.setFlags(Qt.ItemIsEnabled)
            msg.setBackground(QColor("#FFF9C4"))   # Vàng nhạt nổi bật
            self._table.setItem(0, 0, msg)
            self._table.setSpan(0, 0, 1, len(_COL_HEADERS))
            self._table.setRowHeight(0, 52)
            # Thẻ stat hiện 0
            self._sc_packets._val.setText("0")
            self._sc_duration._val.setText("0.0 s")
            for sc in [self._sc_e0, self._sc_e1, self._sc_e2,
                        self._sc_t0, self._sc_t1, self._sc_t2]:
                sc._val.setText("—")
            return

        rows = []
        for idx, s in enumerate(t.first_5):
            for j in range(3):
                rows.append(("first", idx + 1, j, s))

        has_sep = t.total_packets > 5 and len(t.last_5) > 0
        last_list = list(t.last_5)
        base = max(t.total_packets - len(last_list) + 1, 1)
        for idx, s in enumerate(last_list):
            for j in range(3):
                rows.append(("last", base + idx, j, s))

        n_rows = len(rows) + (1 if has_sep else 0)
        self._table.setRowCount(n_rows)

        sep_done = False
        tr = 0
        for group, sno, j, s in rows:
            if has_sep and not sep_done and group == "last":
                self._sep_row(tr)
                tr += 1
                sep_done = True

            bg = _COLOR_FIRST if group == "first" else _COLOR_LAST
            err = abs(s.pos_set[j] - s.pos_actual[j])
            cells = [
                str(sno), _JOINT_NAMES[j],
                f"{s.pos_actual[j]:.2f}", f"{s.pos_set[j]:.2f}", f"{err:.2f}",
                f"{s.vel_actual[j]:.2f}", f"{s.vel_set[j]:.2f}",
                f"{s.acc_actual[j]:.2f}", f"{s.acc_set[j]:.2f}",
                f"{s.torque[j]:.3f}",
            ]
            for col, txt in enumerate(cells):
                item = QTableWidgetItem(txt)
                item.setTextAlignment(Qt.AlignCenter)
                item.setBackground(bg)
                self._table.setItem(tr, col, item)
            tr += 1

        # Cập nhật thẻ thống kê
        self._sc_packets._val.setText(str(t.total_packets))
        self._sc_duration._val.setText(f"{t.duration_sec:.1f} s")
        self._sc_e0._val.setText(f"{t.max_tracking_error[0]:.2f} °")
        self._sc_e1._val.setText(f"{t.max_tracking_error[1]:.2f} °")
        self._sc_e2._val.setText(f"{t.max_tracking_error[2]:.2f} °")
        self._sc_t0._val.setText(f"{t.min_torque[0]:.3f}  |  {t.max_torque[0]:.3f}")
        self._sc_t1._val.setText(f"{t.min_torque[1]:.3f}  |  {t.max_torque[1]:.3f}")
        self._sc_t2._val.setText(f"{t.min_torque[2]:.3f}  |  {t.max_torque[2]:.3f}")

        self._lbl_title.setText(
            f"Tóm tắt Phiên Đo  —  {t.total_packets} gói  |  {t.duration_sec:.1f} s"
        )
        self._all_rows = rows

    def _sep_row(self, tr: int):
        mid = len(_COL_HEADERS) // 2
        for col in range(len(_COL_HEADERS)):
            item = QTableWidgetItem(". . ." if col == mid else "")
            item.setBackground(QColor("#B0BEC5"))
            item.setTextAlignment(Qt.AlignCenter)
            item.setFlags(Qt.ItemIsEnabled)
            self._table.setItem(tr, col, item)
        self._table.setRowHeight(tr, 12)

    # ─────────────────────────────────────────────────────────────────────
    def _export_csv(self):
        name = f"session_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        path, _ = QFileDialog.getSaveFileName(self, "Lưu CSV", name, "CSV (*.csv)")
        if not path:
            return
        t = self._tracker
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["Nhom","Mau #","Khop",
                        "Pos_Act","Pos_Set","Err_Pos",
                        "Vel_Act","Vel_Set","Acc_Act","Acc_Set","Torque"])
            for group, sno, j, s in self._all_rows:
                err = abs(s.pos_set[j] - s.pos_actual[j])
                w.writerow([
                    "DAU" if group == "first" else "CUOI", sno, _JOINT_NAMES[j],
                    f"{s.pos_actual[j]:.4f}", f"{s.pos_set[j]:.4f}", f"{err:.4f}",
                    f"{s.vel_actual[j]:.4f}", f"{s.vel_set[j]:.4f}",
                    f"{s.acc_actual[j]:.4f}", f"{s.acc_set[j]:.4f}",
                    f"{s.torque[j]:.6f}",
                ])
            w.writerow([])
            w.writerow(["=== THONG KE ==="])
            w.writerow(["Tong goi tin", t.total_packets])
            w.writerow(["Thoi luong (s)", f"{t.duration_sec:.2f}"])
            for i, nm in enumerate(_JOINT_NAMES):
                w.writerow([f"Max_Err_{nm} (deg)", f"{t.max_tracking_error[i]:.4f}"])
            for i, nm in enumerate(_JOINT_NAMES):
                w.writerow([f"Max_Torque_{nm} (Nm)", f"{t.max_torque[i]:.6f}"])
