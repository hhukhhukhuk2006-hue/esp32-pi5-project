"""Entry point — Giao diện phục hồi chức năng (PyQt5).

Adapted từ LLRR_app/scripts/GUI.py. Khởi động LoginScreen, tự động
add thư mục gốc dự án (phcn/) vào sys.path nếu cần import các module khác.
"""

import os
import sys

# Đảm bảo `from backend.frontend_adapter import ...` chạy được khi user chạy
# `python GUI.py` từ giaodienphuchoi/scripts/. `scripts/` chính là package root.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from PyQt5.QtWidgets import QApplication
from login import LoginScreen


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = LoginScreen()
    win.showMaximized()
    sys.exit(app.exec())