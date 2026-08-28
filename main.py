"""
==========================================================================
  MAIN.PY - File khởi động tổng hợp cho hệ thống Robot phục hồi chức năng
==========================================================================
  Kiến trúc:
    1. uart_handler.py   → Giao tiếp UART với ESP32
    2. Trajectory_controller.py (ODriveThread) → Điều khiển quỹ đạo + ODrive
    3. Control_GUI_Basic.py (ControlGUI)       → Giao diện điều khiển

  File này đóng vai trò "Giám đốc" điều phối cả 3 module làm việc cùng nhau.
  Chạy: python main.py
==========================================================================
"""

import sys
import time
import logging
import argparse #cấu hình tham số truyền dữ liệu vào.

# --- Import 3 module chính ---
from uart_handler import ESP32UARTManager, RobotTelemetry
from Trajectory_controller import ODriveThread
from Control_GUI_Basic import ControlGUI

# Cấu hình logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] (%(name)s) %(message)s"
)
logger = logging.getLogger("MAIN")


def main():
    # ======================================================================
    # 0. PARSE THAM SỐ DÒNG LỆNH (tuỳ chọn)
    # ======================================================================
    parser = argparse.ArgumentParser(description="Robot Rehabilitation Controller")
    parser.add_argument(
        "--port", type=str, default=None,
        help="Cổng UART kết nối ESP32 Mặc định: tự phát hiện."
    )
    parser.add_argument(
        "--no-uart", action="store_true",
        help="Chạy không cần ESP32 (chỉ dùng ODrive trực tiếp như cũ)"
    )
    args = parser.parse_args()

    print("=" * 60)
    print("   HE THONG DIEU KHIEN ROBOT PHUC HOI CHUC NANG")
    print("   Pi5 (Trajectory + GUI) <-> ESP32 (UART) <-> ODrive")
    print("=" * 60)

    # ======================================================================
    # 1. KHỞI TẠO UART MANAGER (Culi bốc vác dữ liệu)
    # ======================================================================
    uart_mgr = None

    if not args.no_uart:
        logger.info("[1/3] Khoi tao ESP32 UART Manager...")
        uart_mgr = ESP32UARTManager()

        port = args.port if args.port else uart_mgr.auto_detect_port()
        logger.info(f"  -> Cong UART: {port}")

        success = uart_mgr.connect(port)
        if success:
            logger.info(f"  -> Ket noi UART thanh cong ({port})!")
        else:
            logger.warning(f"  -> Khong the ket noi ESP32 tren {port}.")
            logger.warning("  -> Tiep tuc chay KHONG co ESP32 (chi dung ODrive truc tiep).")
            uart_mgr = None
    else:
        logger.info("[1/3] Chay che do KHONG co ESP32 (--no-uart)")

    # ======================================================================
    # 2. KHỞI TẠO TRAJECTORY CONTROLLER (Kỹ sư điều khiển)
    #    Giữ nguyên ODriveThread hoạt động như cũ.
    #    Sau này khi refactor, sẽ thay ODrive USB bằng gửi lệnh qua UART.
    # ======================================================================
    logger.info("[2/3] Khoi tao Trajectory Controller (ODriveThread)...")

    # Lưu ý: ControlGUI hiện tại tự tạo ODriveThread bên trong __init__.
    # Nên ở đây ta chỉ cần tạo GUI, nó sẽ tự tạo controller.
    # Sau khi refactor, ta sẽ tạo controller ở đây rồi inject vào GUI.

    # ======================================================================
    # 3. KHỞI TẠO GUI VÀ INJECT UART (Lễ tân giao tiếp)
    # ======================================================================
    logger.info("[3/3] Khoi tao Control GUI...")
    app = ControlGUI()

    # --- INJECT: Gắn uart_mgr vào controller và GUI để dùng chung ---
    if uart_mgr is not None:
        # Gắn UART vào controller (ODriveThread) bên trong GUI
        app.controller.uart = uart_mgr
        # Gắn UART vào GUI để sau này GUI có thể gọi lệnh UART trực tiếp
        app.uart = uart_mgr

        # Đăng ký callback: Mỗi khi ESP32 gửi telemetry lên,
        # in ra terminal để debug (sau này sẽ đưa lên GUI)
        def on_esp32_telemetry(data: RobotTelemetry):
            j0 = data.joints[0]
            logger.debug(
                f"[ESP32] HIP: pos={j0.actual_pos:.2f} vel={j0.actual_vel:.2f} "
                f"tor={j0.torque_set:.2f}"
            )

        uart_mgr.register_telemetry_callback(on_esp32_telemetry)
        logger.info("  -> Da gan UART Manager vao Controller va GUI.")
    else:
        app.uart = None
        logger.info("  -> Chay KHONG co UART (ODrive truc tiep qua USB).")

    # ======================================================================
    # 4. CHẠY GIAO DIỆN (Main Loop - chặn ở đây cho đến khi đóng app)
    # ======================================================================
    logger.info("=" * 50)
    logger.info("  HE THONG SAN SANG! Dang mo giao dien...")
    logger.info("=" * 50)

    try:
        app.mainloop()
    except KeyboardInterrupt:
        logger.info("Nhan Ctrl+C, dang tat he thong...")

    # ======================================================================
    # 5. DỌN DẸP KHI ĐÓNG APP
    # ======================================================================
    logger.info("Dang tat he thong...")

    # Dừng controller (ODriveThread)
    try:
        if hasattr(app, "controller"):
            app.controller.stop()
            app.controller.join(timeout=2.0)
            logger.info("  -> Controller da dung.")
    except Exception as e:
        logger.error(f"  -> Loi khi dung controller: {e}")

    # Ngắt kết nối UART
    if uart_mgr is not None:
        try:
            uart_mgr.disconnect()
            logger.info("  -> UART da ngat ket noi.")
        except Exception as e:
            logger.error(f"  -> Loi khi ngat UART: {e}")

    logger.info("Hoan tat. Tam biet!")


if __name__ == "__main__":
    main()
