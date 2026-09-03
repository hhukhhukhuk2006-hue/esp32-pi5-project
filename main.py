"""
==========================================================================
  MAIN.PY - File khởi động hệ thống Robot Phục hồi chức năng
==========================================================================
  Kiến trúc:
    Pi5 (GUI + Python) <-> UART <-> ESP32 <-> CAN <-> ODrive (3 khớp)

  Luồng hoạt động:
    1. Khởi tạo ESP32UARTManager (thread nền tự động giải mã dữ liệu)
    2. Mở giao diện GUI (LoginScreen -> DevMainScreen / DoctorMainScreen / PatientMainScreen)
    3. Ghép nối UART với GUI:
       - Nhận: ESP32 gửi RobotTelemetry (3 khớp: Pos/Vel/Acc actual + setpoint)
               -> Bơm thẳng lên JointPlotWidget (9 đồ thị realtime)
       - Gửi:  Bấm nút GUI (Bắt đầu / Dừng / Reset / Đặt góc)
               -> Gọi uart_mgr.run_exercise() / stop_exercise() / reset() ...

  Chạy:
    python main.py                      # Tự động phát hiện cổng UART
    python main.py --port /dev/ttyUSB0  # Chỉ định cổng UART cụ thể
    python main.py --no-uart            # Chạy GUI không cần ESP32 (demo/debug)
==========================================================================
"""

import os
import sys
import argparse
import logging

# ── Thêm thư mục scripts vào sys.path để import từ giaodienphuchoi ──────────
_ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS_DIR = os.path.join(_ROOT_DIR, "giaodienphuchoi", "scripts")
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

# ── Import thư viện Qt ───────────────────────────────────────────────────────
from PyQt5.QtWidgets import QApplication

# ── Import UART handler (giao tiếp ESP32) ────────────────────────────────────
from uart_handler import (
    ESP32UARTManager, RobotTelemetry,
    JointID,
)

# ── Import màn hình đăng nhập GUI ────────────────────────────────────────────
from login import LoginScreen

# ── Cấu hình logging ─────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] (%(name)s) %(message)s"
)
logger = logging.getLogger("MAIN")


# =============================================================================
#  UartGUIBridge — Cầu nối UART <-> GUI
# =============================================================================
class UartGUIBridge:
    """
    Kết nối ESP32UARTManager với DevMainScreen.

    Nhận: RobotTelemetry 3 khớp từ UART thread -> push vào JointPlotWidget
    Gửi: Các lệnh điều khiển từ GUI -> uart_handler -> ESP32
    """

    def __init__(self, uart_manager: ESP32UARTManager):
        self.uart = uart_manager
        self._gui_screen = None  # DevMainScreen instance
        self._dbg_count = 0
        
        # --- THÊM GIẢ LẬP ĐỒ THỊ SIN ---
        self._sim_running = False
        self._sim_thread = None
        # -------------------------------

        # Đăng ký callback: khi uart_handler nhận được gói tin telemetry, nó sẽ gọi hàm này.
        self.uart.register_telemetry_callback(self._on_telemetry)

    def attach_screen(self, screen):
        """Lưu tham chiếu đến DevMainScreen để có thể gọi push_data() vào đồ thị."""
        self._gui_screen = screen
        logger.info("[Bridge] Đã gắn DevMainScreen vào UartGUIBridge.")
        
    def _run_simulator(self):
        import math
        import time
        from uart_handler import RobotTelemetry
        t0 = time.time()
        while self._sim_running:
            t = time.time() - t0
            # Tạo 3 sóng sin lệch pha nhau
            val_hip = math.sin(t * 2) * 50
            val_knee = math.sin(t * 2 + 1) * 50
            val_ankle = math.sin(t * 2 + 2) * 50
            
            mock_data = RobotTelemetry()
            mock_data.joints[0].actual_pos = val_hip
            mock_data.joints[1].actual_pos = val_knee
            mock_data.joints[2].actual_pos = val_ankle
            mock_data.joints[0].set_pos = 50.0
            mock_data.joints[1].set_pos = 50.0
            mock_data.joints[2].set_pos = 50.0
            
            self._on_telemetry(mock_data)
            time.sleep(0.02) # 50Hz

    def detach_screen(self):
        """Gỡ kết nối khi logout."""
        self._gui_screen = None

    # ── Nhận telemetry từ ESP32 -> đẩy lên đồ thị ───────────────────────────
    def _on_telemetry(self, data: RobotTelemetry):
        """
        Callback gọi từ UART background thread — KHÔNG vẽ Qt trực tiếp.
        Chỉ push vào deque của JointPlotWidget, QTimer trong widget tự vẽ.
        Index: joints[0]=Hip, joints[1]=Knee, joints[2]=Ankle
        """
        # Debug counter (in mỗi 100 packet để không spam)
        self._dbg_count = getattr(self, '_dbg_count', 0) + 1
        if self._dbg_count <= 3 or self._dbg_count % 100 == 0:
            j = data.joints
            logger.info(
                f"[Bridge] Telemetry #{self._dbg_count}: "
                f"screen={'OK' if self._gui_screen else 'None'} | "
                f"Hip={j[0].actual_pos:.2f} Knee={j[1].actual_pos:.2f} Ankle={j[2].actual_pos:.2f}"
            )

        screen = self._gui_screen
        if screen is None:
            return
        pw = getattr(screen, '_joint_plot_widget', None)
        if pw is None:
            if self._dbg_count <= 3:
                logger.warning("[Bridge] _joint_plot_widget chua ton tai tren screen!")
            return
        try:
            j = data.joints
            pw.push_data(
                pos_actual=[j[0].actual_pos, j[1].actual_pos, j[2].actual_pos],
                vel_actual=[j[0].actual_vel, j[1].actual_vel, j[2].actual_vel],
                acc_actual=[j[0].actual_acc, j[1].actual_acc, j[2].actual_acc],
                pos_set   =[j[0].set_pos,    j[1].set_pos,    j[2].set_pos],
                vel_set   =[j[0].set_vel,    j[1].set_vel,    j[2].set_vel],
                acc_set   =[j[0].set_acc,    j[1].set_acc,    j[2].set_acc],
            )
            if self._dbg_count <= 3:
                logger.info(f"[Bridge] push_data OK - plot_widget visible={pw.isVisible()}")
        except Exception as e:
            logger.warning(f"[Bridge] Loi day du lieu: {e}")

    # ── Gửi lệnh xuống ESP32 (GUI gọi khi bấm nút) ──────────────────────────
    def send_run(self) -> bool:
        """BẮT ĐẦU bài tập."""
        import threading
        # Bật giả lập nếu chưa bật
        if not self._sim_running:
            self._sim_running = True
            self._sim_thread = threading.Thread(target=self._run_simulator, daemon=True)
            self._sim_thread.start()
            logger.info("[Bridge] BẮT ĐẦU GIẢ LẬP ĐỒ THỊ SIN")
        
        return self.uart.run_exercise()

    def send_stop(self) -> bool:
        """DỪNG bài tập."""
        self._sim_running = False
        logger.info("[Bridge] DỪNG GIẢ LẬP ĐỒ THỊ SIN")
        return self.uart.stop_exercise()

    def send_reset(self) -> bool:
        """RESET / về IDLE."""
        self._sim_running = False
        return self.uart.reset()

    def send_joint_target(self, joint: str, angle_deg: float) -> bool:
        """Đặt góc mục tiêu — joint: 'hip' / 'knee' / 'ankle'."""
        jmap = {'hip': JointID.HIP, 'knee': JointID.KNEE, 'ankle': JointID.ANKLE}
        jid = jmap.get(joint.lower())
        if jid is None:
            logger.warning(f"[Bridge] Tên khớp không hợp lệ: {joint}")
            return False
        logger.info(f"[Bridge] SET_JOINT_TARGET {joint.upper()} = {angle_deg:.2f}°")
        return self.uart.send_joint_target(jid, angle_deg)

    def send_load(self, weight_kg: float, height_m: float, l1_m: float, l2_m: float) -> bool:
        """Cập nhật thông số nhân trắc học người tập."""
        return self.uart.send_load(weight_kg, height_m, l1_m, l2_m)

    def send_control_coef(self, joint: str, kp: float, kd: float) -> bool:
        """Cập nhật hệ số PD cho khớp."""
        jmap = {'hip': JointID.HIP, 'knee': JointID.KNEE, 'ankle': JointID.ANKLE}
        jid = jmap.get(joint.lower())
        if jid is None:
            return False
        return self.uart.send_control_coef(jid, kp, kd)

    def send_homing(self) -> bool:
        return self.uart.homing(JointID.ALL)

    def send_offset_encoder(self) -> bool:
        return self.uart.offset_encoder(JointID.ALL)

    @property
    def is_connected(self) -> bool:
        return self.uart.is_connected()

    @property
    def packets_received(self) -> int:
        return self.uart.packets_received

    @property
    def packets_crc_error(self) -> int:
        return self.uart.packets_crc_error


# =============================================================================
#  Hook LoginScreen: tự động attach Bridge khi login thành công
# =============================================================================
def _patch_login_screen(login_win: LoginScreen, bridge: UartGUIBridge):
    """
    Monkey-patch check_ID để gắn bridge vào DevMainScreen ngay sau login.
    """
    original_check_ID = login_win.check_ID

    def patched_check_ID():
        original_check_ID()
        screen = getattr(login_win, 'mainscreen', None)
        if screen is not None:
            bridge.attach_screen(screen)
            screen.uart_bridge = bridge   # Để screen gọi bridge.send_run() v.v.
            logger.info(f"[Main] Bridge gắn vào {type(screen).__name__} thành công.")
            # Tự động hiện đồ thị realtime ngay sau khi login + kết nối UART
            pw = getattr(screen, '_joint_plot_widget', None)
            btn = getattr(screen, '_btn_toggle_plot', None)
            if pw is not None:
                try:
                    pw.show()
                    if btn is not None:
                        btn.setChecked(True)
                        btn.setText("⬅ Điều khiển")
                    # Ẩn frame exercises để nhường chỗ cho đồ thị
                    if hasattr(screen, 'ui') and hasattr(screen.ui, 'frame_exercises'):
                        screen.ui.frame_exercises.hide()
                    screen._plot_view_active = True
                    logger.info("[Main] Đồ thị realtime đã được bật tự động.")
                except Exception as e:
                    logger.warning(f"[Main] Không thể auto-show plot: {e}")

    login_win.check_ID = patched_check_ID


# =============================================================================
#  MAIN
# =============================================================================
def main():
    # ── Parse tham số dòng lệnh ──────────────────────────────────────────────
    parser = argparse.ArgumentParser(
        description="Robot Phục hồi Chức năng — Pi5 <-> UART <-> ESP32 <-> ODrive"
    )
    parser.add_argument("--port", type=str, default=None,
                        help="Cổng UART (vd: /dev/ttyUSB0 hoặc COM3). Tự phát hiện nếu bỏ qua.")
    parser.add_argument("--baud", type=int, default=921600,
                        help="Tốc độ baud UART (mặc định: 921600).")
    parser.add_argument("--no-uart", action="store_true",
                        help="Chạy GUI không cần ESP32 (chế độ demo/debug).")
    args = parser.parse_args()

    print("=" * 62)
    print("   HE THONG DIEU KHIEN ROBOT PHUC HOI CHUC NANG")
    print("   Pi5 <-> UART <-> ESP32 <-> CAN <-> ODrive (3 khop)")
    print("=" * 62)

    # ── 1. Khởi tạo Qt Application ───────────────────────────────────────────
    qt_app = QApplication(sys.argv)

    # ── 2. Khởi tạo UART + Bridge ────────────────────────────────────────────
    uart_mgr = None
    bridge = None

    if not args.no_uart:
        logger.info("[1/3] Khởi tạo ESP32 UART Manager...")
        uart_mgr = ESP32UARTManager(baudrate=args.baud)
        port = args.port or uart_mgr.auto_detect_port()
        logger.info(f"  -> Thử kết nối: {port} @ {args.baud} baud")

        if uart_mgr.connect(port):
            logger.info(f"  -> Kết nối UART thành công ({port})!")
            bridge = UartGUIBridge(uart_mgr)
        else:
            logger.warning(f"  -> Không kết nối được {port}. Chuyển sang chế độ giả lập.")
            uart_mgr = None
            # Hiện QMessageBox cảnh báo trước khi vào GUI
            from PyQt5.QtWidgets import QMessageBox
            mb = QMessageBox()
            mb.setWindowTitle("⚠️  Không tìm thấy ESP32")
            mb.setIcon(QMessageBox.Warning)
            mb.setText(
                f"<b>Không thể kết nối tới ESP32</b><br>"
                f"Cổng UART <code>{port}</code> không phản hồi."
            )
            mb.setInformativeText(
                "Hệ thống sẽ khởi động ở <b>chế độ giả lập (Demo)</b>.<br>"
                "Dữ liệu thực tế từ cảm biến sẽ <u>không được cập nhật</u>.<br><br>"
                "Kiểm tra lại:<br>"
                "&nbsp;&nbsp;• Cáp USB/UART đã cắm chưa?<br>"
                f"&nbsp;&nbsp;• Đúng cổng <code>{port}</code> chưa?<br>"
                "&nbsp;&nbsp;• ESP32 đã được nạp firmware và đang chạy chưa?"
            )
            mb.setStandardButtons(QMessageBox.Ok)
            mb.setDefaultButton(QMessageBox.Ok)
            mb.exec()
    else:
        logger.info("[1/3] Chế độ --no-uart: Bỏ qua kết nối ESP32.")

    # ── 3. Mở giao diện đăng nhập ────────────────────────────────────────────
    logger.info("[2/3] Khởi tạo GUI...")
    login_win = LoginScreen()

    # ── 4. Gắn Bridge vào GUI ────────────────────────────────────────────────
    if bridge is not None:
        logger.info("[3/3] Gắn UartGUIBridge...")
        _patch_login_screen(login_win, bridge)
        logger.info("  -> Bridge sẽ kích hoạt sau khi đăng nhập thành công.")
    else:
        logger.info("[3/3] Chế độ Giả lập (Demo) — ESP32 UART không hoạt động.")

    # ── 5. Hiện cửa sổ ───────────────────────────────────────────────────────
    login_win.showMaximized()
    logger.info("GUI sẵn sàng. Đăng nhập để bắt đầu.")
    if uart_mgr:
        logger.info(f"UART: {uart_mgr.port} @ {uart_mgr.baudrate} baud — Lắng nghe dữ liệu ESP32...")

    exit_code = qt_app.exec()

    # ── Dọn dẹp khi thoát ────────────────────────────────────────────────────
    if uart_mgr is not None:
        logger.info("Ngắt kết nối UART...")
        uart_mgr.disconnect()
    logger.info("Hệ thống đã dừng.")
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
