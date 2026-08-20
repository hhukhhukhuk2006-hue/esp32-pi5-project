"""
Module giao tiếp UART giữa Raspberry Pi 5 và ESP32.
Tương thích 100% với firmware ESP32 (uart_driver.c, struct_define.h, enum_define.h).
"""

import sys
import time
import struct
import logging
import threading
from enum import IntEnum
from dataclasses import dataclass, field
from typing import Callable, Optional, List, Dict

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

# Cấu hình logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] (%(name)s) %(message)s"
)
logger = logging.getLogger("ESP32_UART")

# ==============================================================================
# ĐỊNH NGHĨA HẰNG SỐ & GIAO THỨC (ĐỒNG BỘ VỚI ESP32)
# ==============================================================================
DOF = 3
UART_FRAME_SOF = 0x55
UART_FRAME_EOF = 0xAA
DEFAULT_BAUDRATE = 921600

class JointID(IntEnum):
    ALL = 0
    HIP = 1
    KNEE = 2
    ANKLE = 3

class ExerciseType(IntEnum):
    IDLE = 0
    SWINGING = 1
    WALKING = 2
    CYCLING = 3

class ControlMode(IntEnum):
    IDLE = 0
    BRAKE = 1
    ANTI_GRAVITY = 2
    CLOSE_LOOP = 3

class UartRxCmd(IntEnum):
    """Lệnh từ Pi5 gửi xuống ESP32 (khớp với uart_rx_cmd_e trong C)"""
    UPDATE_JOINT_LIMIT = 0
    UPDATE_LOAD = 1
    UPDATE_CONTROL_COEF = 2
    SET_JOINT_TARGET = 3
    SET_END_POINT_TARGET = 4
    SET_EXCERCISE = 5
    RUN_EXCERCISE = 6
    STOP_EXCERCISE = 7
    RESET = 8
    OFFSET_ENCODER = 9
    HOMING = 10

class UartTxCmd(IntEnum):
    """Lệnh từ ESP32 gửi lên Pi5 (khớp với uart_tx_cmd_e trong C)"""
    UPDATE_STATE = 0
    UPDATE_ERROR = 1

class ErrorCode(IntEnum):
    TWAI_ERROR_INIT_FAIL = 0
    TWAI_ERROR_SEND_FAIL = 1
    TWAI_ERROR_RECV_FAIL = 2
    SERIAL_ERROR_INIT_FAIL = 3
    SERIAL_ERROR_SEND_FAIL = 4
    SERIAL_ERROR_RECV_FAIL = 5
    ODRIVE_ERROR_ENCODER = 6
    ODRIVE_ERROR_MOTOR = 7
    ODRIVE_ERROR_CONTROLLER = 8
    ODRIVE_ERROR_HEART_BEAT_DROP = 9


# ==============================================================================
# CẤU TRÚC DỮ LIỆU TELEMETRY
# ==============================================================================
@dataclass
class JointTelemetry:
    """Dữ liệu trạng thái thực tế và giá trị đặt của 1 khớp (32 bytes)"""
    # Trạng thái thực tế (Actual state)
    actual_pos: float = 0.0
    actual_vel: float = 0.0
    actual_acc: float = 0.0
    actual_jerk: float = 0.0
    # Giá trị đặt mục tiêu (Set points)
    set_pos: float = 0.0
    set_vel: float = 0.0
    set_acc: float = 0.0
    # Mô-men xoắn cài đặt
    torque_set: float = 0.0

@dataclass
class RobotTelemetry:
    """Tổng hợp dữ liệu toàn bộ robot (DOF khớp)"""
    joints: List[JointTelemetry] = field(default_factory=lambda: [JointTelemetry() for _ in range(DOF)])
    last_error_code: int = 0
    last_updated_time: float = 0.0


# ==============================================================================
# LỚP XỬ LÝ GIAO TIẾP UART CHÍNH
# ==============================================================================
class ESP32UARTManager:
    """
    Quản lý luồng gửi/nhận UART với ESP32.
    - Chạy luồng nền (background thread) liên tục giải mã dữ liệu theo FSM.
    - Thread-safe, hỗ trợ callback khi có dữ liệu mới.
    """
    def __init__(self, port: Optional[str] = None, baudrate: int = DEFAULT_BAUDRATE, dof: int = DOF):
        self.port = port
        self.baudrate = baudrate
        self.dof = dof

        self.ser: Optional[serial.Serial] = None
        self.is_running = False
        self.rx_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        # Dữ liệu telemetry mới nhất
        self.telemetry = RobotTelemetry()

        # Callback thông báo dữ liệu mới
        self.on_telemetry_callbacks: List[Callable[[RobotTelemetry], None]] = []
        self.on_error_callbacks: List[Callable[[int], None]] = []

        # Thống kê gói tin
        self.packets_received = 0
        self.packets_crc_error = 0
        self.packets_sent = 0

    @staticmethod
    def auto_detect_port() -> str:
        """Tự động phát hiện cổng UART trên Raspberry Pi hoặc Windows/Linux"""
        if sys.platform.startswith("linux"):
            # Ưu tiên các cổng USB-UART gắn ngoài trước, sau đó mới đến cổng GPIO mặc định
            candidate_ports = [
                "/dev/ttyUSB0",
                "/dev/ttyUSB1",
                "/dev/ttyACM0",
                "/dev/ttyACM1",
                "/dev/serial0",
                "/dev/ttyAMA0"
            ]
            for p in candidate_ports:
                try:
                    import os
                    if os.path.exists(p):
                        return p
                except Exception:
                    pass
            return "/dev/ttyUSB0"
        else:
            # Trên Windows: tìm cổng COM có sẵn
            if serial:
                ports = list(serial.tools.list_ports.comports())
                if ports:
                    return ports[0].device
            return "COM3"

    def connect(self, port: Optional[str] = None) -> bool:
        """Mở kết nối cổng nối tiếp"""
        if serial is None:
            logger.error("Thư viện 'pyserial' chưa được cài đặt! Hãy chạy: pip install pyserial")
            return False

        if port is not None:
            self.port = port
        if self.port is None:
            self.port = self.auto_detect_port()

        try:
            logger.info(f"Đang kết nối tới ESP32 qua {self.port} @ {self.baudrate} baud...")
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            self.is_running = True
            self.rx_thread = threading.Thread(target=self._rx_worker, name="ESP32_UART_RX", daemon=True)
            self.rx_thread.start()
            logger.info(f"Kết nối UART thành công trên cổng {self.port}!")
            return True
        except Exception as e:
            logger.error(f"Lỗi kết nối UART tới {self.port}: {e}")
            self.ser = None
            return False

    def disconnect(self):
        """Đóng kết nối UART và dừng luồng đọc"""
        self.is_running = False
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        logger.info("Đã ngắt kết nối UART.")

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open and self.is_running

    # ==========================================================================
    # CÁC HÀM GỬI LỆNH XUỐNG ESP32 (PACKET ENCODING)
    # ==========================================================================
    def _build_frame(self, cmd: UartRxCmd, joint_id: int, payload: bytes = b"") -> bytes:
        """
        Đóng gói Frame theo đúng chuẩn của ESP32:
        [SOF (0x55)] [ID] [DataLen] [Payload (N bytes)] [CRC_L] [CRC_H] [EOF (0xAA)]
        Trong đó:
          ID = (cmd << 3) | (joint_id & 0x07)
          CRC = (ID + dataLen + payload[0] + payload[-1]) & 0xFFFF (nếu dataLen > 0)
                hoặc (ID + dataLen) & 0xFFFF (nếu dataLen == 0)
        """
        frame_id = ((int(cmd) & 0x1F) << 3) | (int(joint_id) & 0x07)
        data_len = len(payload)

        # Tính toán checksum khớp hoàn toàn với hàm uart_recv_task của ESP32
        if data_len > 0:
            crc_val = (frame_id + data_len + payload[0] + payload[-1]) & 0xFFFF
        else:
            crc_val = (frame_id + data_len) & 0xFFFF

        frame = bytearray()
        frame.append(UART_FRAME_SOF)
        frame.append(frame_id)
        frame.append(data_len)
        frame.extend(payload)
        frame.extend(struct.pack("<H", crc_val))  # 2 bytes CRC little-endian
        frame.append(UART_FRAME_EOF)
        return bytes(frame)

    def _send_raw_frame(self, frame: bytes) -> bool:
        """Gửi chuỗi byte qua UART với lock bảo vệ"""
        if not self.is_connected():
            logger.warning("Không thể gửi dữ liệu: Chưa kết nối UART!")
            return False
        try:
            with self.lock:
                self.ser.write(frame)
                self.ser.flush()
                self.packets_sent += 1
            return True
        except Exception as e:
            logger.error(f"Lỗi khi gửi dữ liệu UART: {e}")
            return False

    def send_joint_limit(self, joint_id: JointID, max_v: float, max_a: float, max_j: float) -> bool:
        """
        Cập nhật giới hạn động học cho khớp: Max velocity, Max acceleration, Max jerk.
        (UART_CMD_UPDATE_JOINT_LIMIT = 0, jointID: 1..DOF, dataLen = 12 bytes)
        """
        payload = struct.pack("<fff", float(max_v), float(max_a), float(max_j))
        frame = self._build_frame(UartRxCmd.UPDATE_JOINT_LIMIT, joint_id, payload)
        logger.info(f"Gửi giới hạn khớp {joint_id.name}: MaxV={max_v}, MaxA={max_a}, MaxJ={max_j}")
        return self._send_raw_frame(frame)

    def send_load(self, weight: float, height: float, l1: float, l2: float) -> bool:
        """
        Cập nhật thông số tải trọng / cơ thể người tập.
        (UART_CMD_UPDATE_LOAD = 1, dataLen = 16 bytes)
        """
        payload = struct.pack("<ffff", float(weight), float(height), float(l1), float(l2))
        frame = self._build_frame(UartRxCmd.UPDATE_LOAD, JointID.ALL, payload)
        logger.info(f"Gửi thông số tải trọng: Cân nặng={weight}kg, Chiều cao={height}m, L1={l1}m, L2={l2}m")
        return self._send_raw_frame(frame)

    def send_control_coef(self, joint_id: JointID, kp: float, kd: float) -> bool:
        """
        Cập nhật hệ số điều khiển Kp, Kd cho khớp.
        (UART_CMD_UPDATE_CONTROL_COEF = 2, jointID: 1..DOF, dataLen = 8 bytes)
        """
        payload = struct.pack("<ff", float(kp), float(kd))
        frame = self._build_frame(UartRxCmd.UPDATE_CONTROL_COEF, joint_id, payload)
        logger.info(f"Gửi hệ số điều khiển khớp {joint_id.name}: Kp={kp}, Kd={kd}")
        return self._send_raw_frame(frame)

    def send_joint_target(self, joint_id: JointID, target_pos: float) -> bool:
        """
        Đặt góc mục tiêu cho khớp.
        (UART_CMD_SET_JOINT_TARGET = 3, jointID: 1..DOF, dataLen = 4 bytes)
        """
        payload = struct.pack("<f", float(target_pos))
        frame = self._build_frame(UartRxCmd.SET_JOINT_TARGET, joint_id, payload)
        return self._send_raw_frame(frame)

    def send_endpoint_target(self, x: float, y: float, phi: float) -> bool:
        """
        Đặt vị trí điểm cuối mục tiêu (X, Y, Phi).
        (UART_CMD_SET_END_POINT_TARGET = 4, dataLen = 12 bytes)
        """
        payload = struct.pack("<fff", float(x), float(y), float(phi))
        frame = self._build_frame(UartRxCmd.SET_END_POINT_TARGET, JointID.ALL, payload)
        logger.info(f"Gửi tọa độ EndPoint: X={x}, Y={y}, Phi={phi}")
        return self._send_raw_frame(frame)

    def send_exercise_type(self, exercise_type: ExerciseType) -> bool:
        """
        Chọn bài tập phục hồi chức năng (IDLE, SWINGING, WALKING, CYCLING).
        (UART_CMD_SET_EXCERCISE = 5, dataLen = 1 byte)
        """
        payload = struct.pack("<B", int(exercise_type))
        frame = self._build_frame(UartRxCmd.SET_EXCERCISE, JointID.ALL, payload)
        logger.info(f"Chọn bài tập: {exercise_type.name}")
        return self._send_raw_frame(frame)

    def run_exercise(self) -> bool:
        """Bắt đầu chạy bài tập (UART_CMD_RUN_EXCERCISE = 6, dataLen = 0)"""
        frame = self._build_frame(UartRxCmd.RUN_EXCERCISE, JointID.ALL)
        logger.info("Gửi lệnh: BẮT ĐẦU BÀI TẬP (RUN_EXCERCISE)")
        return self._send_raw_frame(frame)

    def stop_exercise(self) -> bool:
        """Dừng bài tập (UART_CMD_STOP_EXCERCISE = 7, dataLen = 0)"""
        frame = self._build_frame(UartRxCmd.STOP_EXCERCISE, JointID.ALL)
        logger.info("Gửi lệnh: DỪNG BÀI TẬP (STOP_EXCERCISE)")
        return self._send_raw_frame(frame)

    def reset(self) -> bool:
        """Gửi lệnh Reset hệ thống (UART_CMD_RESET = 8)"""
        frame = self._build_frame(UartRxCmd.RESET, JointID.ALL)
        logger.info("Gửi lệnh: RESET HỆ THỐNG")
        return self._send_raw_frame(frame)

    def offset_encoder(self, joint_id: JointID = JointID.ALL) -> bool:
        """Gửi lệnh Offset Encoder (UART_CMD_OFFSET_ENCODER = 9)"""
        frame = self._build_frame(UartRxCmd.OFFSET_ENCODER, joint_id)
        logger.info(f"Gửi lệnh: OFFSET ENCODER cho khớp {joint_id.name}")
        return self._send_raw_frame(frame)

    def homing(self, joint_id: JointID = JointID.ALL) -> bool:
        """Gửi lệnh Homing cho khớp (UART_CMD_HOMING = 10)"""
        frame = self._build_frame(UartRxCmd.HOMING, joint_id)
        logger.info(f"Gửi lệnh: HOMING cho khớp {joint_id.name}")
        return self._send_raw_frame(frame)

    # ==========================================================================
    # LUỒNG NHẬN VÀ GIẢI MÃ DỮ LIỆU TỪ ESP32 (STATE MACHINE PARSER)
    # ==========================================================================
    def _rx_worker(self):
        """
        Luồng nền đọc byte-by-byte theo máy trạng thái:
        SOF -> ID -> DataLen -> Data -> CRC (2 bytes) -> EOF
        """
        rx_state = "WAIT_SOF"
        frame_id = 0
        data_len = 0
        payload = bytearray()
        crc_buff = bytearray()

        while self.is_running and self.ser and self.ser.is_open:
            try:
                byte_in = self.ser.read(1)
                if not byte_in:
                    continue
                rx_byte = byte_in[0]

                if rx_state == "WAIT_SOF":
                    if rx_byte == UART_FRAME_SOF:
                        rx_state = "READ_ID"

                elif rx_state == "READ_ID":
                    frame_id = rx_byte
                    rx_state = "READ_LEN"

                elif rx_state == "READ_LEN":
                    data_len = rx_byte
                    payload = bytearray()
                    crc_buff = bytearray()
                    if data_len == 0:
                        rx_state = "READ_CRC"
                    else:
                        rx_state = "READ_DATA"

                elif rx_state == "READ_DATA":
                    payload.append(rx_byte)
                    if len(payload) >= data_len:
                        rx_state = "READ_CRC"

                elif rx_state == "READ_CRC":
                    crc_buff.append(rx_byte)
                    if len(crc_buff) >= 2:
                        rx_state = "READ_EOF"

                elif rx_state == "READ_EOF":
                    if rx_byte == UART_FRAME_EOF:
                        # Kiểm tra checksum CRC
                        crc_recv = struct.unpack("<H", crc_buff)[0]
                        # ESP32 sendDataToPC: crcVal = ID + dataLen + frame[3] + frame[frameLen-4]
                        # frame[3] = payload[1], frame[frameLen-4] = payload[dataLen-1]
                        if data_len >= 2:
                            crc_calc = (frame_id + data_len + payload[0] + payload[data_len - 1]) & 0xFFFF
                        elif data_len == 1:
                            # frame[3] is CRC byte (not payload), frame[frameLen-4]=frame[2]=payload[0]
                            # ESP32 sendDataToPC with dataLen=1: frame[3]=CRC_L, frame[frameLen-4]=frame[2]=payload[0]
                            # This is an edge case unlikely to occur in practice (telemetry is always 96 bytes)
                            crc_calc = (frame_id + data_len + payload[0] + payload[0]) & 0xFFFF
                        else:
                            crc_calc = (frame_id + data_len) & 0xFFFF
                        crc_valid = (crc_recv == crc_calc)

                        if crc_valid:
                            self.packets_received += 1
                            self._parse_incoming_packet(frame_id, bytes(payload))
                        else:
                            self.packets_crc_error += 1
                            logger.warning(
                                f"Gói tin UART lỗi CRC! ID={frame_id}, Nhận={crc_recv}, Dự kiến={crc_calc}"
                            )
                    else:
                        logger.warning(f"Gói tin UART mất EOF (nhận được {hex(rx_byte)} thay vì {hex(UART_FRAME_EOF)})")
                    rx_state = "WAIT_SOF"

                    # Reset máy trạng thái cho gói tiếp theo
                    rx_state = "WAIT_SOF"
                    payload.clear()
                    crc_buff.clear()

            except serial.SerialException as e:
                if self.is_running:
                    logger.error(f"Lỗi cổng serial trong luồng RX: {e}")
                    time.sleep(0.5)
            except Exception as e:
                if self.is_running:
                    logger.error(f"Lỗi ngoại lệ trong luồng RX: {e}")

    def _parse_incoming_packet(self, frame_id: int, payload: bytes):
        """Phân tích nội dung payload sau khi qua kiểm tra CRC"""
        cmd = (frame_id >> 3) & 0x1F
        joint_id = frame_id & 0x07

        if cmd == UartTxCmd.UPDATE_STATE:
            # Gói tin cập nhật telemetry trạng thái các khớp (DOF * 8 * 4 bytes)
            # Mỗi khớp: actual_pos, actual_vel, actual_acc, actual_jerk, set_pos, set_vel, set_acc, torque_set
            expected_joint_len = 32  # 8 floats * 4 bytes
            for i in range(self.dof):
                offset = i * expected_joint_len
                if len(payload) >= offset + expected_joint_len:
                    chunk = payload[offset : offset + expected_joint_len]
                    pos, vel, acc, jerk, set_pos, set_vel, set_acc, tor = struct.unpack("<ffffffff", chunk)
                    with self.lock:
                        j = self.telemetry.joints[i]
                        j.actual_pos = pos
                        j.actual_vel = vel
                        j.actual_acc = acc
                        j.actual_jerk = jerk
                        j.set_pos = set_pos
                        j.set_vel = set_vel
                        j.set_acc = set_acc
                        j.torque_set = tor

            self.telemetry.last_updated_time = time.time()

            # Gọi các callback đã đăng ký để giao diện GUI vẽ đồ thị
            for cb in self.on_telemetry_callbacks:
                try:
                    cb(self.telemetry)
                except Exception as e:
                    logger.error(f"Lỗi trong callback telemetry: {e}")

        elif cmd == UartTxCmd.UPDATE_ERROR:
            # Gói tin thông báo lỗi hệ thống từ ESP32 (4 bytes uint32_t)
            if len(payload) >= 4:
                err_code = struct.unpack("<I", payload[:4])[0]
                with self.lock:
                    self.telemetry.last_error_code = err_code
                logger.error(f"ESP32 BÁO LỖI HỆ THỐNG: Mã lỗi = {hex(err_code)} ({err_code})")
                for cb in self.on_error_callbacks:
                    try:
                        cb(err_code)
                    except Exception as e:
                        logger.error(f"Lỗi trong callback error: {e}")

    # ==========================================================================
    # CÁC HÀM TIỆN ÍCH CHO GUI / CONTROLLER ĐỌC DỮ LIỆU
    # ==========================================================================
    def register_telemetry_callback(self, cb: Callable[[RobotTelemetry], None]):
        """Đăng ký hàm callback nhận telemetry theo thời gian thực (realtime)"""
        if cb not in self.on_telemetry_callbacks:
            self.on_telemetry_callbacks.append(cb)

    def get_joint_state(self, joint_idx: int = 0) -> JointTelemetry:
        """Lấy bản sao dữ liệu an toàn (thread-safe) của 1 khớp (0: HIP, 1: KNEE, 2: ANKLE)"""
        with self.lock:
            if 0 <= joint_idx < len(self.telemetry.joints):
                j = self.telemetry.joints[joint_idx]
                return JointTelemetry(
                    actual_pos=j.actual_pos,
                    actual_vel=j.actual_vel,
                    actual_acc=j.actual_acc,
                    actual_jerk=j.actual_jerk,
                    set_pos=j.set_pos,
                    set_vel=j.set_vel,
                    set_acc=j.set_acc,
                    torque_set=j.torque_set
                )
            return JointTelemetry()


# ==============================================================================
# ĐOẠN CODE TEST KHI CHẠY TRỰC TIẾP FILE NÀY (STANDALONE TEST)
# ==============================================================================
if __name__ == "__main__":
    import os
    # Đảm bảo UTF-8 trên Windows console
    if sys.platform.startswith("win"):
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except Exception:
            pass

    print("=" * 60)
    print("       CHUONG TRINH KIEM TRA GIAO TIEP UART ESP32 - PI 5")
    print("=" * 60)

    # Khởi tạo UART Manager
    uart_mgr = ESP32UARTManager()
    detected_port = sys.argv[1] if len(sys.argv) > 1 else uart_mgr.auto_detect_port()
    print(f"[*] Cong UART duoc chon: {detected_port}")

    # Hàm in telemetry khi nhận được gói tin từ ESP32
    def print_telemetry(data: RobotTelemetry):
        j0 = data.joints[0]
        print(
            f"\r[DATA] J1: Pos={j0.actual_pos:6.2f} (Set={j0.set_pos:6.2f}) | "
            f"Vel={j0.actual_vel:6.2f} | Tor={j0.torque_set:6.2f} | "
            f"Packets: {uart_mgr.packets_received} OK, {uart_mgr.packets_crc_error} CRC err",
            end=""
        )

    uart_mgr.register_telemetry_callback(print_telemetry)

    # Thử kết nối
    success = uart_mgr.connect(detected_port)
    if not success:
        print(f"[-] Khong the mo cong {detected_port}. Chay o che do Demo Frame Encoding...")
        # Demo tạo frame gửi lệnh
        demo_frame_limit = uart_mgr._build_frame(
            UartRxCmd.UPDATE_JOINT_LIMIT, JointID.HIP, struct.pack("<fff", 60.0, 120.0, 300.0)
        )
        print(f"[+] Frame Cap nhat Limit (Hex): {' '.join(f'{b:02X}' for b in demo_frame_limit)}")

        demo_frame_target = uart_mgr._build_frame(
            UartRxCmd.SET_JOINT_TARGET, JointID.HIP, struct.pack("<f", 45.0 / 360.0)
        )
        print(f"[+] Frame Dat goc muc tieu (Hex): {' '.join(f'{b:02X}' for b in demo_frame_target)}")
    else:
        print("[+] Dang lang nghe du lieu tu ESP32... Nhan Ctrl+C de dung.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[!] Dang dung...")
        finally:
            uart_mgr.disconnect()
            print("[+] Hoan tat.")
