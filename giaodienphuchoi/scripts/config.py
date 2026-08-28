"""Cấu hình chung — chạm vào đây để đổi platform (Windows ↔ Pi5).

Tự động phát hiện:
    - Windows  → COMx (vd: COM21)
    - Linux    → /dev/ttyUSBx hoặc /dev/ttyACMx (Pi/RPi/Jetson)
    - macOS    → /dev/cu.usbserial-*

Ngoài ra, có thể:
    - Override cứng qua biến môi trường `TWAI_SERIAL_PORT`
    - Override cứng qua file `serial_port_override.txt` (1 dòng, vd `/dev/ttyUSB0`)
    - Tự động scan port đầu tiên có ESP32 (vid=0x303a cho ESP32-S3, 0x10c4 cho CP210x)
"""
import os
import sys
import glob
import serial.tools.list_ports  # pyserial


# ─────────────────────────────────────────────────────────────────────────────
#  Hằng số mặc định
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_BAUD = 115200
DEFAULT_WINDOWS_PORT = "COM21"
DEFAULT_LINUX_PORT = "/dev/ttyUSB0"
DEFAULT_MACOS_PORT = "/dev/cu.usbserial"

# VID phổ biến cho ESP32 / USB-UART / debug cổng.
ESP32_VIDS = {
    0x303A,  # ESP32-S2/S3 builtin USB-Serial/JTAG (Espressif direct)
    0x10C4,  # CP210x (Silicon Labs) — kit ESP32 DevKit thường dùng
    0x1A86,  # CH340/CH341/CH343 (NanJing Qinheng) — kit rẻ Trung Quốc
    0x0403,  # FTDI (FT232)
    0x067B,  # Prolific PL2303
    0x2341,  # Arduino (hiếm)
    0x2E8A,  # Raspberry Pi Pico (RP2040)
}


# ─────────────────────────────────────────────────────────────────────────────
#  Auto-detect
# ─────────────────────────────────────────────────────────────────────────────
def list_serial_ports() -> list[str]:
    """Trả về list COM/TTY port hiện có trên máy."""
    return [p.device for p in serial.tools.list_ports.comports()]


def find_esp32_port() -> str | None:
    """Tìm port có VID của ESP32 / USB-UART. Trả None nếu không thấy.

    Trên Pi5 thường thấy:
        - /dev/ttyUSB0 (CP210x)
        - /dev/ttyACM0 (ESP32-S3 native USB)
    """
    for port in serial.tools.list_ports.comports():
        if port.vid in ESP32_VIDS:
            return port.device
    return None


def detect_serial_port(explicit: str | None = None) -> str:
    """Phát hiện serial port ưu tiên:

    1. `explicit` (tham số truyền vào từ GUI).
    2. Biến môi trường `TWAI_SERIAL_PORT`.
    3. File `serial_port_override.txt` cùng thư mục.
    4. Auto-detect VID (Pi5/Win).
    5. Fallback OS-specific (COM21 / /dev/ttyUSB0).
    """
    if explicit:
        return explicit

    env = os.environ.get("TWAI_SERIAL_PORT")
    if env:
        return env.strip()

    override_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "serial_port_override.txt")
    if os.path.exists(override_path):
        try:
            with open(override_path, "r", encoding="utf-8") as f:
                line = f.readline().strip()
                if line:
                    return line
        except Exception:
            pass

    auto = find_esp32_port()
    if auto:
        return auto

    if sys.platform.startswith("win"):
        return DEFAULT_WINDOWS_PORT
    if sys.platform.startswith("linux"):
        return DEFAULT_LINUX_PORT
    if sys.platform == "darwin":
        return DEFAULT_MACOS_PORT
    return DEFAULT_LINUX_PORT


def detect_baud(explicit: int | None = None) -> int:
    """Tương tự detect_serial_port — ưu tiên explicit, env, default."""
    if explicit:
        return int(explicit)
    env = os.environ.get("TWAI_BAUD")
    if env:
        try:
            return int(env.strip())
        except ValueError:
            pass
    return DEFAULT_BAUD


# ─────────────────────────────────────────────────────────────────────────────
#  Diagnosis helper — in ra cho user debug
# ────────────────────────────────────────────────────────────────────────
def print_diagnosis() -> None:
    """In cấu hình hiện tại ra console — gọi khi khởi động GUI để user biết."""
    print("=" * 60)
    print(f"[config] Platform       : {sys.platform}")
    print(f"[config] Python         : {sys.version.split()[0]}")
    print(f"[config] Serial port    : {detect_serial_port()}")
    print(f"[config] Baudrate       : {detect_baud()}")
    print(f"[config] Available ports:")
    for port in serial.tools.list_ports.comports():
        # vid/pid có thể None khi driver chưa load đúng (ESP32 chưa enumerate)
        vid = f"vid={hex(port.vid)}" if port.vid is not None else "vid=?"
        pid = f"pid={hex(port.pid)}" if port.pid is not None else "pid=?"
        desc = port.description or "(no description)"
        print(f"    - {port.device}  {vid} {pid} {desc}")
    print("=" * 60)


if __name__ == "__main__":
    print_diagnosis()