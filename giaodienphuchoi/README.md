# Giao diện phục hồi chức năng — Phcn

Phiên bản PyQt5 của giao diện LLRR_app, được **adapt** để giao tiếp với
embedded computer chạy `twai_serial_controller1.TWAIController` thay vì
Arduino cũ. Toàn bộ phần frontend (giao diện, dialog, tài khoản, session
block, exercise block) được **giữ nguyên 100%** từ LLRR_app — chỉ phần
backend trong `mainscreen.py` được thay thế.

## Cấu trúc thư mục

```
giaodienphuchoi/
├── runapp.bat                        ← Windows shortcut (đã copy từ LLRR)
├── README.md                         ← file này
├── assets/
│   ├── resources.qrc                 ← Qt resource file
│   └── icons/                        ← 15 file PNG (sine_hip, sine_knee, ...)
├── database/                         ← account + patient data + reports
│   ├── dev/
│   │   ├── dev_dirs.json             ← danh sách tài khoản KTV
│   │   ├── doctor_dirs.json          ← danh sách tài khoản BS
│   │   ├── patient_dirs.json         ← danh sách tài khoản BN
│   │   └── devs-data/KTV0/...        ← control_data.json + reports
│   └── doctors/
│       └── BS0/                      ← bệnh nhân của BS0
│           ├── patients-info.json
│           ├── BN0/, BN1/, ...       ← thư mục từng bệnh nhân
└── scripts/                          ← code Python
    ├── GUI.py                        ← entry point (chạy file này)
    ├── login.py                      ← màn hình đăng nhập (giữ nguyên LLRR)
    ├── popups.py                     ← dialog (giữ nguyên LLRR)
    ├── mainscreen.py                 ← ★ ADAPTED — backend Arduino → embedded
    ├── controller_wrapper.py         ← ★ MỚI — adapter cho TWAIController
    ├── ctc_wrapper.py                ← ★ MỚI — CTC chạy trên embedded computer
    ├── trajectory_wrapper.py         ← ★ MỚI — Spline/Quintic/Sine/Bike + JerkTracker
    ├── resources_rc.py               ← Qt resource compiled (giữ nguyên LLRR)
    └── UiScripts/                    ← 10 file UI từ Qt Designer (giữ nguyên LLRR)
        ├── login_ui.py
        ├── patient_ui.py
        ├── doctor_ui.py
        ├── dev_ui.py
        └── ...
```

## Kiến trúc — phần gì giữ, phần gì đổi

### Giữ nguyên 100% từ LLRR_app
- `login.py` — luồng đăng nhập, validate ID/password, mở mainscreen theo role
- `popups.py` — 6 dialog (SessionDialog, ReportDialog, AccountDialog,
  AccountManagerDialog, SetAngleDialog, SetSineDialog)
- `UiScripts/*.py` — 10 file UI generated từ Qt Designer
- `assets/icons/*` — toàn bộ icon PNG
- `resources_rc.py` — Qt resource compiled
- `database/**` — toàn bộ account + patient data + reports

### MỚI — backend adapter
- **`controller_wrapper.py`** — `EmbeddedBackend` class
  - Bọc `TWAIController`, cung cấp API tối thiểu giống Arduino Serial
    (`connect()`, `send_cmd()`, `receive_packet()`, `is_connected`).
  - Polling thread 200Hz → build `FeedbackPacket` (q_set, q_fb, error)
  - Helpers: `set_joint_target_deg`, `set_gains`, `set_prismatic`,
    `set_offset`, `enter_closed_loop`, `start_motion`, `stop_motion`,
    `emergency_stop`, `reset`, `go_home`.

- **`ctc_wrapper.py`** — `CTCComputer` class
  - Chạy CTC trên embedded computer (parallel với ESP32 on-board) để
    preview torque, plot, debug.
  - Hai mode: `'full'` (M·C+G) hoặc `'scalar'` (ODESC-style).

- **`trajectory_wrapper.py`** — wrappers cho trajectory classes
  - `ThreeJointTrajectoryPlanner` — Spline/Quintic/Cubic/Trapezoidal
  - `SineTrajectory` — single joint sine (thay Arduino formula)
  - `BikeTrajectory` — 3-joint bike exercise
  - `JerkTracker` — SavGol filter từ `kinematic.get_acc_jerk`

### ADAPTED — `mainscreen.py`
- Giữ nguyên class hierarchy: `UserMainScreen` → `DoctorPatient`,
  `DoctorDev` → `PatientMainScreen`, `DoctorMainScreen`, `DevMainScreen`.
- Thay `self.ser` / `self.ard` / `sendToArduino` / `recvFromArduino` /
  `waitForArduino` / `safety_switched` (10 limit switch) bằng
  `self.backend` (EmbeddedBackend).
- Calibration 10 switch → `set_offset() + enter_closed_loop()`.
- Arduino protocol `<mode,data,...>` được dịch sang TWAIController methods
  trong `EmbeddedBackend.send_cmd()`.
- Sine trajectory → dùng `SineTrajectory` wrapper + 10Hz motion loop
  gọi `set_joint_target_deg()` mỗi tick.

## Cách chạy

### 1. Yêu cầu
- Python 3.9+
- PyQt5
- numpy, scipy (cho SavGol filter trong `kinematic.py`)
- pyserial (cho TWAIController)
- Đã cài đặt project `phcn/` (chứa `twai_serial_controller1.py`,
  `ctc_3dof.py`, `trajectory.py`, `kinematic.py`).

```powershell
# Trong PowerShell, cd vào thư mục project root
cd d:\PyCharmMiscProject\phcn

# Activate venv (hoặc dùng system Python nếu đã cài package)
# .\giaodienphuchoi\venv_python\Scripts\activate  # nếu dùng venv riêng

pip install PyQt5 numpy scipy pyserial
```

### 2. Chạy GUI
```powershell
# Cách 1: Double-click runapp.bat trong giaodienphuchoi/
# Cách 2: Command line
cd d:\PyCharmMiscProject\phcn\giaodienphuchoi\scripts
python GUI.py
```

### 3. Đăng nhập
Dùng các tài khoản có sẵn trong `database/dev/*.json`:
- **KTV0** (Kỹ thuật viên) — full quyền + PID + account management
- **BS0** (Bác sỹ) — quản lý bệnh nhân
- **BN1** (Bệnh nhân) — chạy bài tập

Mật khẩu mặc định: xem trong file JSON tương ứng.

### 4. Kết nối embedded computer
GUI sẽ tự thử connect tới `COM21 @ 115200` (mặc định trong `GUI.py`).
Nếu ESP32 chưa cắm, GUI vẫn chạy được — bài tập sẽ ở **simulation mode**
(dùng trajectory wrapper giả lập, không có feedback thật từ động cơ).

Để đổi COM port: sửa `DEFAULT_COM_PORT` trong `scripts/mainscreen.py`
hoặc thêm UI trong `LoginScreen`.

## Luồng hoạt động (Patient role)

1. **Đăng nhập BN1** → mở mainscreen.
2. Nhập thông tin bệnh nhân (weight, height, thigh, shank) → Save.
3. **Tạo session** → nhập tiêu đề → Create.
4. **Chọn bài tập** (sine hip/knee/ankle hoặc bike).
5. **Đặt cycles/timer** → Confirm.
   - Nếu ESP32 ready: gửi data xuống + `set_offset` + `enter_closed_loop`.
   - Nếu chưa: chạy simulation mode.
6. **Start** → bắt đầu motion + nhận feedback 10Hz.
7. **Stop** → dừng motion, log feedback vào buffer.
8. **Save Report** → xem biểu đồ + xuất CSV.

## Tích hợp với code mới

Khi bạn sửa `twai_serial_controller1.py` / `ctc_3dof.py` / `trajectory.py`,
GUI sẽ tự động dùng phiên bản mới vì import trực tiếp (không bundle).

Khi bạn thêm API mới vào TWAIController, thêm wrapper method tương ứng
vào `EmbeddedBackend` (trong `controller_wrapper.py`) rồi gọi từ
`mainscreen.py`.

## Known limitations (MVP)

- Bài sine oscillation: motion loop 10Hz chạy trên PC (có thể upgrade lên
  ESP32 native sine bằng cách thêm lệnh mới vào TWAIController).
- Bike exercise: dùng 3 sine lệch pha cố định (không dùng kinematics
  inverse để tính từ quỹ đạo bàn đạp — sẽ cải tiến sau).
- Plot real-time: chưa có (chỉ log CSV, dùng ReportDialog cũ của LLRR).
- Auto-detect COM port: chưa có UI chọn port.

## Đóng góp

Frontend (UI, dialog, account, session) → sửa file trong `UiScripts/` (regen
từ Qt Designer) hoặc `popups.py`, `login.py`.

Backend logic (motion, control, communication) → sửa `controller_wrapper.py`,
`ctc_wrapper.py`, `trajectory_wrapper.py`, hoặc gọi trực tiếp API từ
`mainscreen.py`.

## Chạy trên Raspberry Pi 5

### Phần cứng cần
- Raspberry Pi 5 (4GB+)
- microSD 32GB+ class A2 (boot) hoặc NVMe SSD
- Màn hình cảm ứng DSI/HDMI 7"–10" (vd: DSI 7" official, hoặc HDMI 10")
- ESP32-S3 cài firmware `espidf_project/`
- Cáp USB-C nối Pi5 ↔ ESP32

### 1. Cài OS
Dùng Raspberry Pi Imager trên PC:
- Chọn **Raspberry Pi OS (64-bit, Bookworm) — Desktop**
- Bật SSH, đặt hostname `phcn-pi5`, user `phcn`, password tùy ý
- Cấu hình WiFi
- Flash, boot, SSH vào Pi5: `ssh phcn@phcn-pi5.local`

### 2. Cài dependencies (chạy trên Pi5)
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3 python3-venv python3-pip \
                    python3-pyqt5 python3-pyqtchart \
                    python3-numpy python3-scipy \
                    python3-serial git rsync

python3 --version   # 3.11.x
```

### 3. Copy code sang Pi5
Cách nhanh nhất — rsync từ PC:
```powershell
# Trên Windows PowerShell
scp -r D:\PyCharmMiscProject\phcn phcn@phcn-pi5.local:~/
```

Hoặc dùng git (nếu bạn đã push lên remote):
```bash
# Trên Pi5
cd ~
git clone https://your-repo.git phcn
```

### 4. Cho phép truy cập serial không cần sudo
```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER
sudo reboot
```

Sau reboot, kiểm tra ESP32 hiện ra:
```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
# Phải thấy crw-rw---- ... dialout /dev/ttyUSB0
```

### 5. Test kết nối
```bash
cd ~/phcn/giaodienphuchoi/scripts
python3 -c "
import sys
sys.path.insert(0, '~/phcn')
import config
config.print_diagnosis()
"
```
Kỳ vọng output:
```
[config] Platform       : linux
[config] Serial port    : /dev/ttyUSB0
[config] Available ports:
    - /dev/ttyUSB0  vid=0x1a86 pid=0x55d3 USB Enhanced Serial
```

### 6. Chạy GUI
```bash
cd ~/phcn/giaodienphuchoi/scripts
chmod +x runapp_linux.sh
./runapp_linux.sh
```

Hoặc trực tiếp:
```bash
cd ~/phcn/giaodienphuchoi/scripts
PYTHONPATH=~/phcn python3 GUI.py
```

GUI sẽ tự phát hiện:
- `QT_QPA_PLATFORM=xcb` nếu đang trên desktop LXDE (có X11)
- `QT_QPA_PLATFORM=eglfs` nếu dùng màn hình cảm ứng DSI/HDMI trực tiếp
- `QT_QPA_PLATFORM=linuxfb` nếu chỉ có framebuffer

### 7. Auto-start khi Pi5 boot

Sửa `giaodienphuchoi.service` (đã có sẵn trong thư mục) cho đúng
`User=` và `WorkingDirectory=`, sau đó:

```bash
sudo cp giaodienphuchoi.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable giaodienphuchoi.service
sudo systemctl start giaodienphuchoi.service

# Xem log
journalctl -u giaodienphuchoi -f
```

### 8. Kết nối ESP32

ESP32 firmware (file `espidf_project/main/main.c`) phải đã được flash
và nối qua USB với Pi5. Cổng `/dev/ttyUSB0` xuất hiện khi ESP32-S3
cắm vào.

Nếu ESP32 cắm mà không thấy port:
```bash
# Kiểm tra driver
sudo dmesg | tail -20
# Cài thêm driver nếu cần (CH340, CH343, CP210x)
sudo apt install -y linux-modules-extra-raspi
```

## Tổng kết các file quan trọng cho Pi5

```
giaodienphuchoi/
├── scripts/
│   ├── GUI.py                     ← entry point
│   ├── runapp_linux.sh            ← shell wrapper cho Linux
│   ├── config.py                  ← auto-detect port (Windows / Linux)
│   ├── controller_wrapper.py      ← adapter backend
│   ├── ctc_wrapper.py
│   ├── trajectory_wrapper.py
│   ├── mainscreen.py
│   ├── login.py
│   ├── popups.py
│   └── UiScripts/
└── giaodienphuchoi.service        ← systemd unit cho auto-start
```