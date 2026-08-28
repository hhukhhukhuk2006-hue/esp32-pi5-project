# single_joint_controller.py
#
# Backend mới cho giaodienphuchoi — chạy thuần code ODESC pattern, KHÔNG qua ESP32.
# Copy y nguyên từ test/ODESC_Control_GUI/Trajectory_controller.py (ODriveThread).
#
# Điểm giữ nguyên từ ODESC:
#   - threading.Thread, run() loop ~10ms (sleep 0.01s)
#   - 1 khớp duy nhất (single axis), không phải list 3 phần tử
#   - Trajectory swap giữa Trap/Cubic/Quintic/Spline/Sinusoidal
#   - SavGol-like filter (window=25, poly_order=2) cho vel_filtered/acc/jerk
#   - CTC single-axis: tor = Ic*(qddot_d - Kp*ep - Kd*ev) + m*g*lc*cos(q) + friction
#   - tor = tor / tor_coef / gear_ratio  (motor-side torque → gửi xuống ODrive)
#   - set_offset() KHÔNG reset target (chỉ ghi encoder offset)
#   - update_ctrlElms(target, max_v, Kp, Kd, ctrl_bw, enc_bw) → 1-nút Move
#   - axis0 = motor (input_torque), axis1 = encoder (pos_estimate) — quirk phần cứng
#
# Điểm khác ODESC:
#   - Đổi tên class thành SingleJointController cho rõ nghĩa.
#   - Bỏ comment "odrive_controller.py" ở đầu (file này là backend chứ không phải test).
#   - Import từ backend.trajectory / backend.kinematic_calculate (thay vì cùng thư mục).

import threading
import time
import math
from collections import deque

# odrive lib — pip install odrive
try:
    import odrive
    from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE
    from odrive.enums import CONTROL_MODE_TORQUE_CONTROL
    from odrive.enums import INPUT_MODE_TORQUE_RAMP
    ODRIVE_AVAILABLE = True
except ImportError as _e:
    print(f"[SingleJointController] Không import được odrive: {_e}")
    print("                    Cài đặt: pip install odrive")
    odrive = None
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8  # ODrive canonical
    AXIS_STATE_IDLE = 1
    CONTROL_MODE_TORQUE_CONTROL = 1     # axis.controller.input_torque — direct
    INPUT_MODE_TORQUE_RAMP = 6          # axis.controller.input_torque + ramp
    ODRIVE_AVAILABLE = False

from .kinematic_calculate import get_acc_jerk
from .trajectory import (
    TrapezoidalTrajectory,
    CubicTrajectory,
    QuinticTrajectory,
    SplineTrajectory,
    SinusoidalTrajectory,
)

import numpy as np


# ── Constants (giống ODESC) ──────────────────────────────────────────────────
CLOSED_LOOP_CONTROL = AXIS_STATE_CLOSED_LOOP_CONTROL
IDLE = AXIS_STATE_IDLE
DEG2RAD = math.pi / 180.0
gear_ratio = 50.0
g = 9.81


class SingleJointController(threading.Thread):
    """Điều khiển 1 khớp (knee — M1 motor + M1 encoder) theo pattern ODESC.

    Hardware mapping (giống ODESC):
        - axis0 = motor điều khiển (ghi input_torque)
        - axis1 = encoder đọc vị trí khớp (pos_estimate, vel_estimate)
    """

    def __init__(self):
        super().__init__()

        # ── System ────────────────────────────────────────────────────────
        self.connected = False
        self.closed_loop_control = False
        self.isCalibrated = False
        self.isOffset = False
        self.error = False
        self.data_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._estop_event = threading.Event()
        self.data = deque(maxlen=800)
        self.control_loop = 0.001

        # ── Driver components ─────────────────────────────────────────────
        self.odrv = None
        self.axis = None
        self.max_torque = 0.0
        self.Kt = 0.0
        self.ctrl_bandwidth = 0.0
        self.enc_bandwidth = 0.0

        # ── Kinetic components ────────────────────────────────────────────
        self.max_vel = 60.0         # deg/s
        self.start_pos = -90.0      # home world-frame (deg)
        self.offset = 0.0           # encoder offset (raw rev)
        self.pos_set = self.start_pos
        self.vel_set = 0.0
        self.acc_set = 0.0
        self.pos = 0.0
        self.vel = 0.0
        self.preT = 0.0             # time ref cho control_loop period
        self.t_ref = -math.inf      # trajectory's time reference
        self.traj = QuinticTrajectory()

        # ── Physic components (link mass / length / inertia) ──────────────
        self.link_mass = 7.197
        self.link_length = 0.53175
        self.center_distance = 0.295
        self.motor_inertia = 0.000643
        self.const_inertia = 0.8236 + gear_ratio ** 2 * self.motor_inertia

        # ── Load parameters ───────────────────────────────────────────────
        self.hanger_mass = 0.2
        self.ext_load = 0.0
        self.hanger_distance = 0.02

        self.m = self.link_mass + self.hanger_mass
        self.lc = (self.center_distance * self.link_mass + self.hanger_distance * self.hanger_mass) / self.m
        self.Ic = self.const_inertia + self.hanger_mass * (self.hanger_distance ** 2)

        # ── Friction (Nm ở joint-side) ────────────────────────────────────
        self.static_friction = 0.092
        self.coul_friction = 0.07
        self.visc_friction = 0.00276 * gear_ratio ** 2

        # ── Control gains ─────────────────────────────────────────────────
        # ⚠ Kp là motor-side gain (sau khi đã chia gear_ratio ở CTC). Vì
        # CTC formula tor = (Kp * ep + Kd * ev + ...) / gear_ratio, Kp
        # thực tế bị chia cho gear. Nên Kp motor-side cần lớn (200) để
        # p_term_motor đủ mạnh thắng friction + gravity khi cos(q) lớn.
        self.Kp = 40
        self.Kd = 8
        self.tor_coef = 1.0
        self.torque_set = 0.0

        # ── ODrive control mode (chọn qua set_control_mode): ───────────────
        #   "torque" → CONTROL_MODE_TORQUE_CONTROL (1) + INPUT_MODE_PASSTHROUGH (1)
        #   "ramp"   → CONTROL_MODE_TORQUE_CONTROL (1) + INPUT_MODE_TORQUE_RAMP (6)
        # Mặc định "ramp" — ODrive smooth setpoint theo torque_ramp_rate, giảm
        # dao động CTC. Đổi sang "torque" nếu cần response tức thì.
        self._control_mode_name = "ramp"
        self._input_mode_value = INPUT_MODE_TORQUE_RAMP   # 6

        # ── SavGol-like filter ────────────────────────────────────────────
        self.window_size = 25                          # odd, > poly_order+1
        self.poly_order = 2
        self.velFilBuf = deque(maxlen=self.window_size)
        self.timeFilBuf = deque(maxlen=self.window_size)
        self.t_filter_ref = 0

    # ─────────────────────────────────────────────────────────────────────
    # Trajectory type swap (giống ODriveThread.set_trajectory_type)
    # ─────────────────────────────────────────────────────────────────────
    def set_trajectory_type(self, traj_type):
        """Set trajectory type by name: Trapezoidal/Cubic/Quintic/Spline/Sinusoidal."""
        try:
            if traj_type == "Trapezoidal":
                self.traj = TrapezoidalTrajectory()
            elif traj_type == "Cubic":
                self.traj = CubicTrajectory()
            elif traj_type == "Quintic":
                self.traj = QuinticTrajectory()
            elif traj_type == "Spline":
                self.traj = SplineTrajectory()
            elif traj_type == "Sinusoidal":
                self.traj = SinusoidalTrajectory()
            else:
                print(f"[SingleJointController] Unknown trajectory type: {traj_type}, using Quintic")
                self.traj = QuinticTrajectory()
            print(f"[SingleJointController] Trajectory type changed to: {traj_type}")
        except Exception as e:
            print(f"[SingleJointController] Error changing trajectory type: {e}")

    # ─────────────────────────────────────────────────────────────────────
    # Connection
    # ─────────────────────────────────────────────────────────────────────
    def connect(self):
        """Tìm và kết nối ODrive qua USB.

        Thứ tự QUAN TRỌNG (giống ODESC):
            1. Đọc firmware torque_lim vào biến tạm fw_torque_lim
            2. Nếu self.max_torque đã được set trước bởi set_load_params()
               VÀ self.max_torque > fw_torque_lim → đẩy lên firmware
            3. SAU ĐÓ mới gán self.max_torque = max(self.max_torque, fw_torque_lim)
        """
        if not ODRIVE_AVAILABLE:
            print("[SingleJointController] odrive không khả dụng — bỏ qua connect()")
            return
        try:
            print("[SingleJointController] Connecting to ODrive...")
            self.odrv = odrive.find_any(timeout=10)
            if self.odrv:
                print(f"[SingleJointController] Connected: serial={self.odrv.serial_number}")
                self.axis = self.odrv.axis0
                self.Kt = self.axis.motor.config.torque_constant
                self.ctrl_bandwidth = self.axis.motor.config.current_control_bandwidth
                self.enc_bandwidth = self.axis.encoder.config.bandwidth
                self.connected = True
                self.error = False
                # ── Sync torque_lim với firmware ─────────────────────
                try:
                    fw_torque_lim = float(self.axis.motor.config.torque_lim)
                    if self.max_torque > fw_torque_lim + 1e-3:
                        print(f"[SingleJointController] Raising torque_lim: "
                              f"{fw_torque_lim:.2f} → {self.max_torque:.2f} Nm")
                        self.axis.motor.config.torque_lim = self.max_torque
                    # Đảm bảo self.max_torque không nhỏ hơn firmware
                    if self.max_torque < fw_torque_lim:
                        self.max_torque = fw_torque_lim
                except Exception as e:
                    print(f"[SingleJointController] sync torque_lim warning: {e}")
            else:
                print("[SingleJointController] ODrive not found")
        except Exception as e:
            self.error = True
            print(f"[SingleJointController] Connection failed: {e}")

    def clear_error(self):
        try:
            if self.axis is not None:
                self.axis.controller.error = 0
                self.axis.encoder.error = 0
                self.axis.motor.error = 0
                self.axis.error = 0
            self.error = False
            print("[SingleJointController] Errors cleared")
        except Exception as e:
            print(f"[SingleJointController] clear_error lỗi: {e}")

    def set_control_mode(self, name: str):
        """Chọn ODrive control mode: 'torque' (direct) hoặc 'ramp' (smooth).

        Phải gọi TRƯỚC enter_closed_loop(). Nếu đã ở closed loop thì set
        xong cần return_IDLE() rồi enter_closed_loop() lại.

        Trong bản odrive này:
            - Cả 2 mode đều dùng control_mode = TORQUE_CONTROL (1)
            - Chỉ khác nhau input_mode (axis.controller.config.input_mode):
                1 = INPUT_MODE_PASSTHROUGH (direct)
                6 = INPUT_MODE_TORQUE_RAMP (ramp theo torque_ramp_rate)
        """
        name = name.lower()
        if name == "torque":
            self._input_mode_value = 1        # INPUT_MODE_PASSTHROUGH
        elif name == "ramp":
            self._input_mode_value = INPUT_MODE_TORQUE_RAMP   # 6
        else:
            print(f"[SingleJointController] Unknown control mode '{name}', "
                  f"keeping {self._control_mode_name}")
            return
        self._control_mode_name = name
        print(f"[SingleJointController] control_mode set to: {name} "
              f"(input_mode={self._input_mode_value})")

    def enter_closed_loop(self):
        """Set ODrive axis sang CLOSED_LOOP_CONTROL + (TORQUE_CONTROL | TORQUE_RAMP).

        Backend mới (khác ODESC):
            - Phải ép control_mode mỗi lần vào closed loop.
              Nếu ODrive config sẵn qua odrivetool thì không cần, nhưng nếu
              firmware mất config (sau reboot) thì input_torque bị ignore.
            - Chọn mode qua set_control_mode("torque"|"ramp") trước khi gọi.

        Returns True nếu vào được CLOSED_LOOP_CONTROL.
        """
        if not self.connected or self.axis is None:
            print("[SingleJointController] enter_closed_loop: ODrive chưa connect")
            return False
        self.clear_error()
        try:
            # 1) Set control_mode = TORQUE_CONTROL + input_mode (ramp/passthrough)
            try:
                cur_ctrl = int(self.axis.controller.config.control_mode)
                cur_in   = int(self.axis.controller.config.input_mode)
                if cur_ctrl != CONTROL_MODE_TORQUE_CONTROL:
                    print(f"[SingleJointController] Switch control_mode: "
                          f"{cur_ctrl} → TORQUE_CONTROL ({CONTROL_MODE_TORQUE_CONTROL})")
                    self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
                if cur_in != self._input_mode_value:
                    print(f"[SingleJointController] Switch input_mode: "
                          f"{cur_in} → {self._control_mode_name} ({self._input_mode_value})")
                    self.axis.controller.config.input_mode = self._input_mode_value
                # Nếu ramp mode → set ramp rate cao để không lag CTC
                if self._input_mode_value == INPUT_MODE_TORQUE_RAMP:
                    try:
                        self.axis.controller.config.torque_ramp_rate = 30.0
                        print("[SingleJointController] torque_ramp_rate = 30 Nm/s")
                    except Exception as re:
                        print(f"[SingleJointController] torque_ramp_rate warning: {re}")
            except Exception as e:
                print(f"[SingleJointController] set control_mode warning: {e}")

            # 2) Enter CLOSED_LOOP_CONTROL state
            self.axis.requested_state = CLOSED_LOOP_CONTROL
            self.closed_loop_control = True
            print(f"[SingleJointController] Entered CLOSED_LOOP_CONTROL "
                  f"(mode={self._control_mode_name}).")
            return True
        except Exception as e:
            print(f"[SingleJointController] enter_closed_loop lỗi: {e}")
            return False

    def return_IDLE(self):
        try:
            if self.axis is not None:
                self.axis.controller.input_torque = 0
                self.axis.requested_state = IDLE
            self.closed_loop_control = False
        except Exception as e:
            print(f"[SingleJointController] return_IDLE lỗi: {e}")

    def is_controlable(self):
        return (self.connected
                and self.closed_loop_control
                and self.isOffset
                and not self._estop_event.is_set())

    # ─────────────────────────────────────────────────────────────────────
    # update_ctrlElms — 1-nút Move (giống ODriveThread)
    # ─────────────────────────────────────────────────────────────────────
    def update_ctrlElms(self, *ctrlElms):
        """Set control parameters + tính trajectory params.

        Args:
            ctrlElms[0] = target    (deg, world frame)
            ctrlElms[1] = max_vel   (deg/s)
            ctrlElms[2] = Kp
            ctrlElms[3] = Kd
            ctrlElms[4] = ctrl_bandwidth  (Hz)
            ctrlElms[5] = enc_bandwidth   (Hz)
        """
        try:
            with self.data_lock:
                self.t_ref = time.time()
                target = ctrlElms[0]
                self.max_vel = ctrlElms[1]
                self.Kp = ctrlElms[2]
                self.Kd = ctrlElms[3]
                self.ctrl_bandwidth = ctrlElms[4]
                self.enc_bandwidth = ctrlElms[5]
                if self.axis is not None:
                    self.axis.motor.config.current_control_bandwidth = self.ctrl_bandwidth
                    self.axis.encoder.config.bandwidth = self.enc_bandwidth
            self.traj.param_calc(self.pos, target, self.max_vel)
        except Exception as e:
            print(f"[SingleJointController] update_ctrlElms lỗi: {e}")

    def update_loadParms(self, *loadParms):
        """Set load/friction params.

        Args:
            loadParms[0] = ext_load        (kg)
            loadParms[1] = hanger_distance (m)
            loadParms[2] = static_friction (Nm)
            loadParms[3] = coul_friction   (Nm)
            loadParms[4] = visc_friction   (Nm/rad)
            loadParms[5] = max_torque      (Nm, motor-side)
        """
        try:
            self.ext_load = loadParms[0]
            self.hanger_distance = loadParms[1]
            self.static_friction = loadParms[2]
            self.coul_friction = loadParms[3]
            self.visc_friction = loadParms[4]
            self.m = self.link_mass + self.hanger_mass + self.ext_load
            self.lc = (self.center_distance * self.link_mass
                       + self.hanger_distance * (self.hanger_mass + self.ext_load)) / self.m
            self.Ic = self.const_inertia + (self.hanger_mass + self.ext_load) * (self.hanger_distance ** 2)
            new_max_torque = loadParms[5]
            self.max_torque = new_max_torque
            # Đồng thời đẩy max_torque lên ODrive firmware để không bị clamp.
            # Nếu chưa connect thì bỏ qua, sẽ được connect() xử lý.
            try:
                if self.connected and self.axis is not None:
                    cur_lim = float(self.axis.motor.config.torque_lim)
                    if new_max_torque > cur_lim + 1e-3:
                        print(f"[SingleJointController] update_loadParms: "
                              f"torque_lim {cur_lim:.2f} → {new_max_torque:.2f} Nm")
                        self.axis.motor.config.torque_lim = new_max_torque
            except Exception as e:
                print(f"[SingleJointController] update_loadParms → torque_lim warning: {e}")
        except Exception as e:
            print(f"[SingleJointController] update_loadParms lỗi: {e}")

    # ─────────────────────────────────────────────────────────────────────
    # State / EStop / data
    # ─────────────────────────────────────────────────────────────────────
    def get_state(self):
        if self.connected and self.axis is not None:
            try:
                return self.axis.current_state
            except Exception:
                return None
        return None

    def emergency_stop(self):
        """Immediate torque -> 0 và chặn ghi torque cho đến reset."""
        self._estop_event.set()
        self.torque_set = 0.0
        try:
            if self.connected and self.axis is not None:
                self.axis.controller.input_torque = 0
        except Exception:
            pass

    def get_data(self):
        """Trả về list các dòng (t, pos, vel_filtered, acc, pos_set, vel_set, acc_set, jerk, tor_set)."""
        with self.data_lock:
            return list(self.data)

    def set_offset(self):
        """Lưu vị trí encoder hiện tại làm home. KHÔNG reset target (giống ODESC).
        Returns True nếu thành công."""
        if not self.connected or self.odrv is None:
            print("[SingleJointController] set_offset: ODrive chưa connect")
            return False
        try:
            self.offset = self.odrv.axis1.encoder.pos_estimate
            self.isOffset = True
            print(f"[SingleJointController] set_offset: raw={self.offset:.4f} rev, world home={self.start_pos}°")
            return True
        except Exception as e:
            print(f"[SingleJointController] set_offset lỗi: {e}")
            return False

    def reset(self):
        """Reset trajectory + filter + closed_loop + estop + offset."""
        self.traj.reset()
        self.t_ref = -math.inf
        self.velFilBuf.clear()
        self.timeFilBuf.clear()
        self.return_IDLE()
        self.isOffset = False
        self._estop_event.clear()

    def stop(self):
        """Thread shutdown."""
        self._stop_event.set()
        try:
            if self.axis is not None:
                self.axis.controller.input_torque = 0
                time.sleep(0.05)
                self.axis.requested_state = AXIS_STATE_IDLE
                time.sleep(0.05)
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────────────
    # Trajectory + CTC + friction
    # ─────────────────────────────────────────────────────────────────────
    def setTarget(self):
        t = time.time() - self.t_ref
        self.pos_set, self.vel_set, self.acc_set = self.traj.desired_state(t)

    def friction_handeling(self, ep, qdot, vel_threshole=2.0):
        stFricDir = ((ep < 0) - (ep > 0)) * ((qdot > -vel_threshole) and (qdot < vel_threshole))
        coFricDir = (qdot > vel_threshole) - (qdot < -vel_threshole)
        return (self.static_friction * stFricDir
                + self.coul_friction * coFricDir
                + self.visc_friction * qdot)

    def dynamic_calculation(self):
        m = self.m
        lc = self.lc
        Ic = self.Ic

        self.setTarget()
        q = self.pos * DEG2RAD
        qdot = self.vel * DEG2RAD
        q_d = self.pos_set * DEG2RAD
        qdot_d = self.vel_set * DEG2RAD
        qddot_d = self.acc_set * DEG2RAD

        Kp = self.Kp
        Kd = self.Kd
        ep = q - q_d
        ev = qdot - qdot_d

        tor = Ic * (qddot_d - (Kp * ep + Kd * ev)) + m * g * lc * math.cos(q) + self.friction_handeling(ep, qdot)
        tor = (tor) / self.tor_coef / gear_ratio
        self.torque_set = max(min(tor, self.max_torque), -self.max_torque)

    # ─────────────────────────────────────────────────────────────────────
    # Main loop
    # ─────────────────────────────────────────────────────────────────────
    def run(self):
        while not self._stop_event.is_set():
            t_start = time.perf_counter()
            try:
                if not self.connected:
                    self.connect()
                    if not self.connected:
                        time.sleep(0.1)
                        continue

                if self._estop_event.is_set():
                    self._stop_event.wait(0.1)
                    continue

                # ── Data collect ─────────────────────────────────────────
                with self.data_lock:
                    t = time.time()
                    deltaT = t - self.preT
                    self.pos = (self.odrv.axis1.encoder.pos_estimate - self.offset) * 360.0 + self.start_pos
                    self.vel = self.odrv.axis1.encoder.vel_estimate * 360.0  # raw velocity

                    vel_filtered = 0.0
                    acc = 0.0
                    jerk = 0.0
                    if deltaT >= self.control_loop:
                        self.velFilBuf.append(self.vel)
                        self.timeFilBuf.append(t)

                        if len(self.velFilBuf) == self.window_size:
                            vel_arr = np.array(list(self.velFilBuf))
                            t_arr = np.array(list(self.timeFilBuf))
                            vel_filtered, acc, jerk = get_acc_jerk(
                                t_arr, vel_arr, self.window_size, self.poly_order
                            )

                    try:
                        tor_set = self.axis.motor.current_control.Iq_setpoint * self.Kt
                    except Exception:
                        tor_set = 0.0
                    self.data.append((t, self.pos, vel_filtered, acc,
                                      self.pos_set, self.vel_set, self.acc_set, jerk, tor_set))

                    if len(self.data) > 800:
                        self.data = self.data[-800:]

                # ── Control ──────────────────────────────────────────────
                if self.is_controlable():
                    with self.data_lock:
                        self.dynamic_calculation()
                        try:
                            self.axis.controller.input_torque = self.torque_set
                        except Exception as e:
                            if not getattr(self, "_write_err_log_ts", False):
                                print(f"[SingleJointController] input_torque write lỗi: {e}")
                                self._write_err_log_ts = True
                else:
                    self.pos_set = self.start_pos
                    self.torque_set = 0.0

            except Exception as e:
                print(f"[SingleJointController] ODrive error: {e}")
                self.connected = False
                self.closed_loop_control = False
                self.error = True
                time.sleep(1)

            t_end = time.perf_counter()
            t_sleep = 0.01 - (t_end - t_start)
            if t_sleep > 0:
                self._stop_event.wait(timeout=t_sleep)
