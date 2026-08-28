"""SessionManager — coordinator cấp cao cho SingleJointController.

Cung cấp API giống `UnifiedBackend` cũ mà `mainscreen.py` đang gọi:
    - connect(), close(), is_connected
    - set_offset(), enter_closed_loop()
    - set_trajectory_mode(mode)         # Trap/Cubic/Quintic/Spline/Sinusoidal
    - set_move(target, max_v, Kp, Kd)   # 1-nút Move (giống update_ctrlElms)
    - stop_motion(), estop()
    - set_load_params(ext_load, hanger_distance, static_fric, coul_fric, visc_fric, max_torque)
    - set_filter_params(window_size, poly_order)
    - get_latest_state()                # dict pos/vel/torque để plot

Mục đích: GUI (chưa wire) có thể dùng SingleJointController trực tiếp,
hoặc dùng SessionManager để có API giống backend cũ.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from .single_joint_controller import SingleJointController
from .trajectory import (
    TrapezoidalTrajectory,
    CubicTrajectory,
    QuinticTrajectory,
    SplineTrajectory,
    SinusoidalTrajectory,
)


_TRAJ_NAME_TO_TYPE = {
    "trapezoidal": TrapezoidalTrajectory,
    "cubic": CubicTrajectory,
    "quintic": QuinticTrajectory,
    "spline": SplineTrajectory,
    "sinusoidal": SinusoidalTrajectory,
}


@dataclass
class SessionState:
    """Snapshot trạng thái SingleJointController tại 1 thời điểm (cho GUI)."""
    connected: bool = False
    closed_loop: bool = False
    is_offset: bool = False
    in_estop: bool = False
    error: bool = False

    pos_deg: float = 0.0
    vel_deg_s: float = 0.0
    pos_set_deg: float = 0.0
    vel_set_deg_s: float = 0.0
    acc_set_deg_s2: float = 0.0
    torque_nm: float = 0.0

    trajectory_mode: str = "quintic"
    kp: float = 40.0
    kd: float = 8.0
    max_vel: float = 60.0
    status_message: str = "Chưa khởi tạo"


class SessionManager:
    """Coordinator giữa GUI và SingleJointController.

    Khởi tạo 1 SingleJointController, chạy thread đọc/ghi encoder/torque,
    expose API cho GUI gọi.
    """

    def __init__(self):
        self.ctrl = SingleJointController()
        self._lock = threading.Lock()
        self._state = SessionState()
        self._state.trajectory_mode = "quintic"
        self._last_data_poll = 0.0

    # ── Lifecycle ───────────────────────────────────────────────────────
    def connect(self):
        """Khởi động thread controller (controller tự gọi connect() ODESC nếu chưa)."""
        if self.ctrl.is_alive():
            return
        self.ctrl.start()
        # Chờ connect() xong (max 15s)
        deadline = time.time() + 15.0
        while not self.ctrl.connected and time.time() < deadline:
            time.sleep(0.1)
        self._update_state()
        return self.ctrl.connected

    def close(self):
        """Stop thread + return ODrive sang IDLE."""
        if self.ctrl.is_alive():
            self.ctrl.stop()
            self.ctrl.join(timeout=3.0)
        self._update_state()

    @property
    def is_connected(self) -> bool:
        return self.ctrl.connected

    # ── Mode/params ─────────────────────────────────────────────────────
    def set_trajectory_mode(self, mode: str):
        """mode: trapezoidal / cubic / quintic / spline / sinusoidal."""
        mode_norm = (mode or "").strip().lower()
        if mode_norm not in _TRAJ_NAME_TO_TYPE:
            print(f"[SessionManager] Unknown trajectory mode: {mode}, dùng quintic")
            mode_norm = "quintic"
        with self._lock:
            self.ctrl.set_trajectory_type(mode_norm.capitalize())
            self._state.trajectory_mode = mode_norm

    def set_filter_params(self, window_size: int, poly_order: int):
        """Set SavGol filter window/poly_order."""
        with self._lock:
            self.ctrl.window_size = window_size
            self.ctrl.poly_order = poly_order
            # Reset buffers (giống ODESC GUI _on_apply_filter)
            from collections import deque as _dq
            self.ctrl.velFilBuf = _dq(maxlen=window_size)
            self.ctrl.timeFilBuf = _dq(maxlen=window_size)

    def set_load_params(self, ext_load: float, hanger_distance: float,
                        static_friction: float, coul_friction: float,
                        visc_friction: float, max_torque: float):
        """Set load/friction + max_torque (giống ODriveThread.update_loadParms)."""
        with self._lock:
            self.ctrl.update_loadParms(ext_load, hanger_distance,
                                       static_friction, coul_friction,
                                       visc_friction, max_torque)

    # ── Move (1-nút, giống ODESC) ───────────────────────────────────────
    def set_move(self, target_deg: float, max_vel: Optional[float] = None,
                 Kp: Optional[float] = None, Kd: Optional[float] = None,
                 ctrl_bandwidth: Optional[float] = None,
                 enc_bandwidth: Optional[float] = None):
        """1-nút Move: set target + gains → controller tính traj + clock start + CTC.

        Tất cả tham số optional: nếu None, dùng giá trị hiện tại của controller.
        """
        with self._lock:
            if max_vel is None:
                max_vel = self.ctrl.max_vel
            if Kp is None:
                Kp = self.ctrl.Kp
            if Kd is None:
                Kd = self.ctrl.Kd
            if ctrl_bandwidth is None:
                ctrl_bandwidth = self.ctrl.ctrl_bandwidth or 1000.0
            if enc_bandwidth is None:
                enc_bandwidth = self.ctrl.enc_bandwidth or 1000.0
            self.ctrl.update_ctrlElms(target_deg, max_vel, Kp, Kd,
                                      ctrl_bandwidth, enc_bandwidth)
            self._state.kp = Kp
            self._state.kd = Kd
            self._state.max_vel = max_vel

    # ── Lifecycle states (giống UnifiedBackend) ─────────────────────────
    def set_offset(self):
        """Lưu encoder pos làm home. Returns True nếu thành công."""
        with self._lock:
            return self.ctrl.set_offset()

    def enter_closed_loop(self):
        """Set ODrive sang CLOSED_LOOP_CONTROL. Returns True nếu thành công."""
        with self._lock:
            return self.ctrl.enter_closed_loop()

    def return_idle(self):
        """Set ODrive về IDLE."""
        with self._lock:
            self.ctrl.return_IDLE()

    def stop_motion(self):
        """Stop motion (giữ closed_loop) — set pos_set = start_pos, torque = 0."""
        with self._lock:
            self.ctrl.pos_set = self.ctrl.start_pos
            self.ctrl.torque_set = 0.0
            try:
                if self.ctrl.axis is not None:
                    self.ctrl.axis.controller.input_torque = 0.0
            except Exception:
                pass

    def estop(self):
        """Emergency stop — set estop event, torque = 0, chặn cho đến reset."""
        with self._lock:
            self.ctrl.emergency_stop()

    def reset_estop(self):
        with self._lock:
            self.ctrl.reset_estop()

    def reset(self):
        with self._lock:
            self.ctrl.reset()

    def clear_error(self):
        """Clear ODrive axis/controller/encoder/motor errors."""
        with self._lock:
            self.ctrl.clear_error()

    # ── Snapshot cho GUI ────────────────────────────────────────────────
    def get_state(self) -> SessionState:
        self._update_state()
        return self._state

    def get_data(self):
        """Raw data buffer (t, pos, vel_filt, acc, pos_set, vel_set, acc_set, jerk, tor_set)."""
        return self.ctrl.get_data()

    def _update_state(self):
        with self._lock:
            self._state.connected = self.ctrl.connected
            self._state.closed_loop = self.ctrl.closed_loop_control
            self._state.is_offset = self.ctrl.isOffset
            self._state.in_estop = self.ctrl._estop_event.is_set()
            self._state.error = self.ctrl.error

            self._state.pos_deg = self.ctrl.pos
            self._state.vel_deg_s = self.ctrl.vel
            self._state.pos_set_deg = self.ctrl.pos_set
            self._state.vel_set_deg_s = self.ctrl.vel_set
            self._state.acc_set_deg_s2 = self.ctrl.acc_set
            self._state.torque_nm = self.ctrl.torque_set

            self._state.kp = self.ctrl.Kp
            self._state.kd = self.ctrl.Kd
            self._state.max_vel = self.ctrl.max_vel

            if self._state.in_estop:
                self._state.status_message = "ESTOP"
            elif not self._state.connected:
                self._state.status_message = "Chưa kết nối ODrive"
            elif not self._state.closed_loop:
                self._state.status_message = "IDLE — bấm Closed Loop để vào CLOSED_LOOP_CONTROL"
            elif not self._state.is_offset:
                self._state.status_message = "CLOSED_LOOP_CONTROL — bấm Set Offset để ghi home"
            else:
                self._state.status_message = "READY — bấm Move để chạy"