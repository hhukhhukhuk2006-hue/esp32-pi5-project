"""frontend_adapter — thin shim nối `mainscreen.py` (PyQt5 GUI) với `backend/`.

Thay thế `unified_backend.py` (đã xoá). Chỉ phục vụ **1 khớp knee** điều khiển
bằng ODrive USB — flow giống test/ODESC_Control_GUI.

Các class mà `mainscreen.py` đang dùng:
    - OdriveBackend      — wrap SessionManager, expose API cũ (send_cmd, connect, ...)
    - CTCComputer        — single-axis CTC đơn giản (chỉ để preview, không dùng để
                            gửi torque — torque tính trong SessionManager)
    - JerkTracker        — track max jerk/acc bằng SavGol filter
    - Constants          — JOINT_LIMITS_DEG, JOINT_CODES, TRAJECTORY_MODES, ...

Khi frontend gọi `send_cmd("2,j,angle")` với j != knee → silent skip + log warning.
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

# ── Path setup: cho phép import backend.* khi chạy `python GUI.py` ─────────
# `frontend_adapter.py` ở `giaodienphuchoi/scripts/backend/`, cùng package với
# session_manager / single_joint_controller — không cần thêm sys.path.
# Khi ngoài package (vd. `python mainscreen.py` ở scripts/), package import là
# relative — caller phải đảm bảo `giaodienphuchoi/scripts/` nằm trong sys.path.
from .session_manager import SessionManager, SessionState
from .kinematic_calculate import get_acc_jerk


# ─────────────────────────────────────────────────────────────────────────────
#  Constants — giữ nguyên giá trị từ unified_backend cũ
# ─────────────────────────────────────────────────────────────────────────────
JOINT_LIMITS_DEG = {
    "hip":   (-15.0,  30.0),
    "knee":  (-100.0, 0.0),
    "ankle": (-60.0,  15.0),
}
JOINT_NAMES_VI = {"hip": "hông", "knee": "đầu gối", "ankle": "cổ chân"}
JOINT_CODES    = {"hip": 0,    "knee": 1,      "ankle": 2}

# SessionManager hỗ trợ 5 mode này (trajectory.py).
TRAJECTORY_MODES = ("trapezoidal", "cubic", "quintic", "spline", "sinusoidal")
_TRAJ_ALIAS = {
    "trap": "trapezoidal",
    "trapezoidal": "trapezoidal",
    "cubic": "cubic",
    "quintic": "quintic",
    "spline": "spline",
    "sinusoidal": "sinusoidal",
    "sine": "sinusoidal",
}

FEEDBACK_BUFFER_SIZE = 2000

JOINT_PRESETS = {
    "hip":   {"kp": 40.0, "kd": 8.0,  "max_vel": 60.0, "label": "Hông",     "joint_code": 0},
    "knee":  {"kp": 40.0, "kd": 8.0,  "max_vel": 60.0, "label": "Gối",      "joint_code": 1},
    "ankle": {"kp": 3.0,  "kd": 1.0,  "max_vel": 60.0, "label": "Cổ chân",  "joint_code": 2},
}
JOINT_ALIASES = {"hip": 0, "knee": 1, "ankle": 2, "m0": 0, "m1": 1, "m2": 2}


# ─────────────────────────────────────────────────────────────────────────────
#  Data classes — tương thích unified_backend cũ cho GUI cũ
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class FeedbackPacket:
    """Multi-joint packet (mode=7) cho GUI — luôn fill 3 joint (knee ở index 1)."""
    mode: int = 0
    joint_code: int = 0
    q_set_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    q_fb_deg:  tuple[float, float, float] = (0.0, 0.0, 0.0)
    err_deg:   tuple[float, float, float] = (0.0, 0.0, 0.0)
    timestamp: float = 0.0


@dataclass
class TorqueRecord:
    """CTC single-axis preview record (GUI chỉ đọc .tau_nm[1])."""
    timestamp: float = 0.0
    q_deg:  tuple[float, float, float] = (0.0, 0.0, 0.0)
    qd_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    qdot_deg_s:  tuple[float, float, float] = (0.0, 0.0, 0.0)
    qdot_d_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    qddot_d_deg_s2: tuple[float, float, float] = (0.0, 0.0, 0.0)
    err_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tau_nm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    g_nm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    p_term_nm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    d_term_nm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    friction_nm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mode: str = "single"


# ─────────────────────────────────────────────────────────────────────────────
#  JerkTracker — real-time max jerk/acc (SavGol)
# ─────────────────────────────────────────────────────────────────────────────
class JerkTracker:
    """Theo dõi max jerk & accel bằng SavGol filter. Không phụ thuộc numpy khi
    không đủ cửa sổ (trả về 0)."""

    def __init__(self, window_size: int = 13, poly_order: int = 2):
        self.window_size = max(3, window_size | 1)  # lẻ
        self.poly_order = max(1, min(poly_order, self.window_size - 2))
        self._buf_t: deque[float] = deque(maxlen=self.window_size)
        self._buf_v: deque[float] = deque(maxlen=self.window_size)
        self._max_jerk = 0.0
        self._max_acc = 0.0

    def push(self, t: float, vel: float):
        self._buf_t.append(float(t))
        self._buf_v.append(float(vel))
        if len(self._buf_t) == self.window_size:
            self._update_stats()

    def _update_stats(self):
        try:
            import numpy as np
            t_arr = np.array(self._buf_t, dtype=float)
            v_arr = np.array(self._buf_v, dtype=float)
            _, acc, jerk = get_acc_jerk(
                t_arr, v_arr, self.window_size, self.poly_order, pos=self.window_size // 2
            )
            self._max_acc = max(self._max_acc, abs(float(acc)))
            self._max_jerk = max(self._max_jerk, abs(float(jerk)))
        except Exception:
            pass

    @property
    def max_jerk(self) -> float:
        return self._max_jerk

    @property
    def max_acc(self) -> float:
        return self._max_acc

    def reset(self):
        self._buf_t.clear()
        self._buf_v.clear()
        self._max_jerk = 0.0
        self._max_acc = 0.0


# ─────────────────────────────────────────────────────────────────────────────
#  CTCComputer — single-axis preview (chỉ dùng để vẽ plot, không gửi xuống ODrive)
# ─────────────────────────────────────────────────────────────────────────────
class CTCComputer:
    """CTC single-axis đơn giản cho GUI preview.

    Công thức (giống SingleJointController.dynamic_calculation):
        tor = Ic*(qddot_d - Kp*ep - Kd*ev) + m*g*lc*cos(q) + friction
        friction = static*sign(qdot) + coul*tanh(50*qdot) + visc*qdot
    """

    def __init__(self, mode: str = "single"):
        self.mode = mode
        self._lock_startup_t: float | None = None

        # Physic defaults (giống SingleJointController.__init__).
        self.link_mass = 7.197
        self.link_length = 0.53175
        self.center_distance = 0.295
        self.motor_inertia = 0.000643
        gear_ratio = 50.0
        self.const_inertia = 0.8236 + gear_ratio ** 2 * self.motor_inertia

        self.hanger_mass = 0.0
        self.hanger_distance = 0.0
        self.m = self.link_mass + self.hanger_mass
        self.lc = self.center_distance  # đơn giản hoá
        self.Ic = self.const_inertia

        self.gravity = 9.81
        self.static_friction = 0.092
        self.coul_friction = 0.07
        self.visc_friction = 0.00276 * gear_ratio ** 2

        self.gains = (40.0, 40.0, 40.0)  # (kp,kp,kp)
        self.gains_kd = (8.0, 8.0, 8.0)   # (kd,kd,kd)

    # ── API tương thích unified_backend cũ ─────────────────────────────────
    def set_gains(self, kp, kd):
        self.gains = tuple(kp)
        self.gains_kd = tuple(kd)

    def set_mode(self, mode: str):
        if mode in ("full", "scalar", "single"):
            self.mode = mode

    def update_prismatic(self, hip_mm: float, knee_mm: float):
        """Update shank length (knee_mm) — ảnh hưởng link mass không lớn trong
        single-axis model, giữ API tương thích cũ."""
        pass

    def set_load(self, ext_load_kg: float = 0.0):
        self.hanger_mass = ext_load_kg
        self.m = self.link_mass + self.hanger_mass
        if self.m > 0:
            self.lc = (self.center_distance * self.link_mass
                       + self.hanger_distance * self.hanger_mass) / self.m
        self.Ic = self.const_inertia + self.hanger_mass * (self.hanger_distance ** 2)

    def reset_startup(self):
        self._lock_startup_t = None

    def compute(
        self,
        q_deg: tuple[float, float, float],
        qd_deg: tuple[float, float, float],
        qdot_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0),
        qdot_d_deg_s: tuple[float, float, float] = (0.0, 0.0, 0.0),
        qddot_d_deg_s2: tuple[float, float, float] = (0.0, 0.0, 0.0),
        startup_t: float | None = None,
        smooth_startup: bool = True,
        startup_duration: float = 0.8,
    ) -> TorqueRecord:
        if startup_t is None:
            if self._lock_startup_t is None:
                self._lock_startup_t = time.perf_counter()
            startup_t = time.perf_counter() - self._lock_startup_t

        D2R = math.pi / 180.0
        # Chỉ tính knee (index 1), hip/ankle = 0.
        q = q_deg[1] * D2R
        qd = qd_deg[1] * D2R
        qdot = qdot_deg_s[1] * D2R
        qdot_d = qdot_d_deg_s[1] * D2R
        qddot_d = qddot_d_deg_s2[1] * D2R
        kp = self.gains[1]
        kd = self.gains_kd[1]

        ep = qd - q
        ev = qdot_d - qdot

        startup_w = 1.0
        if smooth_startup and startup_t < startup_duration:
            startup_w = 0.5 * (1.0 - math.cos(math.pi * startup_t / startup_duration))

        feedfwd = self.Ic * qddot_d
        p_term = self.Ic * kp * ep
        d_term = self.Ic * kd * ev
        g_term = self.m * self.gravity * self.lc * math.cos(q)
        fric_qdot = qdot
        fric = (self.static_friction * math.copysign(1.0, fric_qdot)
                + self.coul_friction * math.tanh(50.0 * fric_qdot)
                + self.visc_friction * fric_qdot)

        tau_knee = startup_w * (feedfwd + p_term + d_term) + g_term + fric

        zeros = (0.0, 0.0, 0.0)
        tau = (0.0, tau_knee, 0.0)
        g_tuple = (0.0, g_term, 0.0)
        p_tuple = (0.0, p_term, 0.0)
        d_tuple = (0.0, d_term, 0.0)
        f_tuple = (0.0, fric, 0.0)
        err = (0.0, qd_deg[1] - q_deg[1], 0.0)

        return TorqueRecord(
            timestamp=time.perf_counter(),
            q_deg=q_deg, qd_deg=qd_deg,
            qdot_deg_s=qdot_deg_s, qdot_d_deg_s=qdot_d_deg_s,
            qddot_d_deg_s2=qddot_d_deg_s2,
            err_deg=err,
            tau_nm=tau,
            g_nm=g_tuple, p_term_nm=p_tuple, d_term_nm=d_tuple,
            friction_nm=f_tuple,
            mode="single",
        )


# ─────────────────────────────────────────────────────────────────────────────
#  OdriveBackend — facade 1-knee ODrive thay cho UnifiedBackend cũ
# ─────────────────────────────────────────────────────────────────────────────
class OdriveBackend:
    """Backend mới — wrap SessionManager, expose API cũ mà `mainscreen.py` đang
    gọi. Chỉ điều khiển **1 khớp knee**. Hip/Ankle commands → silent skip.
    """

    def __init__(
        self,
        trajectory_mode: str = "quintic",
        default_load_kg: float = 0.0,
        default_max_torque: float = 12.0,
    ):
        self._sess = SessionManager()
        self._serial_port = "ODrive USB"   # display name
        self._baudrate = None
        self._closed_loop = False
        self._offset_done = False

        # Last world target cho start_motion() flow.
        self._last_world_target: tuple[float, float, float] | None = None

        # Feedback buffer (giống UnifiedBackend).
        self._buf_lock = threading.Lock()
        self._feedback_buf: deque[FeedbackPacket] = deque(maxlen=FEEDBACK_BUFFER_SIZE)
        self.last_feedback: Optional[FeedbackPacket] = None

        self._sess.set_trajectory_mode(trajectory_mode)
        try:
            self._sess.set_load_params(
                ext_load=default_load_kg,
                hanger_distance=0.0,
                static_friction=0.092,
                coul_friction=0.07,
                visc_friction=0.00276 * 50.0 ** 2,
                max_torque=default_max_torque,
            )
        except Exception as e:
            print(f"[OdriveBackend] set_load_params lỗi: {e}")

    # ── Properties tương thích cũ ──────────────────────────────────────────
    @property
    def serial_port(self) -> str:
        return self._serial_port

    @property
    def baudrate(self):
        return self._baudrate

    @property
    def is_connected(self) -> bool:
        return self._sess.is_connected

    @property
    def status_message(self) -> str:
        try:
            return self._sess.get_state().status_message
        except Exception:
            return ""

    @property
    def error_flag(self) -> bool:
        try:
            return self._sess.get_state().error
        except Exception:
            return False

    # ── Lifecycle ─────────────────────────────────────────────────────────
    def connect(self) -> bool:
        ok = self._sess.connect()
        if ok:
            print(f"[OdriveBackend] ✓ Đã kết nối ODrive (knee-only).")
        else:
            print(f"[OdriveBackend] ✗ Không tìm thấy ODrive — chạy offline mode.")
        return bool(ok)

    def close(self):
        print(f"[OdriveBackend] close() — stop thread")
        self._sess.close()

    # ── Send/receive (Arduino-protocol cũ) ────────────────────────────────
    _SEND_CMD_COUNT = 0

    def send_cmd(self, cmd: str):
        """Parse `"<mode,j,p1,p2,...>"` tương thích GUI cũ. Khớp duy nhất = knee."""
        OdriveBackend._SEND_CMD_COUNT += 1
        if OdriveBackend._SEND_CMD_COUNT <= 10 or OdriveBackend._SEND_CMD_COUNT % 50 == 0:
            print(f"[OdriveBackend] send_cmd #{OdriveBackend._SEND_CMD_COUNT}: {cmd!r}")
        if not self.is_connected:
            return
        cmd = (cmd or "").strip().lstrip("<").rstrip(">")
        if not cmd:
            return
        try:
            parts = [p.strip() for p in cmd.split(",")]
            mode = int(parts[0])
        except (ValueError, IndexError):
            print(f"[OdriveBackend] cmd không hợp lệ: {cmd}")
            return

        try:
            if mode == 1:
                self._cmd_start()
            elif mode == 0:
                self._cmd_stop()
            elif mode == 2:
                joint = int(parts[1]); angle = float(parts[2])
                self._cmd_const_angle(joint, angle)
            elif mode == 5:
                # Cũ: parts = [mode, P, I, D]. CTC không tích phân → dùng Kp=p, Kd=d.
                p = float(parts[1]); d = float(parts[3])
                self._cmd_pid(p, d)
            else:
                print(f"[OdriveBackend] mode {mode} chưa hỗ trợ")
        except Exception as e:
            print(f"[OdriveBackend] Lỗi xử lý cmd mode={mode}: {e}")

    _RECV_PKT_COUNT = 0

    def receive_packet(self) -> Optional[FeedbackPacket]:
        """Trả về multi-joint packet (mode=7) — knee ở index 1, hip/ankle = 0."""
        OdriveBackend._RECV_PKT_COUNT += 1
        with self._buf_lock:
            if self._feedback_buf:
                return self._feedback_buf.popleft()
        # Nếu buffer rỗng, build 1 packet mới từ state hiện tại.
        state = self._sess.get_state()
        if not state.connected:
            return None
        now = time.perf_counter()
        pos   = (0.0, state.pos_deg,    0.0)
        setp  = (0.0, state.pos_set_deg, 0.0)
        err_t = (0.0, setp[1] - pos[1],  0.0)
        pkt = FeedbackPacket(
            mode=7,
            joint_code=-1,
            q_set_deg=setp,
            q_fb_deg=pos,
            err_deg=err_t,
            timestamp=now,
        )
        self.last_feedback = pkt
        # Log 10 packet đầu (debug dễ), sau đó mỗi 50 packet (~10s @ 5Hz)
        n = OdriveBackend._RECV_PKT_COUNT
        if n <= 10 or n % 50 == 0:
            print(f"[OdriveBackend] recv #{n}: pos={pos[1]:7.2f}°, set={setp[1]:7.2f}°, "
                  f"err={err_t[1]:+6.2f}°, τ={state.torque_nm:+6.2f}Nm, "
                  f"cl={state.closed_loop}, off={state.is_offset}, "
                  f"v={state.vel_deg_s:+6.2f}°/s, estop={state.in_estop}")
        return pkt

    # ── High-level API cho GUI ────────────────────────────────────────────
    def set_joint_target_deg(self, hip: float, knee: float, ankle: float):
        if not self.is_connected:
            return
        if abs(hip) > 0.01:
            print(f"[OdriveBackend] HIP target bị bỏ (chỉ hỗ trợ KNEE).")
        if abs(ankle) > 0.01:
            print(f"[OdriveBackend] ANKLE target bị bỏ (chỉ hỗ trợ KNEE).")
        self._cmd_const_angle(1, knee)

    def set_gains(self, kp, kd):
        """kp, kd = (kpkp, kpkp, kpkp), (kdkd, kdkd, kdkd). SessionManager là
        single-axis nên lấy index 1."""
        if not self.is_connected:
            return
        try:
            self._sess._state.kp = kp[1]
            self._sess._state.kd = kd[1]
            self._sess.ctrl.Kp = kp[1]
            self._sess.ctrl.Kd = kd[1]
        except Exception as e:
            print(f"[OdriveBackend] set_gains lỗi: {e}")

    def set_max_vel(self, vmax_deg_s: float):
        if not self.is_connected:
            return
        try:
            self._sess.ctrl.max_vel = vmax_deg_s
            self._sess._state.max_vel = vmax_deg_s
        except Exception as e:
            print(f"[OdriveBackend] set_max_vel lỗi: {e}")

    def set_prismatic(self, hip_mm: float, knee_mm: float):
        """Map thigh/shank (mm) → mass distribution cho gravity comp.

        Args:
            hip_mm:   thigh length (mm) — chỉ để tương thích, không dùng trực tiếp
            knee_mm:  shank length (mm) — lever distance từ knee joint → COM

        Công thức đơn giản:
            hanger_distance = knee_mm / 2 / 1000  (COM ở giữa shank)
            m = link_mass (fixed 7.2 kg) — không có hanger, không có ext_load

        Nếu sau này có payload, gọi thêm set_load_params(ext_load_kg, ...).
        """
        if not self.is_connected:
            return
        try:
            hanger_distance = max(0.05, min(0.5, knee_mm / 2000.0))   # COM ở giữa shank
            # Giữ ext_load = 0 ở đây (chưa có UI cho trọng lượng bệnh nhân)
            ext_load_kg = 0.0
            # CTC torque peak với payload 0 + lever ~0.2m = m*g*l ≈ 7.2*9.81*0.2 ≈ 14Nm
            # Đặt max_torque = 25 Nm (ODrive S1 limit) để có margin CTC dương.
            max_torque_nm = 25
            self._sess.set_load_params(
                ext_load=ext_load_kg,
                hanger_distance=hanger_distance,
                static_friction=0.092,
                coul_friction=0.07,
                visc_friction=0.00276 * 50.0 ** 2,
                max_torque=max_torque_nm,
            )
            print(f"[OdriveBackend] set_prismatic(hip={hip_mm}mm, knee={knee_mm}mm) "
                  f"→ hanger={hanger_distance*1000:.1f}mm, max_torque={max_torque_nm:.1f}Nm")
        except Exception as e:
            print(f"[OdriveBackend] set_prismatic lỗi: {e}")

    def set_trajectory_mode(self, mode: str):
        norm = _TRAJ_ALIAS.get((mode or "").strip().lower(), "quintic")
        self._sess.set_trajectory_mode(norm)

    def set_offset(self) -> bool:
        if not self.is_connected:
            return False
        try:
            self._sess.set_offset()
            self._offset_done = True
            return True
        except Exception as e:
            print(f"[OdriveBackend] set_offset lỗi: {e}")
            return False

    def enter_closed_loop(self) -> bool:
        if not self.is_connected:
            return False
        try:
            self._sess.enter_closed_loop()
            self._closed_loop = True
            return True
        except Exception as e:
            print(f"[OdriveBackend] enter_closed_loop lỗi: {e}")
            return False

    def return_idle(self) -> bool:
        try:
            self._sess.return_idle()
            self._closed_loop = False
            return True
        except Exception as e:
            print(f"[OdriveBackend] return_idle lỗi: {e}")
            return False

    def clear_error(self) -> bool:
        """Clear ODrive axis errors + reset SessionManager.error flag."""
        try:
            self._sess.clear_error()
            return True
        except Exception as e:
            print(f"[OdriveBackend] clear_error lỗi: {e}")
            return False

    def reset(self) -> bool:
        try:
            self._sess.reset()
            self._offset_done = False
            self._closed_loop = False
            return True
        except Exception as e:
            print(f"[OdriveBackend] reset lỗi: {e}")
            return False

    def go_home(self, q_max_deg: float = 90.0) -> bool:
        """Set target = start_pos (home world-frame)."""
        if not self.is_connected:
            return False
        try:
            self._sess.set_move(self._sess.ctrl.start_pos, max_vel=q_max_deg)
            return True
        except Exception as e:
            print(f"[OdriveBackend] go_home lỗi: {e}")
            return False

    def emergency_stop(self):
        try:
            self._sess.estop()
        except Exception as e:
            print(f"[OdriveBackend] estop lỗi: {e}")

    def start_motion(self):
        """Start motion với target đã set ở Confirm (start_pos = home)."""
        if not self.is_connected:
            print(f"[OdriveBackend] start_motion: not connected, skip")
            return
        if not self._closed_loop:
            print(f"[OdriveBackend] start_motion: not closed_loop, skip")
            return
        try:
            state = self._sess.get_state()
            target = state.pos_set_deg or self._sess.ctrl.start_pos
            print(f"[OdriveBackend] start_motion → set_move(target={target}°, "
                  f"Kp={self._sess.ctrl.Kp}, Kd={self._sess.ctrl.Kd}, "
                  f"vmax={self._sess.ctrl.max_vel})")
            self._sess.set_move(
                target_deg=target,
                Kp=self._sess.ctrl.Kp,
                Kd=self._sess.ctrl.Kd,
                max_vel=self._sess.ctrl.max_vel,
            )
        except Exception as e:
            print(f"[OdriveBackend] start_motion lỗi: {e}")

    def stop_motion(self):
        try:
            self._sess.stop_motion()
        except Exception as e:
            print(f"[OdriveBackend] stop_motion lỗi: {e}")

    def get_state_snapshot(self) -> dict:
        state = self._sess.get_state()
        return {
            "connected": state.connected,
            "esp32_ready": False,
            "closed_loop": state.closed_loop,
            "isOffset": state.is_offset,
            "motion_armed": False,
            "in_estop": state.in_estop,
            "pos_deg": (0.0, state.pos_deg, 0.0),
            "pos_set_deg": (0.0, state.pos_set_deg, 0.0),
            "vel_deg_s": (0.0, state.vel_deg_s, 0.0),
            "torque_nm": (0.0, state.torque_nm, 0.0),
            "kp": (0.0, state.kp, 0.0),
            "kd": (0.0, state.kd, 0.0),
            "mode": "single",
            "status": state.status_message,
        }

    # ── Command handlers (Arduino-protocol mapping) ───────────────────────
    def _cmd_start(self):
        self._sess.set_move(self._sess.ctrl.start_pos)

    def _cmd_stop(self):
        self._sess.stop_motion()

    def _cmd_const_angle(self, joint_code: int, angle_deg: float):
        if joint_code != 1:
            print(f"[OdriveBackend] joint_code={joint_code} bị bỏ (chỉ hỗ trợ KNEE=1).")
            return
        print(f"[OdriveBackend] _cmd_const_angle: KNEE target = {angle_deg:.2f}°")
        self._last_world_target = (0.0, angle_deg, 0.0)
        self._sess.set_move(angle_deg)

    def _cmd_pid(self, p: float, _d_unused: float):
        """Mode 5 cũ: P/D pair. GUI cũ truyền P, _, D ở parts[1..3]."""
        self.set_gains((p, p, p), (8.0, 8.0, 8.0))


# ─────────────────────────────────────────────────────────────────────────────
#  Backward-compat aliases (cũ dùng tên này)
# ─────────────────────────────────────────────────────────────────────────────
EmbeddedBackend = OdriveBackend
