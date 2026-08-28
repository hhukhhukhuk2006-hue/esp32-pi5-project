"""example.py — Test console cho backend mới (SingleJointController + SessionManager).

Tương tự Control_GUI_Basic.py ở ODESC nhưng không có GUI matplotlib — chỉ log ra console.

Usage:
    python -m giaodienphuchoi.scripts.backend.example
hoặc:
    cd giaodienphuchoi/scripts
    python -m backend.example

Flow (giống ODESC):
    1. Kết nối ODrive
    2. Set Offset (lưu home)
    3. Enter Closed Loop
    4. Set Move (target + max_vel + Kp + Kd)
    5. Quan sát log mỗi 200ms
    Ctrl+C để dừng.
"""

import os
import sys
import time
import math

# Đảm bảo import được backend.*
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS_DIR = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

# Nếu chạy từ giaodienphuchoi/scripts/ thì dùng import ngắn
try:
    from backend.session_manager import SessionManager
    from backend.single_joint_controller import SingleJointController
except ImportError:
    # Chạy từ thư mục khác → import full path
    from giaodienphuchoi.scripts.backend.session_manager import SessionManager
    from giaodienphuchoi.scripts.backend.single_joint_controller import SingleJointController


def main():
    sm = SessionManager()
    print("=" * 60)
    print("Backend mới — ODESC pattern, 1-khớp (knee)")
    print("=" * 60)

    # ── 1. Kết nối ─────────────────────────────────────────────────────
    print("\n[1] Connecting to ODrive...")
    ok = sm.connect()
    if not ok:
        print("Không kết nối được ODrive. Kiểm tra USB.")
        return

    print(f"  connected={sm.ctrl.connected}, axis={sm.ctrl.axis}")
    print(f"  Kt={sm.ctrl.Kt:.4f} Nm/A, max_torque={sm.ctrl.max_torque:.4f} Nm")

    # ── 2. Set Offset ──────────────────────────────────────────────────
    print("\n[2] Set Offset (lưu encoder pos làm home)...")
    sm.set_offset()
    print(f"  isOffset={sm.ctrl.isOffset}, offset={sm.ctrl.offset:.4f} rev")

    # ── 3. Enter Closed Loop ───────────────────────────────────────────
    print("\n[3] Enter CLOSED_LOOP_CONTROL...")
    sm.enter_closed_loop()
    time.sleep(1.0)  # đợi ODrive settle
    print(f"  closed_loop_control={sm.ctrl.closed_loop_control}, state={sm.ctrl.get_state()}")

    # ── 4. Set Trajectory mode (mặc định quintic) ─────────────────────
    print("\n[4] Set trajectory mode = Quintic")
    sm.set_trajectory_mode("quintic")

    # ── 5. Set Move (1-nút, giống ODESC.update_ctrlElms) ──────────────
    target = float(input("\n[5] Target (deg, world frame) [-90..90]: ") or "0.0")
    max_v = float(input("    Max velocity (deg/s) [60]: ") or "60")
    Kp = float(input("    Kp [40]: ") or "40")
    Kd = float(input("    Kd [8]: ") or "8")
    print(f"  → set_move(target={target}, max_v={max_v}, Kp={Kp}, Kd={Kd})")
    sm.set_move(target, max_vel=max_v, Kp=Kp, Kd=Kd)

    # ── 6. Quan sát 10s ────────────────────────────────────────────────
    print("\n[6] Quan sát 10s (Ctrl+C để dừng)...")
    t_start = time.time()
    last_print = 0.0
    try:
        while time.time() - t_start < 10.0:
            if time.time() - last_print >= 0.2:
                state = sm.get_state()
                print(f"  t={time.time()-t_start:5.1f}s  "
                      f"q={state.pos_deg:+7.2f}°  "
                      f"qd={state.pos_set_deg:+7.2f}°  "
                      f"qd_dot={state.vel_deg_s:+7.2f}°/s  "
                      f"torque={state.torque_nm:+6.3f}Nm  "
                      f"estop={state.in_estop}")
                last_print = time.time()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n  Ctrl+C — stopping")

    # ── 7. Cleanup ─────────────────────────────────────────────────────
    print("\n[7] Stop motion + return IDLE...")
    sm.stop_motion()
    time.sleep(0.5)
    sm.return_idle()
    sm.close()
    print("Done.")


if __name__ == "__main__":
    main()