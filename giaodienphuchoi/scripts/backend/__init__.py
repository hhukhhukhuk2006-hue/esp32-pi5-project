"""Backend mới cho giaodienphuchoi — dựa trên ODESC pattern.

Kiến trúc:
    - trajectory.py           : 5 loại trajectory (Trap/Cubic/Quintic/Spline/Sinusoidal)
    - kinematic_calculate.py  : SavGol filter cho acc/jerk estimation
    - single_joint_controller : SingleJointController — chạy 1 khớp (knee) với
                                full pipeline ODESC (trajectory → SavGol → CTC → torque)
                                qua ESP32 serial bridge.
    - session_manager.py      : SessionManager — coordinator giữa GUI (mainscreen) và
                                SingleJointController. Cung cấp API cũ mà mainscreen
                                đang dùng (UnifiedBackend API) nhưng chạy single-axis.

Pattern tham khảo: test/ODESC_Control_GUI/Trajectory_controller.py (ODriveThread).
Điểm khác biệt:
    - ODESC gọi trực tiếp `odrive` lib → `axis.controller.input_torque`.
    - SingleJointController giao tiếp qua ESP32 serial (giống firmware bridge cũ):
      `_send_torque_to_esp32(axis, tau_motor)`.
"""