"""Mainscreen — phiên bản ADAPTED từ LLRR_app.

Giữ nguyên 100% giao diện (UiScripts/*.py, popups.py không đổi).
Chỉ thay thế phần backend:
    - ard_connect()          → self.backend.connect()
    - sendToArduino(s)       → self.backend.send_cmd(s)
    - recvFromArduino()      → self.backend.receive_packet()
    - self.ard               → self.backend.is_connected
    - calibration (10 switch) → self.backend.set_offset() + enter_closed_loop()
    - simulation timer       → dùng EmbeddedBackend + ct.session_manager

Cấu trúc class giữ nguyên:
    UserMainScreen
        └── DoctorPatient  (thêm patient info)
        └── DoctorDev      (thêm customable exercises)
            ├── PatientMainScreen  (DoctorPatient)
            ├── DoctorMainScreen   (DoctorPatient, DoctorDev)
            └── DevMainScreen      (DoctorDev)
"""

import os
import sys
import json
import re
import time
import math
import copy
import csv
import threading

from PyQt5.QtGui import (QRegExpValidator, QFont)
from PyQt5.QtCore import (QRegExp, QDate, QTime, QTimer, pyqtSignal)
from PyQt5.QtWidgets import (QFrame, QMainWindow, QMessageBox, QTableWidgetItem)

from UiScripts.dev_ui import Ui_DevScreen
from UiScripts.doctor_ui import Ui_DoctorScreen
from UiScripts.patient_ui import Ui_PatientScreen
from popups import (AccountManagerDialog, SessionDialog, ReportDialog,
                    AccountDialog, SetAngleDialog, SetSineDialog)

# ── Backend adapter (mới) — chỉ 1 khớp knee, ODrive USB ─────────────────────
from backend.frontend_adapter import (
    OdriveBackend as EmbeddedBackend,
    JOINT_LIMITS_DEG, JOINT_NAMES_VI, JOINT_CODES,
    JerkTracker, TRAJECTORY_MODES,
    CTCComputer,
)


# ─────────────────────────────────────────────────────────────────────────────
#  Hằng số chung
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_COM_PORT = "COM21"
DEFAULT_BAUD = 115200

# Giới hạn khớp — dùng cho validation input (giống LLRR).
DEFAULT_LIMIT_RANGES = {
    'hip':   (-15.0,  30.0),
    'knee':  (-100.0, 0.0),
    'ankle': (-60.0,  15.0),
    'thigh': (39.5,   45.5),     # cm — physical link length range
    'shank': (35.0,   45.5),     # cm
}


class UserMainScreen(QMainWindow):
    """Base class cho mọi role. Phần lớn giống LLRR — chỉ phần backend đổi."""

    signal_session = pyqtSignal(dict)
    signal_stop = pyqtSignal()

    def __init__(self, dirs, id, parent=None):
        super().__init__(parent)
        self.ui.setupUi(self)

        self.user_id = id
        self.declare_paths_and_vars(dirs)

        # ── Khởi tạo backend (thay cho ard_connect) ───────────────────────
        self.backend = EmbeddedBackend(
            trajectory_mode="quintic",
            default_load_kg=0.0,
            default_max_torque=12.0,
        )
        self.ctc = CTCComputer(mode="single")  # single-axis preview
        self.jerk_tracker = JerkTracker(window_size=13, poly_order=2)

        # ── Wire UI buttons (giống LLRR) ──────────────────────────────────
        self.ui.button_sessionCreate.clicked.connect(self.session_create)
        self.ui.text_sessionTitle.returnPressed.connect(self.session_retitle)
        self.ui.button_sessionEdit.clicked.connect(self.session_edit)
        self.ui.button_sessionSave.clicked.connect(self.session_save_report)
        self.ui.button_setCycles.clicked.connect(self.set_cycles)
        self.ui.button_setTimer.clicked.connect(self.set_timer)
        self.ui.button_confirm.clicked.connect(self.set_confirm)
        self.ui.button_start.clicked.connect(self.set_start)
        self.ui.button_stop.clicked.connect(self.set_stop)
        self.ui.button_account.clicked.connect(self.account)
        self.ui.button_logout.clicked.connect(self.logout)

        # ── Default cho self.exercise / self.exercise_details ──────────────
        # Hai attr này được set khi user chạy flow cũ (confirm_sin / confirm_cyc).
        # Với flow mới (đơn khớp knee qua set_knee_target_single), user có thể
        # bấm Start mà không đi qua confirm_sin/confirm_cyc, nên ta cần default
        # để set_start() không AttributeError.
        # ── Default cho self.exercise / self.exercise_details ──────────────
        # Hai attr này được set khi user chạy flow cũ (confirm_sin / confirm_cyc).
        # Với flow mới (đơn khớp knee qua set_knee_target_single), user có thể
        # bấm Start mà không đi qua confirm_sin/confirm_cyc, nên ta cần default
        # để set_start() không AttributeError.
        self.exercise = 'Chạy đơn khớp knee'
        self.exercise_details = ''

        # ── Feedback buffer — khởi tạo NGAY trong __init__ để data_receive_timer
        # (được start tự động sau khi connect OK) có thể poll mà không AttributeError,
        # ngay cả khi user chưa bấm Confirm. set_confirm() sẽ reset lại nó.
        self.feedback_data = [
            {'name': 'hip',   'error': [], 'q_set': [], 'q_fb': []},
            {'name': 'knee',  'error': [], 'q_set': [], 'q_fb': []},
            {'name': 'ankle', 'error': [], 'q_set': [], 'q_fb': []},
        ]

        # Limit inputs (giống LLRR)
        regex = QRegExp("^[0-9]{1,3}\\.[0-9]{1,2}$")
        self.validator = QRegExpValidator(regex, self.ui.centralwidget)
        self.ui.text_weight.setValidator(self.validator)
        self.ui.text_height.setValidator(self.validator)
        self.ui.text_thigh.setValidator(self.validator)
        self.ui.text_shank.setValidator(self.validator)
        self.ui.text_cycles.setValidator(self.validator)

        self.signal_stop.connect(self.control_stop)

        # ── Timer poll feedback (thay cho data_receive_timer) ─────────────
        self.run_timer = QTimer(self)
        self.run_timer.timeout.connect(self.control_stop)
        self.run_timer.setSingleShot(True)

        self.data_receive_timer = QTimer(self)
        self.data_receive_timer.timeout.connect(self.data_receive)

        # ── Connect backend khi mở window (lazy, có thể bỏ qua nếu chưa cắm ESP32) ──
        QTimer.singleShot(500, self.try_connect_backend)

        # Initial UI state
        self.limit_ranges = dict(DEFAULT_LIMIT_RANGES)

        # ── Hardcode cho single-joint knee ─────────────────────────────────
        # Bài tập duy nhất: kéo knee tới góc cố định (mode 2 const-angle).
        # UI dùng SetAngleDialog → self.set_angle_exercise() sẽ set self.mode = 2.
        self.joint = 'knee'
        self.mode = 2

        # ── Flow mới: KHÔNG có nút Dừng — disable vĩnh viễn ngay từ đầu ──
        self.ui.button_stop.setEnabled(False)
        self.ui.button_stop.setVisible(False)

        # ── Flow mới: KHÔNG cần cycles/timer để bật Confirm. Bật Confirm
        # ngay khi mở app để user có thể vào CLOSED_LOOP + set_offset trước
        # khi bấm Bắt đầu (motor chạy liên tục, không có timeout/stop). ──
        self.ui.button_confirm.setEnabled(True)
        self.ui.button_start.setEnabled(False)

        # ── Nút "Chế độ" đặt cạnh nút Bắt đầu — chế độ Kp=0 (zero
        # torque), giống Bắt đầu nhưng moment = 0 (motor "free", chỉ giữ
        # bằng ma sát + trọng lực). ──
        from PyQt5.QtWidgets import QPushButton
        from PyQt5.QtGui import QFont
        self.ui.button_mode = QPushButton(self.ui.frame_set)
        self.ui.button_mode.setGeometry(140, 330, 100, 100)
        font = QFont()
        font.setFamily('MS Shell Dlg 2')
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.ui.button_mode.setFont(font)
        self.ui.button_mode.setStyleSheet(
            "#button_mode{\n"
            "    color: rgb(255, 255, 255);\n"
            "    border: 0px;\n"
            "    background-color: rgb(255, 152, 0);\n"   # cam để phân biệt xanh Bắt đầu
            "}\n"
            "#button_mode:hover{\n"
            "    background-color: rgb(245, 178, 66);\n"
            "}\n"
            "#button_mode:pressed{\n"
            "    background-color: rgb(198, 119, 0);\n"
            "}\n"
            "#button_mode:disabled{\n"
            "    color: rgb(208, 217, 222);\n"
            "    border: 2px solid rgb(208, 217, 222);\n"
            "    background-color: none;\n"
            "}"
        )
        self.ui.button_mode.setText('CHẾ ĐỘ')
        self.ui.button_mode.setObjectName('button_mode')
        self.ui.button_mode.setEnabled(False)
        self.ui.button_mode.clicked.connect(self.set_mode)
        # Cờ: đang ở chế độ nào (False = Kp mặc định, True = Kp=0).
        self._mode_zero_kp = False

        # ── Nút "Reset" — đặt cạnh nút Chế độ. Zero moment + về IDLE +
        # clear ODrive errors. Dùng khi muốn hệ thống trở về trạng thái
        # nghỉ mà không cần reboot (giá trị torque feedback sẽ về 0). ──
        self.ui.button_reset = QPushButton(self.ui.frame_set)
        self.ui.button_reset.setGeometry(240, 330, 100, 100)
        font_r = QFont()
        font_r.setFamily('MS Shell Dlg 2')
        font_r.setPointSize(11)
        font_r.setBold(True)
        font_r.setWeight(75)
        self.ui.button_reset.setFont(font_r)
        self.ui.button_reset.setStyleSheet(
            "#button_reset{\n"
            "    color: rgb(255, 255, 255);\n"
            "    border: 0px;\n"
            "    background-color: rgb(96, 125, 139);\n"   # xám-xanh để phân biệt cam (mode)
            "}\n"
            "#button_reset:hover{\n"
            "    background-color: rgb(120, 144, 156);\n"
            "}\n"
            "#button_reset:pressed{\n"
            "    background-color: rgb(69, 90, 100);\n"
            "}\n"
            "#button_reset:disabled{\n"
            "    color: rgb(208, 217, 222);\n"
            "    border: 2px solid rgb(208, 217, 222);\n"
            "    background-color: none;\n"
            "}"
        )
        self.ui.button_reset.setText('RESET')
        self.ui.button_reset.setObjectName('button_reset')
        self.ui.button_reset.setEnabled(False)
        self.ui.button_reset.clicked.connect(self.set_reset)

    # ─────────────────────────────────────────────────────────────────────
    #  Setup
    # ─────────────────────────────────────────────────────────────────────
    def declare_paths_and_vars(self, dirs):
        path_assets, path_icons, path_database, path_dev, path_users_file, user, path_root = dirs
        self.path_assets = path_assets
        self.path_icons = path_icons
        self.path_database = path_database
        self.path_dev = path_dev
        self.path_users_file = path_users_file
        self.path_root = path_root
        self.manager_id = user['manager_id']
        self.manager_name = user['manager']
        self.password = user['password']

        self.dict_data = {}
        self.listID = []
        self.session_info = {}
        self.time_stop = None
        self.time_start = None
        self.session_data = {}
        self.login_screen = None
        self.control_data = {}

        # Thay self.ard bằng self.backend.is_connected (property)
        self.session_count = 0

        # ── Default an toàn cho flow mới (đơn khớp knee) ───────────────
        # total_seconds được set bởi set_cycles() / set_timer() ở flow cũ.
        # Flow mới (single-knee) có thể bấm Start mà không set cycles/timer,
        # nên cần default để set_start() không AttributeError.
        #   - total_seconds = 0   → run_timer.start(0)  → timeout ngay → motion dừng
        #   - 30 giây là thời gian hợp lý cho 1 phiên đơn khớp
        self.total_seconds = 30

    def try_connect_backend(self):
        """Thử kết nối embedded computer. Không block UI nếu fail."""
        ok = self.backend.connect()
        if ok:
            self.status_message(f"Đã kết nối {self.backend.serial_port}")
            print(f"[GUI] try_connect_backend: OK → start data_receive_timer @ 5Hz")
            # Poll feedback ngay từ đầu — để pos/vel/τ luôn cập nhật trên UI,
            # không cần đợi bấm Start (giống pattern NAPF Control_GUI_Basic.py).
            self.data_receive_timer.start(100)
        else:
            self.status_message("Chưa kết nối — bấm 'Kết nối' để thử lại")
            print(f"[GUI] try_connect_backend: FAIL — backend.is_connected=False")

    def status_message(self, msg: str):
        """Hiển thị status lên UI — dùng statusBar hoặc label nếu có."""
        # Ưu tiên statusBar nếu Qt MainWindow có.
        try:
            self.statusBar().showMessage(msg, 5000)
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────────────
    #  Embedded backend glue — các method cũ được thay bằng wrapper
    # ─────────────────────────────────────────────────────────────────────
    @property
    def ard(self) -> bool:
        """Backward-compat alias — code cũ kiểm tra `if self.ard:`."""
        return self.backend.is_connected

    # ── Account ───────────────────────────────────────────────────────────
    def account(self):
        self.account_dialog = AccountDialog(self)
        self.account_dialog.ui.text_accountID.setText(self.user_id)
        self.account_dialog.password.connect(self.account_password_update)
        self.account_dialog.show()

    def account_password_update(self, password):
        with open(self.path_users_file, 'r') as f:
            dict_users = json.load(f)
            user = dict_users[self.user_id]
            f.close()
        user.update({'password': password})
        dict_users.update({self.user_id: user})
        with open(self.path_users_file, 'w') as f:
            json.dump(dict_users, f)
            f.close()
        QMessageBox.information(self, 'Thông tin', 'Bạn đã đặt lại mật khẩu thành công')

    def logout(self):
        self.backend.close()
        if self.login_screen:
            self.login_screen.showMaximized()
        self.close()

    # ─────────────────────────────────────────────────────────────────────
    #  Session block — giống LLRR 100%, không đổi
    # ─────────────────────────────────────────────────────────────────────
    def session_create(self):
        prefix = 'session'
        id_patient = self.ui.label_dispID.text()
        session_date = str(QDate.currentDate().toPyDate())
        reports = [r for r in os.listdir(self.path_reports) if r.endswith('.txt')]
        session_num = str(len(reports))
        if int(session_num) < 10:
            session_num = '0' + session_num
        session_title = self.ui.text_sessionTitle.text()
        filename = id_patient + '_' + prefix + session_num + '_' + session_date
        if session_title == '':
            session_title = filename
            self.ui.text_sessionTitle.setText(session_title)
            self.ui.text_sessionTitle.setCursorPosition(0)
        session_patient = self.dict_data[id_patient]['name']
        session_time = QTime.currentTime().toString('hh:mm:ss')

        self.session_info = {'title': session_title,
                             'date': session_date, 'time': session_time,
                             'patient': session_patient, 'patient_id': id_patient,
                             'doctor': self.manager_name, 'doctor_id': self.manager_id,
                             'description': '',
                             'path_report': self.path_reports,
                             'filename': filename}

        self.specs = []
        self.ui.button_sessionEdit.setEnabled(True)
        self.ui.frame_exercises.setEnabled(True)

    def session_retitle(self):
        session_title = self.ui.text_sessionTitle.text()
        self.session_info.update({'title': session_title})
        self.ui.text_sessionTitle.setCursorPosition(0)

    def session_edit(self):
        self.session_dialog = SessionDialog(self)
        self.session_dialog.sessionSaved.connect(self.session_edit_save)
        if self.session_info:
            self.session_dialog.session_update(self.session_info)
            self.date_set(self.session_dialog.ui.text_sessionDate, self.session_info['date'])
        self.session_dialog.show()

    def session_edit_save(self, session_info):
        for field in list(session_info.keys()):
            self.session_info.update({field: session_info[field]})
        self.ui.text_sessionTitle.setText(self.session_info['title'])

    def session_save_report(self):
        id_ = self.ui.label_dispID.text()
        self.report_dialog = ReportDialog(self)
        self.report_dialog.session_update(self.session_info)
        self.report_dialog.reports_update_list()
        if not self.time_stop:
            self.time_stop = QTime.currentTime()
        self.report_dialog.ui.text_sessionEndTime.setTime(self.time_stop)
        self.report_dialog.report_update(self.session_data)

        if re.split(r'\d+', id_)[0] != 'KTV':
            self.path_trainning_data = os.path.join(self.path_reports, f'{id_}_total_training_data.json')
            with open(self.path_trainning_data, 'r') as f:
                self.training_data = json.load(f)
                f.close()
            self.report_dialog.report_chart(self.training_data)
            self.report_dialog.log.connect(self.log_training_data)
        else:
            self.report_dialog.ui.table_exercises.setColumnCount(6)
            self.report_dialog.ui.table_exercises.setHorizontalHeaderItem(3, QTableWidgetItem('T.tin tải'))
            self.report_dialog.ui.table_exercises.setHorizontalHeaderItem(4, QTableWidgetItem('H.số PID'))
            self.report_dialog.ui.table_exercises.setHorizontalHeaderItem(5, QTableWidgetItem('Dải sai số(độ)'))
            self.report_dialog.ui.table_exercises.horizontalHeader().setFont(QFont("MS Shell Dlg 2", 11))

            for rowPosition in range(self.report_dialog.ui.table_exercises.rowCount()):
                row_data = self.specs[rowPosition]
                weight = row_data['weight']
                thigh = row_data['thigh']
                shank = row_data['shank']
                load_specs = f'K.lượng: {weight}kg, c.dài đùi: {thigh}cm, c.dài cẳng chân: {shank}cm'
                self.report_dialog.ui.table_exercises.setItem(rowPosition, 3, QTableWidgetItem(load_specs))

                p = row_data['P']
                i = row_data['I']
                d = row_data['D']
                pid_specs = f'P: {p}, I: {i}, D: {d}'
                self.report_dialog.ui.table_exercises.setItem(rowPosition, 4, QTableWidgetItem(pid_specs))

                error_range = 0
                row_feedback_data = row_data['feedback_data']
                if row_data['mode'] == 3:
                    error_range = '{} đến {}'.format(
                        min(row_feedback_data[row_data['joint_code']]['error']),
                        max(row_feedback_data[row_data['joint_code']]['error']))
                if row_data['mode'] == 4:
                    error_range_hip = '{} đến {}'.format(min(row_feedback_data[0]['error']), max(row_feedback_data[0]['error']))
                    error_range_knee = '{} đến {}'.format(min(row_feedback_data[1]['error']), max(row_feedback_data[1]['error']))
                    error_range_ankle = '{} đến {}'.format(min(row_feedback_data[2]['error']), max(row_feedback_data[2]['error']))
                    error_range = f'hông: {error_range_hip}, đầu gối: {error_range_knee}, cổ chân: {error_range_ankle}'
                self.report_dialog.ui.table_exercises.setItem(rowPosition, 5, QTableWidgetItem(error_range))
                self.report_dialog.report_error_chart(row_feedback_data, rowPosition)
            self.report_dialog.ui.table_exercises.resizeRowsToContents()
            self.report_dialog.log.connect(self.log_session_data)
        self.report_dialog.show()

    def session_set_text(self, var, text):
        var.setFrameShape(QFrame.Shape.StyledPanel)
        var.setStyleSheet('color: rgb(0, 0, 0); background-color: rgb(255, 255, 255);')
        var.setText(text)

    def session_reset_text(self, var):
        var.setFrameShape(QFrame.Shape.NoFrame)
        var.setStyleSheet('color: rgb(0, 0, 0); background-color: rgba(255, 255, 255, 0);')
        var.clear()

    def session_set_state(self, state):
        self.ui.text_sessionTitle.setEnabled(state)
        self.ui.button_sessionCreate.setEnabled(state)
        self.ui.button_sessionEdit.setEnabled(not state)
        self.ui.button_sessionSave.setEnabled(not state)

    def set_cycles(self):
        cycles = self.ui.text_cycles.text()
        if cycles not in ('', '0'):
            self.ui.label_cycles_timer.setText('Số chu kỳ:')
            self.session_set_text(self.ui.text_cycles_timer, cycles)
            self.ui.button_confirm.setEnabled(True)
            self.total_seconds = int(cycles) * int(self.cycle)
        else:
            QMessageBox.warning(self, "Cảnh báo", "Xin hãy nhập từ 1 chu kỳ trở lên")

    def set_timer(self):
        timer = self.ui.text_timer.time().toString('mm:ss')
        minutes, seconds = timer.split(':')
        if int(minutes) or int(seconds) > 0:
            self.ui.label_cycles_timer.setText('Thời gian:')
            time_str = f'{minutes} phút {seconds} giây'
            self.session_set_text(self.ui.text_cycles_timer, time_str)
            self.ui.button_confirm.setEnabled(True)
            self.total_seconds = int(seconds) + int(minutes) * 60
        else:
            QMessageBox.warning(self, "Cảnh báo", "Xin hãy đặt thời gian hơn 1 giây ")

    # ─────────────────────────────────────────────────────────────────────
    #  Exercise selection — cập nhật cho embedded computer
    # ─────────────────────────────────────────────────────────────────────
    def _motion_loop_tick(self):
        """Tick ~100Hz: hiện đang strip — chỉ giữ mode 1 (đơn khớp knee)."""
        return

    # ─────────────────────────────────────────────────────────────────────
    #  CONFIRM — gửi exercise spec xuống embedded computer
    # ─────────────────────────────────────────────────────────────────────
    def set_confirm(self):
        """Confirm + gửi data xuống embedded computer + zero-offset.

        Chỉ ARM (set_offset + enter_closed_loop) — KHÔNG cấp moment. Moment
        chỉ được cấp khi user bấm Start (set_start).
        """
        checked = True
        # KHÔNG tắt frame_exercises/frame_session — với flow mới (chọn bài
        # tập → nhập góc → Bắt đầu → chọn lại), user vẫn cần chọn bài tập
        # và tạo session sau khi calibrate.
        self.ui.button_confirm.setEnabled(False)
        self.ui.button_start.setEnabled(False)
        # KHÔNG tắt frame_set — các nút Start/Stop/Confirm nằm trong frame_set,
        # parent disabled sẽ vô hiệu luôn setEnabled(True) của con.
        # Chỉ tắt các input để user không sửa giữa lúc calibrate.
        self.ui.button_setCycles.setEnabled(False)
        self.ui.button_setTimer.setEnabled(False)
        self.ui.text_cycles.setEnabled(False)
        self.ui.text_timer.setEnabled(False)

        # Lưu target angle để set_start() dùng (chưa gửi xuống ODrive ở đây).
        try:
            self._pending_target_angle = float(self.control_data[self.joint]['angle'])
            self._pending_joint_code = int(self.control_data[self.joint]['code'])
        except Exception:
            self._pending_target_angle = None
            self._pending_joint_code = None

        # ── Nếu đã kết nối ODrive: set_prismatic + vào closed_loop (no torque) ──
        if self.ard:
            thigh_m = round(float(self.ui.text_thigh.text()) * 1e-2, 2)  # cm → m
            shank_m = round(float(self.ui.text_shank.text()) * 1e-2, 2)

            # Hip = thigh_m, Knee = shank_m (mm)
            self.backend.set_prismatic(thigh_m * 1000, shank_m * 1000)
            self.ctc.update_prismatic(thigh_m * 1000, shank_m * 1000)

            # KHÔNG gửi send_cmd("2,1,angle") ở đây — set_move() sẽ cấp moment
            # ngay (đó là 1-nút Move, không có khái niệm "arm"). Việc cấp moment
            # sẽ do set_start() xử lý.

            if self.mode != 2:
                QMessageBox.warning(self, "Cảnh báo", "Chế độ bài tập không hợp lệ")
                self.ui.button_confirm.setEnabled(True)
                self.frame_set_state(True)
                self.ui.frame_set.setEnabled(True)
                return

            # ── Nếu đã vào CLOSED_LOOP rồi → không cần set_offset/enter
            # closed_loop lại. Chỉ bật Bắt đầu để user cấp moment mới.
            if self.backend._closed_loop:
                QMessageBox.information(self, 'Thông báo',
                    'Đã ở Closed Loop — bấm Bắt đầu để chạy target mới.')
                self.ui.button_start.setEnabled(True)
                return

            # Calibrate: set_offset → enter_closed_loop (chỉ lần đầu).
            QTimer.singleShot(300, self._run_calibration_sequence)
        else:
            # ── Simulation mode: không cần ODrive, dùng trajectory giả lập ──
            QMessageBox.information(
                self, 'Thông báo',
                'Chưa kết nối embedded computer — chạy chế độ simulation offline.'
            )
            self.start_time = time.perf_counter()
            # Ở simulation: bật thẳng Start (không cần qua CLOSED_LOOP).
            self.ui.button_start.setEnabled(True)

        if checked:
            # Chuẩn bị feedback buffer — thay cho safety_switched cũ.
            self.feedback_data = [
                {'name': 'hip',   'error': [], 'q_set': [], 'q_fb': []},
                {'name': 'knee',  'error': [], 'q_set': [], 'q_fb': []},
                {'name': 'ankle', 'error': [], 'q_set': [], 'q_fb': []},
            ]

    def _run_calibration_sequence(self):
        """Calibration sequence: clear_errors → set_offset → enter_closed_loop.
        Thay cho việc chờ 10 limit switch từ Arduino.
        """
        # Clear ODrive axis/controller/encoder/motor errors trước khi
        # set_offset — nếu còn lỗi từ phiên trước, set_offset sẽ fail.
        ok_clear = self.backend.clear_error()
        if not ok_clear:
            print('[mainscreen] clear_error thất bại (có thể không có lỗi) — tiếp tục')

        ok_offset = self.backend.set_offset()
        if not ok_offset:
            QMessageBox.warning(self, 'Cảnh báo', 'Không thể set_offset — kiểm tra kết nối')
            self._restore_buttons_after_calib(False)
            return

        # Đợi 1s cho ESP32 ổn định rồi vào closed loop.
        QTimer.singleShot(1000, self._enter_closed_loop_step)

    def _enter_closed_loop_step(self):
        ok = self.backend.enter_closed_loop()
        self._restore_buttons_after_calib(ok)

    def _restore_buttons_after_calib(self, ok: bool):
        """Sau set_offset + enter_closed_loop → bật Start (chưa cấp moment).

        Quan trọng: KHÔNG tắt frame_set — Start nằm trong frame_set, parent bị
        disable thì setEnabled(True) của con bị vô hiệu.
        KHÔNG tắt frame_exercises — user cần chọn lại bài tập sau khi
        Bắt đầu (flow mới: chọn → nhập góc → Bắt đầu → có thể chọn lại).
        """
        if ok:
            QMessageBox.information(self, 'Thông báo',
                'Đã vào Closed Loop — chọn bài tập rồi bấm Bắt đầu để chạy.')
            # Confirm TẮT (đã calibrate); Start BẬT (mới là nơi moment chạy).
            self.ui.button_confirm.setEnabled(False)
            self.ui.button_start.setEnabled(True)
            # Flow mới: KHÔNG có nút Dừng — disable vĩnh viễn.
            self.ui.button_stop.setEnabled(False)
            # Tắt những control bên trong frame_set có thể gây xung đột
            # nhưng KHÔNG tắt cả frame_set (Start nằm ở đây).
            self.ui.button_setCycles.setEnabled(False)
            self.ui.button_setTimer.setEnabled(False)
            self.ui.text_cycles.setEnabled(False)
            self.ui.text_timer.setEnabled(False)
            # KHÔNG gọi frame_set_state(False) — user cần chọn lại bài tập
            # (DoctorDev) hoặc tạo session mới (KTV).
        else:
            QMessageBox.warning(self, 'Thông báo', 'Không vào được Closed Loop')
            self.ui.button_confirm.setEnabled(True)
            self.frame_set_state(True)
            self.ui.frame_set.setEnabled(True)

    # ─────────────────────────────────────────────────────────────────────
    #  START / STOP — bắt đầu / dừng motion
    # ─────────────────────────────────────────────────────────────────────
    def set_start(self):
        """Bắt đầu / CẬP NHẬT exercise — gọi set_move() với target đã lưu ở
        set_confirm (lần đầu) hoặc target mới (nếu user đổi bài tập).

        Không dừng motion: sau khi đến target, CTC vẫn giữ moment để giữ
        vị trí. User có thể chọn bài tập khác + bấm lại Bắt đầu để
        trajectory reset từ vị trí hiện tại → target mới.
        """
        checked = True
        if self.ard:
            # Chỉ ở đây mới cấp moment (set_move = 1-nút Move: traj + clock +
            # CTC torque). Nếu user chưa bấm Confirm (lỗi), fallback về
            # start_motion() cũ.
            target = getattr(self, '_pending_target_angle', None)
            jc = getattr(self, '_pending_joint_code', None)
            if target is not None and jc == 1:
                # Chỉ knee (code=1) — cấp moment.
                message = f'2,{jc},{target}'
                self.backend.send_cmd(message)
            else:
                # Fallback: nếu không phải knee mode 2, dùng start_motion cũ.
                self.backend.start_motion()
        else:
            # Simulation — đã start ở set_confirm, không làm gì thêm.
            pass

        if checked:
            self.session_count += 1
            self.session_data.update({
                self.session_count: {
                    'exercise': self.exercise,
                    'details': self.exercise_details,
                    'info': self.ui.label_cycles_timer.text() + ' ' + self.ui.text_cycles_timer.text(),
                }
            })
            self.time_start = QTime.currentTime()
            self.ui.button_confirm.setEnabled(False)
            # KHÔNG tắt frame_set_state (vì user cần chọn lại bài tập).
            # Chỉ tắt cycles/timer input (control_set_state).
            self.control_set_state(False)
            # Với flow mới: không có nút Dừng; Start vẫn BẬT để user bấm
            # lại sau khi đổi bài tập / nhập góc mới. Nút Chế độ (Kp=0)
            # cũng bật kèm để user có thể chuyển sang chế độ zero-torque.
            # Nút Reset cũng bật để user đưa hệ thống về idle/zero-torque.
            self.control_start_state(True)
            self.ui.button_mode.setEnabled(True)
            self.ui.button_reset.setEnabled(True)
            if self._mode_zero_kp:
                self.ui.button_mode.setText('VỀ BẮT ĐẦU')
            else:
                self.ui.button_mode.setText('CHẾ ĐỘ')

            # Bắt đầu nhận feedback — dùng data_receive_timer (5Hz cho UI).
            if not self.data_receive_timer.isActive():
                self.data_receive_timer.start(100)

            self.ctc.reset_startup()
            self.jerk_tracker.reset()

    def set_mode(self):
        """Chuyển chế độ Kp=0 ↔ Kp=mặc định (40). Tương tự Bắt đầu nhưng
        đổi giá trị Kp. Kd giữ nguyên. Motor sẽ "free" (chỉ giữ bằng ma sát
        + trọng lực) khi Kp=0, hoặc giữ vị trí cứng khi Kp=40.
        """
        if not getattr(self, 'ard', False):
            # Simulation mode — chỉ đổi cờ UI.
            self._mode_zero_kp = not self._mode_zero_kp
            self.ui.button_mode.setText(
                'VỀ BẮT ĐẦU' if self._mode_zero_kp else 'CHẾ ĐỘ')
            return

        target = getattr(self, '_pending_target_angle', None)
        jc = getattr(self, '_pending_joint_code', None)
        if target is None or jc != 1:
            # Chưa ở mode 2 (knee const-angle) — không áp dụng được.
            QMessageBox.warning(self, 'Cảnh báo',
                'Hãy bấm Xác nhận + Bắt đầu trước khi chuyển chế độ.')
            return

        # Toggle flag.
        self._mode_zero_kp = not self._mode_zero_kp
        if self._mode_zero_kp:
            # Kp=0 → "free"; Kd giữ nguyên để có damping nhẹ.
            self.backend.set_gains((0.0, 0.0, 0.0), (8.0, 8.0, 8.0))
            self.ui.button_mode.setText('VỀ BẮT ĐẦU')
            print('[mainscreen] mode → Kp=0 (free)')
        else:
            # Kp mặc định → 40.
            self.backend.set_gains((40.0, 40.0, 40.0), (8.0, 8.0, 8.0))
            self.ui.button_mode.setText('CHẾ ĐỘ')
            print('[mainscreen] mode → Kp=40 (default)')

        # Gọi set_move() để trajectory reset từ vị trí hiện tại về target.
        # Nếu đang ở half-trajectory thì quá trình reset này là cần thiết.
        if not getattr(self, '_mode_zero_kp', False) or True:
            # Luôn gửi lại set_move để update Kp hiện hành và reset traj.
            try:
                self.backend._sess.set_move(
                    target_deg=target,
                    Kp=self.backend._sess.ctrl.Kp,
                    Kd=self.backend._sess.ctrl.Kd,
                    max_vel=self.backend._sess.ctrl.max_vel,
                )
            except Exception as e:
                print(f'[mainscreen] set_move after mode toggle lỗi: {e}')

    def set_reset(self):
        """Zero moment + về IDLE + clear ODrive errors.

        Khác stop_motion (chỉ zero torque, giữ closed_loop): Reset đưa
        axis về state IDLE để feedback torque nằm im ở 0. Sau Reset, user
        phải bấm lại Xác nhận để vào lại closed_loop.
        """
        confirm = QMessageBox.question(
            self, 'Xác nhận Reset',
            'Đưa hệ thống về IDLE? Motor sẽ mất moment — cần Xác nhận '
            'lại trước khi chạy.',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if confirm != QMessageBox.Yes:
            return

        # 1. Zero torque trước (stop_motion vẫn giữ closed_loop).
        try:
            self.backend.stop_motion()
        except Exception as e:
            print(f'[mainscreen] reset stop_motion lỗi: {e}')

        # 2. Đưa axis về IDLE.
        try:
            self.backend.return_idle()
        except Exception as e:
            print(f'[mainscreen] reset return_idle lỗi: {e}')

        # 3. Clear ODrive errors (axis/controller/encoder/motor).
        try:
            self.backend.clear_error()
        except Exception as e:
            print(f'[mainscreen] reset clear_error lỗi: {e}')

        # 4. Reset UI: Start/Reset/Confirm — tắt Reset+Start, bật Confirm.
        self.ui.button_reset.setEnabled(False)
        self.ui.button_start.setEnabled(False)
        self.ui.button_mode.setEnabled(False)
        self.ui.button_confirm.setEnabled(True)
        self.ui.button_mode.setText('CHẾ ĐỘ')
        self._mode_zero_kp = False

        # Cho phép nhập lại thông số.
        self.ui.button_setCycles.setEnabled(True)
        self.ui.button_setTimer.setEnabled(True)
        self.ui.text_cycles.setEnabled(True)
        self.ui.text_timer.setEnabled(True)

        QMessageBox.information(self, 'Thông báo',
            'Đã reset — bấm Xác nhận để vào lại Closed Loop.')

    def set_stop(self):
        self.signal_stop.emit()

    def control_stop(self):
        """Được gọi khi run_timer timeout (cycles/timer đã hết) — hoặc từ
        các role khác (KTV, ...) khi cần dừng hẳn motion.

        Lưu ý: với flow mới (không còn nút Dừng), hàm này CHỈ lưu session
        log nếu cycles/timer đã cấu hình; KHÔNG tắt ODrive. Motion vẫn
        chạy và CTC vẫn giữ moment tại vị trí target.
        """
        # Nếu từ app chính (đã bấm Bắt đầu với cycles/timer) → log session.
        # Nếu chưa có cycles/timer (total_seconds = default 30s) thì bỏ qua.
        if not getattr(self, 'ard', False):
            # Simulation mode hoặc không có ODrive — không log.
            return

        # Vẫn lưu session data để user mở Report xem lại.
        self.time_stop = QTime.currentTime()
        self.ui.button_sessionSave.setEnabled(True)

        if re.split(r'\d+', self.ui.label_dispID.text())[0] == 'KTV':
            self.data['mode'] = self.mode
            self.data['joint_code'] = self.control_data[self.joint]['code']
            self.data['feedback_data'] = self.feedback_data
            self.specs.append(copy.deepcopy(self.data))

    # ─────────────────────────────────────────────────────────────────────
    #  Feedback receive — polling từ EmbeddedBackend (thay cho Serial read)
    # ─────────────────────────────────────────────────────────────────────
    def data_receive(self):
        """Đọc gói feedback mới nhất từ backend → cập nhật feedback_data[]."""
        if not self.ard:
            return
        pkt = self.backend.receive_packet()
        if pkt is None:
            return
        # Safe-check: feedback_data có thể chưa được tạo nếu __init__ thất bại
        # (ví dụ khi load UI chưa xong) — bỏ qua packet để tránh AttributeError.
        if not hasattr(self, 'feedback_data') or self.feedback_data is None:
            return
        # EmbeddedBackend packet luôn là mode=7 (multi-joint, q_set/q_fb/err cho cả 3)
        if pkt.mode == 7:
            for i in range(3):
                joint = self.feedback_data[i]
                joint['q_set'].append(pkt.q_set_deg[i])
                joint['q_fb'].append(pkt.q_fb_deg[i])
                joint['error'].append(pkt.err_deg[i])

    # ─────────────────────────────────────────────────────────────────────
    #  Helper giống LLRR
    # ─────────────────────────────────────────────────────────────────────
    def date_set(self, var, date):
        qdate = QDate.fromString(date, 'yyyy-MM-dd')
        var.setDisplayFormat('dd/MM/yyyy')
        var.setDate(qdate)

    def control_set_state(self, state):
        self.ui.text_cycles.setEnabled(state)
        self.ui.text_timer.setEnabled(state)
        self.ui.label_cycles.setEnabled(state)
        self.ui.label_timer.setEnabled(state)
        self.ui.button_setCycles.setEnabled(state)
        self.ui.button_setTimer.setEnabled(state)

    def control_start_state(self, state):
        self.ui.button_start.setEnabled(state)
        self.ui.button_stop.setEnabled(not state)

    def frame_set_state(self, state):
        self.ui.frame_exercises.setEnabled(state)
        self.ui.frame_session.setEnabled(state)
        self.ui.label_frameSession.setEnabled(state)


# ─────────────────────────────────────────────────────────────────────────────
#  DoctorPatient — thêm patient info (giống LLRR)
# ─────────────────────────────────────────────────────────────────────────────
class DoctorPatient(UserMainScreen):
    def __init__(self, dirs, id, parent=None):
        super().__init__(dirs, id, parent)
        self.update_dict_patient()
        self.date_reset()
        self.ui.button_male.toggled.connect(self.info_other_gender_off)
        self.ui.button_female.toggled.connect(self.info_other_gender_off)
        self.ui.button_others.toggled.connect(self.info_other_gender)
        self.ui.button_noMedcard.toggled.connect(self.info_medCard_off)
        self.ui.button_yesMedcard.toggled.connect(self.info_medCard)
        self.ui.button_infoEdit.clicked.connect(self.info_edit)
        self.ui.text_age.setValidator(QRegExpValidator(QRegExp("^[0-9]{3}$"), self.ui.text_age))

    def update_dict_patient(self):
        self.path_dict_data = os.path.join(self.path_root, 'patients-info.json')
        try:
            with open(self.path_dict_data, 'r') as f:
                self.dict_data = json.load(f); f.close()
        except FileNotFoundError:
            with open(self.path_dict_data, 'w') as f:
                json.dump(self.dict_data, f); f.close()

    def info_show(self, id, patient):
        self.ui.label_dispID.setText(id)
        self.ui.text_name.setText(patient['name'])
        self.ui.text_work.setText(patient['work'])
        self.ui.text_age.setText(patient['age'])
        gender = patient['gender']
        if gender == self.ui.button_male.text():
            self.ui.button_male.setChecked(True)
        elif gender == self.ui.button_female.text():
            self.ui.button_female.setChecked(True)
        else:
            self.ui.button_others.setChecked(True)
            self.ui.text_others.setText(gender)
            self.ui.text_others.setEnabled(False)
        medCardstate = patient['medCardstate']
        if medCardstate == self.ui.button_noMedcard.text():
            self.ui.button_noMedcard.setChecked(True)
        if medCardstate == self.ui.button_yesMedcard.text():
            self.ui.button_yesMedcard.setChecked(True)
        self.ui.text_medCardinfo.setText(patient['medCardinfo'])
        self.date_set(self.ui.date_admitDate, patient['admitDate'])
        self.date_set(self.ui.date_treatDate, patient['treatDate'])

        path_record = os.path.join(self.path_root, id, id + '_records.txt')
        with open(path_record, 'r', encoding='utf-8') as f:
            record = f.read(); f.close()
        self.ui.text_record.setPlainText(record)

        self.ui.text_weight.setText(patient['weight'])
        self.ui.text_height.setText(patient['height'])
        self.ui.text_thigh.setText(patient['thigh'])
        self.ui.text_shank.setText(patient['shank'])

        self.path_reports = os.path.join(self.path_root, id, 'reports')

        self.path_control_data = os.path.join(self.path_root, id, 'control_data.json')
        with open(self.path_control_data, 'r') as f:
            self.control_data = json.load(f); f.close()

        self.ui.button_infoEdit.setEnabled(True)
        self.ui.frame_session.setEnabled(True)
        self.ui.label_frameSession.setEnabled(True)
        self.ui.button_sessionCreate.setEnabled(True)

    def info_other_gender(self):
        if self.ui.button_others.isChecked():
            self.ui.text_others.setEnabled(True)

    def info_other_gender_off(self):
        if (self.ui.button_male.isChecked() or self.ui.button_female.isChecked()):
            self.ui.text_others.setEnabled(False)

    def info_medCard(self):
        if self.ui.button_yesMedcard.isChecked():
            self.ui.text_medCardinfo.setEnabled(True)
            self.ui.text_medCardinfo.setPlaceholderText('Nhập số BHYT...')

    def info_medCard_off(self):
        if self.ui.button_noMedcard.isChecked():
            self.ui.text_medCardinfo.setEnabled(False)
            self.ui.text_medCardinfo.setPlaceholderText('')

    def info_save(self):
        id_ = self.ui.label_dispID.text()
        name = self.ui.text_name.text()
        gender = self.ui.text_others.text()
        if self.ui.button_male.isChecked():
            gender = self.ui.button_male.text()
        if self.ui.button_female.isChecked():
            gender = self.ui.button_female.text()
        medCardstate = self.ui.button_noMedcard.text()
        if self.ui.button_yesMedcard.isChecked():
            medCardstate = self.ui.button_yesMedcard.text()

        path_patient_folder = os.path.join(self.path_root, id_)
        os.makedirs(path_patient_folder, exist_ok=True)
        path_record = os.path.join(path_patient_folder, id_ + '_records.txt')

        self.dict_patient['name'] = name
        self.dict_patient['work'] = self.ui.text_work.text()
        self.dict_patient['age'] = self.ui.text_age.text()
        self.dict_patient['gender'] = gender
        self.dict_patient['medCardstate'] = medCardstate
        self.dict_patient['medCardinfo'] = self.ui.text_medCardinfo.text()
        self.dict_patient['admitDate'] = str(self.ui.date_admitDate.date().toPyDate())
        self.dict_patient['treatDate'] = str(self.ui.date_treatDate.date().toPyDate())
        self.dict_patient['weight'] = self.ui.text_weight.text()
        self.dict_patient['height'] = self.ui.text_height.text()
        self.dict_patient['thigh'] = self.ui.text_thigh.text()
        self.dict_patient['shank'] = self.ui.text_shank.text()

        items_trans = {'name': 'Họ và tên', 'weight': 'Cân nặng', 'height': 'Chiều cao',
                       'thigh': 'Kích thước đùi', 'shank': 'Kích thước cẳng chân'}
        must_fill = list(items_trans.keys())
        self.errors = ''
        for item in must_fill:
            if self.dict_patient[item] == '':
                self.errors = self.errors + '-' + items_trans[item] + '\n'
        if not self.errors:
            thigh_min = self.limit_ranges['thigh'][0]
            thigh_max = self.limit_ranges['thigh'][1]
            shank_min = self.limit_ranges['shank'][0]
            shank_max = self.limit_ranges['shank'][1]
            if (float(self.dict_patient['thigh']) < float(thigh_min) or
                float(self.dict_patient['thigh']) > float(thigh_max) or
                float(self.dict_patient['shank']) < float(shank_min) or
                float(self.dict_patient['shank']) > float(shank_max)):
                QMessageBox.warning(self, 'Cảnh báo',
                    f'Giá trị ngoài khoảng cho phép. Hãy nhập giá trị thỏa mãn: \n'
                    f'-kích thước đùi trong khoảng {thigh_min} cm đến {thigh_max} cm\n'
                    f'-kích thước cẳng chân trong {shank_min} cm đến {shank_max} cm.')
            else:
                with open(path_record, 'w', encoding='utf-8') as f:
                    f.write(self.ui.text_record.toPlainText()); f.close()
                self.dict_data.update({id_: self.dict_patient})
                with open(self.path_dict_data, 'w') as f:
                    json.dump(self.dict_data, f); f.close()
                self.info_set_state(False)
                self.ui.button_infoEdit.setEnabled(True)
                self.ui.button_infoSave.setEnabled(False)
                self.ui.text_others.setEnabled(False)
                self.ui.text_medCardinfo.setEnabled(False)
                QMessageBox.information(self, 'Thông tin', 'Cập nhật thông tin thành công')

    def info_edit(self):
        self.info_set_state(True)
        self.ui.button_infoEdit.setEnabled(False)
        self.ui.button_infoSave.setEnabled(True)

    def info_set_state(self, state):
        self.ui.text_work.setEnabled(state)
        self.ui.text_age.setEnabled(state)
        self.ui.button_male.setEnabled(state)
        self.ui.button_female.setEnabled(state)
        self.ui.button_others.setEnabled(state)
        self.ui.button_noMedcard.setEnabled(state)
        self.ui.button_yesMedcard.setEnabled(state)

    def info_control_set_state(self, state):
        self.ui.text_name.setEnabled(state)
        self.ui.date_admitDate.setReadOnly(not state)
        self.ui.date_treatDate.setReadOnly(not state)
        self.ui.text_record.setReadOnly(not state)
        self.ui.text_weight.setEnabled(state)
        self.ui.text_height.setEnabled(state)
        self.ui.text_thigh.setEnabled(state)
        self.ui.text_shank.setEnabled(state)

    def date_reset(self):
        self.ui.date_admitDate.setDate(QDate.currentDate())
        self.ui.date_treatDate.setDate(QDate.currentDate())

    def info_clear(self):
        self.ui.text_name.clear()
        self.ui.text_work.clear()
        self.ui.text_age.clear()
        self.ui.buttonGroup_gender.setExclusive(False)
        self.ui.button_male.setChecked(False)
        self.ui.button_female.setChecked(False)
        self.ui.button_others.setChecked(False)
        self.ui.buttonGroup_gender.setExclusive(True)
        self.ui.text_others.clear()
        self.ui.buttonGroup_medCard.setExclusive(False)
        self.ui.button_noMedcard.setChecked(False)
        self.ui.button_yesMedcard.setChecked(False)
        self.ui.buttonGroup_medCard.setExclusive(True)
        self.ui.text_medCardinfo.clear()
        self.date_reset()
        self.ui.text_record.clear()
        self.ui.text_weight.clear()
        self.ui.text_height.clear()
        self.ui.text_thigh.clear()
        self.ui.text_shank.clear()

    def log_training_data(self):
        time_e = self.time_start.secsTo(self.time_stop)
        amp = self.control_data[self.joint]['angle']
        joint_data = self.training_data[self.joint]
        if amp not in joint_data.keys():
            joint_data[amp] = 0
        time_total = joint_data[amp] + time_e
        joint_data.update({amp: time_total})
        with open(self.path_trainning_data, 'w') as f:
            json.dump(self.training_data, f); f.close()
        QMessageBox().information(self, 'Thông báo', 'Bạn đã lưu báo cáo thành công')

    def frame_set_state(self, state):
        self.ui.frame_info.setEnabled(state)
        return super().frame_set_state(state)


# ─────────────────────────────────────────────────────────────────────────────
#  DoctorDev — customable exercises (giống LLRR)
# ─────────────────────────────────────────────────────────────────────────────
class DoctorDev(UserMainScreen):
    def __init__(self, dirs, id, parent=None):
        super().__init__(dirs, id, parent)
        self.ui.button_setHip.clicked.connect(self.set_angle_hip)
        self.ui.button_setKnee.clicked.connect(self.set_angle_knee)
        self.ui.button_setAnkle.clicked.connect(self.set_angle_ankle)

    def set_angle_hip(self):
        self.set_angle_exercise('hip', self.limit_ranges['hip'])

    def set_angle_knee(self):
        self.set_angle_exercise('knee', self.limit_ranges['knee'])

    def set_angle_ankle(self):
        self.set_angle_exercise('ankle', self.limit_ranges['ankle'])

    def set_angle_exercise(self, joint, range_):
        self.joint = joint
        name = self.control_data[joint]['name']
        self.ui.text_cycles.setEnabled(False)
        self.ui.text_timer.setEnabled(False)
        self.set_angle_dialog = SetAngleDialog(self)
        self.set_angle_dialog.update_range(range_)
        self.set_angle_dialog.ui.label_joint.setText(name)
        self.set_angle_dialog.angle.connect(self.set_angle_update)
        self.set_angle_dialog.show()

    def set_angle_update(self, angle, name):
        """Sau khi user nhập góc xong và bấm Lưu ở SetAngleDialog → bật
        Confirm để user bấm xác nhận trước khi bấm Bắt đầu.

        Flow mới: nhập góc → Lưu → Confirm (set_offset + enter CLOSED_LOOP)
        → Bắt đầu (cấp moment). Sau khi motor đang giữ moment, user chọn
        lại bài tập → nhập góc mới → Lưu → Confirm (cập nhật target) →
        Bắt đầu (moment mới).
        """
        self.ui.label_exercise.setText('Chế độ:')
        exercise = f'Chạy khớp {name} góc {angle} độ'
        self.session_set_text(self.ui.text_exercise, exercise)
        self.control_data[self.joint]['angle'] = angle
        self.mode = 2
        self.control_set_state(False)
        # Bật Confirm (chưa bật Bắt đầu) để user bấm Confirm trước.
        # Confirm sẽ: set_offset + enter_closed_loop (lần đầu) hoặc
        # chỉ cập nhật target nếu đã vào CLOSED_LOOP.
        self.ui.button_confirm.setEnabled(True)
        self.ui.button_start.setEnabled(False)



# ─────────────────────────────────────────────────────────────────────────────
#  PatientMainScreen — DoctorPatient + patient-specific sine buttons
# ─────────────────────────────────────────────────────────────────────────────
class PatientMainScreen(DoctorPatient):
    def __init__(self, dirs, id, parent=None):
        self.ui = Ui_PatientScreen()
        super().__init__(dirs, id, parent)
        self.dict_patient = self.dict_data[id]
        self.ui.button_infoSave.clicked.connect(self.info_save)
        self.info_show(id, self.dict_patient)


# ─────────────────────────────────────────────────────────────────────────────
#  DoctorMainScreen — DoctorPatient + DoctorDev + patient CRUD
# ─────────────────────────────────────────────────────────────────────────────
class DoctorMainScreen(DoctorPatient, DoctorDev):
    def __init__(self, dirs, id, parent=None):
        self.ui = Ui_DoctorScreen()
        super().__init__(dirs, id, parent)
        self.path_patient_file = os.path.join(self.path_dev, 'patient_dirs.json')
        self.update_patient_list()
        self.ui.list_patients.currentTextChanged.connect(self.info_select_show)
        self.ui.button_addPatients.clicked.connect(self.info_add)
        self.ui.button_infoSave.clicked.connect(self.info_save)
        self.ui.button_infoDelete.clicked.connect(self.info_delete)
        self.ui.button_infoEdit.clicked.connect(self.info_edit)

    def update_patient_list(self):
        patients = [id_ + ' - ' + self.dict_data[id_]['name'] for id_ in self.dict_data.keys()]
        self.ui.list_patients.addItems(patients)
        self.ui.list_patients.setCurrentIndex(-1)
        try:
            with open(self.path_patient_file, 'r') as f:
                self.dict_all_patients = json.load(f); f.close()
        except FileNotFoundError:
            pass

    def info_select_show(self, item):
        if self.ui.list_patients.currentIndex() == -1:
            self.ui.text_sessionTitle.clear()
            self.ui.button_sessionEdit.setEnabled(False)
            self.ui.frame_exercises.setEnabled(False)
            self.ui.text_cycles.clear()
            self.ui.text_timer.setTime(QTime(0, 0, 0))
            self.ui.label_exercise.clear()
            self.ui.label_cycles_timer.clear()
            self.ui.button_confirm.setEnabled(False)
            self.ui.button_start.setEnabled(False)
            self.ui.button_stop.setEnabled(False)
            self.info_clear()
            self.frame_set_state(False)
            self.ui.frame_info.setEnabled(True)
            self.control_set_state(False)
            self.session_reset_text(self.ui.text_exercise)
            self.session_reset_text(self.ui.text_cycles_timer)
        else:
            id_ = item.split(' - ')[0]
            self.dict_patient = self.dict_data[id_]
            self.ui.button_infoDelete.setEnabled(True)
            self.info_set_state(False)
            self.info_control_set_state(False)
            self.info_show(id_, self.dict_patient)

    def info_add(self):
        prefix = 'BN'
        id_ = prefix + str(self.dict_all_patients['total'])
        self.dict_patient = {}
        self.ui.label_dispID.setText(id_)
        self.ui.text_record.setPlaceholderText('Nhập bệnh án...')
        self.ui.list_patients.setCurrentIndex(-1)
        self.ui.frame_info.setEnabled(True)
        self.ui.button_infoEdit.setEnabled(False)
        self.ui.button_infoSave.setEnabled(True)
        self.ui.button_infoDelete.setEnabled(True)
        self.info_set_state(True)
        self.info_control_set_state(True)

    def info_edit(self):
        super().info_edit()
        self.info_control_set_state(True)

    def info_save(self):
        super().info_save()
        if not self.errors:
            id_ = self.ui.label_dispID.text()
            name = self.ui.text_name.text()
            item = id_ + ' - ' + name
            combobox_content = set([self.ui.list_patients.itemText(i) for i in range(self.ui.list_patients.count())])
            if item not in combobox_content:
                with open(self.path_patient_file, 'w') as f:
                    self.dict_all_patients.update({"total": self.dict_all_patients['total'] + 1,
                                                    id_: {"manager_id": self.user_id, "manager": self.manager_name, "password": ''}})
                    json.dump(self.dict_all_patients, f); f.close()
                path_patient = os.path.join(self.path_root, id_)
                path_control_data = os.path.join(path_patient, 'control_data.json')
                control_data = {'hip':   {'amplitude': '20', 'cycle': '10', 'phase': '0', 'bias': '0',
                                          'angle': 0, 'code': 0, 'name': 'hông'},
                                'knee':  {'amplitude': '30', 'cycle': '10', 'phase': '0', 'bias': '0',
                                          'angle': 0, 'code': 1, 'name': 'đầu gối'},
                                'ankle': {'amplitude': '18', 'cycle': '10', 'phase': '0', 'bias': '0',
                                          'angle': 0, 'code': 2, 'name': 'cổ chân'}}
                with open(path_control_data, 'w') as f:
                    json.dump(control_data, f); f.close()
                path_reports = os.path.join(path_patient, 'reports')
                os.makedirs(path_reports, exist_ok=True)
                self.path_trainning_data = os.path.join(path_reports, f'{id_}_total_training_data.json')
                training_data = {'hip': {}, 'knee': {}, 'ankle': {}}
                with open(self.path_trainning_data, 'w') as f:
                    json.dump(training_data, f); f.close()
                self.ui.list_patients.addItem(item)
                self.ui.list_patients.setCurrentText(item)
                QMessageBox.information(self, 'Xác nhận',
                    'Bạn đã thêm bệnh nhân thành công, xin nhắc bệnh nhân sửa mật khẩu để tránh rủi ro bảo mật')
            self.ui.button_infoDelete.setEnabled(True)
        else:
            QMessageBox.warning(self, "Cảnh báo", "Xin hãy điền các mục sau:\n" + self.errors)

    def info_delete(self):
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Question)
        box.setWindowTitle('Cảnh báo')
        box.setText('Bạn có chắc bạn muốn xóa bệnh nhân này khỏi danh sách?')
        box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        buttonY = box.button(QMessageBox.Yes); buttonY.setText('Có')
        buttonN = box.button(QMessageBox.No);  buttonN.setText('Không')
        box.setDefaultButton(QMessageBox.No)
        box.exec_()

        if box.clickedButton() == buttonY:
            id_ = self.ui.label_dispID.text()
            if self.ui.list_patients.currentIndex() != -1:
                try:
                    self.dict_data.pop(id_)
                    self.dict_all_patients.pop(id_)
                except Exception:
                    QMessageBox.critical(self, 'Lỗi', 'Đã có lỗi xảy ra, vui lòng kiểm tra lại')
                else:
                    with open(self.path_dict_data, 'w') as f:
                        json.dump(self.dict_data, f); f.close()
                    with open(self.path_patient_file, 'w') as f:
                        json.dump(self.dict_all_patients, f); f.close()
            self.ui.list_patients.removeItem(self.ui.list_patients.currentIndex())
            self.ui.list_patients.setCurrentIndex(-1)
            self.ui.label_dispID.clear()
            self.ui.button_infoEdit.setEnabled(False)
            self.ui.button_infoSave.setEnabled(False)
            self.ui.button_infoDelete.setEnabled(False)
            self.info_set_state(False)
            self.info_control_set_state(False)
            QMessageBox.information(self, 'Xác nhận', 'Bạn đã xóa thành công')


# ─────────────────────────────────────────────────────────────────────────────
#  DevMainScreen — DoctorDev + PID + account management
# ─────────────────────────────────────────────────────────────────────────────
class DevMainScreen(DoctorDev):
    def __init__(self, dirs, id, parent=None):
        self.ui = Ui_DevScreen()
        super().__init__(dirs, id, parent)
        self.ui.button_dataEdit.clicked.connect(self.data_edit)
        self.ui.button_dataSave.clicked.connect(self.data_save)
        self.ui.button_accDoctor.clicked.connect(self.account_manager_doctor)
        self.ui.button_accDev.clicked.connect(self.account_manager_dev)

        self.ui.label_dispID.setText(id)
        regex = QRegExp("^[0-9]+\\.[0-9]+$")
        self.validator = QRegExpValidator(regex, self.ui.centralwidget)
        self.ui.text_coefP.setValidator(self.validator)
        self.ui.text_coefI.setValidator(self.validator)
        self.ui.text_coefD.setValidator(self.validator)

        self.data = {}
        with open(self.path_users_file, 'r') as f:
            self.dict_data = json.load(f); f.close()
        self.path_control_data = os.path.join(self.path_root, id, 'control_data.json')
        with open(self.path_control_data, 'r') as f:
            self.control_data = json.load(f); f.close()
        self.path_reports = os.path.join(self.path_root, id, 'reports')

    def data_save(self):
        """Save PID + push xuống embedded computer."""
        self.data['weight'] = self.ui.text_weight.text()
        self.data['height'] = self.ui.text_height.text()
        self.data['thigh'] = self.ui.text_thigh.text()
        self.data['shank'] = self.ui.text_shank.text()
        self.data['P'] = self.ui.text_coefP.text()
        self.data['I'] = self.ui.text_coefI.text()
        self.data['D'] = self.ui.text_coefD.text()

        items_trans = {'weight': 'Cân nặng', 'height': 'Chiều cao', 'thigh': 'Kích thước đùi',
                       'shank': 'Kích thước cẳng chân', 'P': 'Hệ số P', 'I': 'Hệ số I', 'D': 'Hệ số D'}
        must_fill = list(items_trans.keys())
        self.errors = ''
        for item in must_fill:
            if self.data[item] == '':
                self.errors = self.errors + '-' + items_trans[item] + '\n'

        if not self.errors:
            checked = True
            # ── Gửi PID xuống embedded computer ────────────────────────────
            if self.ard:
                p = float(self.data['P'])
                d = float(self.data['D'])
                # CTC dùng Kp + Kd (không có I); gửi cùng giá trị cho 3 khớp.
                self.backend.set_gains((p, p, p), (d, d, d))
                self.ctc.set_gains((p, p, p), (d, d, d))
            if checked:
                self.data_set_state(False)
            else:
                QMessageBox.critical(self, 'Lỗi', 'Lỗi nhận dữ liệu')
        else:
            QMessageBox.warning(self, "Cảnh báo", "Xin hãy điền các mục sau:\n" + self.errors)

    def data_edit(self):
        self.data_set_state(True)

    def data_set_state(self, state):
        self.ui.text_weight.setEnabled(state)
        self.ui.text_height.setEnabled(state)
        self.ui.text_thigh.setEnabled(state)
        self.ui.text_shank.setEnabled(state)
        self.ui.text_coefP.setEnabled(state)
        self.ui.text_coefI.setEnabled(state)
        self.ui.text_coefD.setEnabled(state)
        self.ui.button_dataSave.setEnabled(state)
        self.ui.button_dataEdit.setEnabled(not state)

    def account_manager_doctor(self):
        self.account_manager('doctor')

    def account_manager_dev(self):
        self.account_manager('dev')

    def account_manager(self, account):
        dict_account = {'doctor': {'prefix': 'BS', 'name': 'Bác sỹ',
                                   'dir': os.path.join(self.path_dev, 'doctor_dirs.json')},
                        'dev':    {'prefix': 'KTV', 'name': 'Kỹ thuật viên',
                                   'dir': self.path_users_file}}
        self.acc = dict_account[account]
        try:
            with open(self.acc['dir'], 'r') as f:
                self.dict_users = json.load(f); f.close()
        except FileNotFoundError:
            pass
        data = {'prefix': self.acc['prefix'], 'dir_file': self.dict_users}
        self.account_manager_dialog = AccountManagerDialog(data, self)
        self.account_manager_dialog.ui.label_accountManager.setText(self.acc['name'])
        self.account_manager_dialog.acc_add.connect(self.account_save)
        self.account_manager_dialog.acc_del.connect(self.account_delete)
        self.account_manager_dialog.show()

    def account_save(self, id_, name):
        if id_ not in self.dict_users.keys():
            if self.acc['prefix'] == 'BS':
                path_user = os.path.join(self.path_database, 'doctors', id_)
                os.makedirs(path_user, exist_ok=True)
                path_patient_info = os.path.join(path_user, 'patients-info.json')
                with open(path_patient_info, 'w') as f:
                    json.dump({}, f); f.close()
            else:
                path_user = os.path.join(self.path_root, id_)
                path_control_data = os.path.join(path_user, 'control_data.json')
                control_data = {'hip':   {'amplitude': '20', 'cycle': '10', 'phase': '0', 'bias': '0',
                                          'angle': 0, 'code': 0, 'name': 'hông'},
                                'knee':  {'amplitude': '30', 'cycle': '10', 'phase': '0', 'bias': '0',
                                          'angle': 0, 'code': 1, 'name': 'đầu gối'},
                                'ankle': {'amplitude': '18', 'cycle': '10', 'phase': '0', 'bias': '0',
                                          'angle': 0, 'code': 2, 'name': 'cổ chân'}}
                with open(path_control_data, 'w') as f:
                    json.dump(control_data, f); f.close()
                path_reports = os.path.join(path_user, 'reports')
                os.makedirs(path_reports, exist_ok=True)

            self.dict_users.update({id_: {'name': name, 'manager_id': id_, 'manager': name, 'password': ''}})
            self.dict_users['total'] += 1
            with open(self.acc['dir'], 'w') as f:
                json.dump(self.dict_users, f); f.close()
            self.account_manager_dialog.ui.list_accounts.addItem(id_)
            self.account_manager_dialog.ui.list_accounts.setCurrentText(id_)
            QMessageBox.information(self, 'Xác nhận',
                'Bạn đã thêm tài khoản thành công, xin nhắc chủ tài khoản sửa mật khẩu để tránh rủi ro bảo mật')
        else:
            self.dict_users[id_].update({'name': name, 'manager': name})
            with open(self.acc['dir'], 'w') as f:
                json.dump(self.dict_users, f); f.close()
            if self.acc['prefix'] == 'BS':
                path_patient_info = os.path.join(self.path_database, 'doctors', id_, 'patients-info.json')
                with open(path_patient_info, 'r') as f:
                    patient_info = json.load(f); f.close()
                path_patient_dir = os.path.join(self.path_dev, 'patient_dirs.json')
                with open(path_patient_dir, 'r') as f:
                    patients = json.load(f); f.close()
                for patient in patient_info.keys():
                    patients[patient].update({'manager': name})
                with open(path_patient_dir, 'r') as f:
                    json.dump(patients, f); f.close()
            QMessageBox.information(self, 'Thông tin', 'Cập nhật thông tin thành công')

    def account_delete(self, id_):
        if self.acc['prefix'] == 'BS':
            path_patient_file = os.path.join(self.path_database, 'doctors', id_, 'patients-info.json')
            try:
                with open(path_patient_file, 'r') as f:
                    patient_file = json.load(f); f.close()
            except FileNotFoundError:
                QMessageBox.critical(self, 'Lỗi', 'Đã có lỗi xảy ra')
            else:
                if patient_file:
                    QMessageBox.critical(self, 'Cảnh báo',
                        'Không thể xóa tài khoản bác sỹ này do họ còn có bệnh nhân đang điều trị hoặc chưa chuyển giao bệnh nhân')
                    return
        self.dict_users.pop(id_)
        with open(self.acc['dir'], 'w') as f:
            json.dump(self.dict_users, f); f.close()
        self.account_manager_dialog.ui.text_accountID.clear()
        self.account_manager_dialog.ui.text_name.clear()
        self.account_manager_dialog.ui.list_accounts.removeItem(self.account_manager_dialog.ui.list_accounts.currentIndex())
        QMessageBox.information(self, 'Xác nhận', 'Bạn đã xóa thành công')
        self.account_manager_dialog.ui.list_accounts.setCurrentIndex(-1)

    def log_session_data(self):
        """Log exercise data ra CSV (giống LLRR)."""
        for data in self.specs:
            feedback_data = data['feedback_data']
            if data['mode'] == 3:
                q_set_data = feedback_data[data['joint_code']]['q_set']
                q_fb_data = feedback_data[data['joint_code']]['q_fb']
                data_transpose = [[q_set_data[i], q_fb_data[i]] for i in range(len(q_set_data))]
                joint = feedback_data[data['joint_code']]['name']
                data_headers = [f'{joint}_qset', f'{joint}_qfeedback']
            if data['mode'] == 4:
                q_set_data = []; q_fb_data = []
                for joint in feedback_data:
                    q_set_data.append(joint['q_set'])
                    q_fb_data.append(joint['q_fb'])
                data_transpose = [[q_set_data[0][i], q_set_data[1][i], q_set_data[2][i],
                                   q_fb_data[0][i], q_fb_data[1][i], q_fb_data[2][i]]
                                  for i in range(len(q_set_data[0]))]
                data_headers = ['hip_qset', 'knee_qset', 'ankle_qset',
                                'hip_qfeedback', 'knee_qfeedback', 'ankle_qfeedback']

            path_session_data = os.path.join(self.path_reports,
                                              self.session_info['filename'] + '_Exercise_' + str(self.specs.index(data)) + '.csv')
            with open(path_session_data, 'w', encoding='utf-8', newline='') as f:
                csvwriter = csv.writer(f)
                csvwriter.writerow(data_headers)
                csvwriter.writerows(data_transpose)
                f.close()
        QMessageBox().information(self, 'Thông báo', 'Bạn đã lưu báo cáo thành công')