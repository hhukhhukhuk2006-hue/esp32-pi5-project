import os
import re

from PyQt5.QtGui import (QRegExpValidator, QPainter, QFont)
from PyQt5.QtCore import (QRegExp, QTime, QPointF, pyqtSignal, QDate)
from PyQt5.QtChart import (QBarCategoryAxis, QBarSet, QChart, QChartView, QLineSeries, QValueAxis, QHorizontalBarSeries)
from PyQt5.QtWidgets import (QDialog, QMessageBox, QTableWidgetItem, QLineEdit, QHBoxLayout)

from UiScripts.session_ui import Ui_SessionDialog
from UiScripts.report_ui  import Ui_ReportDialog
from UiScripts.account_ui import Ui_AccountDialog
from UiScripts.account_manager_ui import Ui_AccountManagerDialog
from UiScripts.set_angle_ui import Ui_SetAngleDialog
from UiScripts.set_sine_ui import Ui_SetSineDialog

class SessionReport(QDialog):
    '''
    Class containing methods for both SessionDialog and ReportDialog. \n
    Loading the ui and has method for updating session info.
    '''
    def __init__(self, ui, parent=None):
        super().__init__(parent)
        self.ui = ui
        self.ui.setupUi(self)
        self.ui.text_sessionDate.setDate(QDate.currentDate())
        self.ui.text_sessionTime.setTime(QTime.currentTime())

    def session_update(self, session_info):
        '''
        Update session information from session_info
        '''
        self.ui.text_sessionTitle.setText(session_info['title'])
        self.ui.text_sessionTime.setDisplayFormat('hh:mm:ss')
        self.ui.text_sessionTime.setTime(QTime.fromString(session_info['time'], 'hh:mm:ss'))
        self.ui.text_patient.setText(session_info['patient'])
        self.ui.text_patientID.setText(session_info['patient_id'])
        self.ui.text_doctor.setText(session_info['doctor'])
        self.ui.text_doctorID.setText(session_info['doctor_id'])
        self.ui.text_description.setPlainText(session_info['description'])
        self.path_reports = session_info['path_report']
        self.filename = session_info['filename']

class SessionDialog(SessionReport):
    '''
    Class containing methods for session popup.
    '''
    sessionSaved = pyqtSignal(dict)
    def __init__(self, parent=None):
        super().__init__(Ui_SessionDialog(),parent)
        self.ui.button_save.clicked.connect(self.session_save)

    def session_save(self):
        '''
        Method linked to QPushButton button_save. Triggered when pressed. \n
        Saves session information to session_info and sends to mainscreen through sessionSaved signal to be updated.
        '''
        self.session_info = {'title': self.ui.text_sessionTitle.text(),
                             'date': str(self.ui.text_sessionDate.date().toPyDate()), 'time': self.ui.text_sessionTime.time().toString('hh:mm:ss'),
                             'patient': self.ui.text_patient.text(), 'patient_id': self.ui.text_patientID.text(),
                             'doctor': self.ui.text_doctor.text(), 'doctor_id': self.ui.text_doctorID.text(),
                             'description': self.ui.text_description.toPlainText()}

        self.sessionSaved.emit(self.session_info)
        self.close()

class ReportDialog(SessionReport):
    '''
    Class containing methods for report popup, including loading list of reports, select and display reports, and plotting data.
    '''
    log = pyqtSignal()
    def __init__(self, parent=None):
        super().__init__(Ui_ReportDialog(), parent)
        self.ui.button_save.clicked.connect(self.report_save)
        self.ui.list_reports.currentTextChanged.connect(self.report_select_show)
        
        # self.reports_update_list()
        ## Allocate which widget to plot which joint data
        self.dict_joint = {'hip': {'name':'hông', 'widget': self.ui.chart_hip}, 'knee':{'name': 'đầu gối', 'widget': self.ui.chart_knee} , 'ankle': {'name':'cổ chân', 'widget': self.ui.chart_ankle}}
        ## Create a chart for each widget to be drawn on later
        for joint in self.dict_joint.keys():
            self.dict_joint[joint]['chart'] = self.chart_config(self.dict_joint[joint]['widget'])
            
    def reports_update_list(self):
        '''
        Loads the list of reports of current patient.
        '''
        reports = [report.split('.')[0] for report in os.listdir(self.path_reports) if report.endswith('.txt')]
        self.ui.list_reports.addItems(reports)
        self.ui.list_reports.setCurrentIndex(-1)
        
    def report_chart(self, data):
        '''
        PLot the bar chart of time spent per exercise's amplitude. \n
        Used by patient and doctor account.
        '''
        for joint in list(data.keys()):
            joint_data = data[joint]
            y = list(joint_data.keys())
            y.sort()
            x = [joint_data[amp]/60 for amp in y] ## Convert seconds to minutes
            self.bar_chart_config(x, y, self.dict_joint[joint]['name'], self.dict_joint[joint]['chart'])

    def bar_chart_config(self, x, y, title, chart):
        '''
        Plots bar chart on chart using data from x and y for x-axis and y-axis, updates name of joint using title.
        '''
        set = QBarSet('TimeSpent')
        set.append(x)

        bar_series = QHorizontalBarSeries()
        bar_series.append(set)

        axis_x = QValueAxis()
        axis_y = QBarCategoryAxis()
        axis_y.append(y)

        chart.addSeries(bar_series)
        chart.setTitle(f'Thống kê bài tập hình sin khớp <br> <center> {title}')
        chart.setTitleFont(QFont("MS Shell Dlg 2", 11, QFont.Bold))
        chart.setAxisX(axis_x, bar_series)
        chart.axisX().setTitleText('Thời gian đã tập(Phút)')
        chart.axisX().setTitleFont(QFont("MS Shell Dlg 2", 8))
        chart.setAxisY(axis_y, bar_series)
        if y:
            chart.axisY().setTitleText('Biên độ(Độ)')
            chart.axisY().setTitleFont(QFont("MS Shell Dlg 2", 8))
        chart.setAnimationOptions(QChart.SeriesAnimations)
        chart.legend().setVisible(False)

    def report_error_chart(self, data, ex_id):
        '''
        Plot line chart of error in feedback data of each exercise. \n
        Used by dev account.
        '''
        for joint in data:
            error_data = joint['error']
            if error_data:
                self.line_chart_config(error_data, ex_id, self.dict_joint[joint['name']]['name'], self.dict_joint[joint['name']]['chart'])
         
    def line_chart_config(self, error_data, ex_id, title, chart):
        '''
        Plot line chart on chart using error_data for y-axis, setting legend using ex_id, and update name of joint using title.
        '''
        line_series = QLineSeries()
        line_series.setName(f'Bài {ex_id}')
        for i in range(len(error_data)):
            line_series.append(QPointF(i, error_data[i]))

        chart.setTitle(f'Sai số khớp {title}')
        chart.setTitleFont(QFont("MS Shell Dlg 2", 11, QFont.Bold))
        chart.addSeries(line_series)
        chart.createDefaultAxes()
        
        # chart.axisX().setTitleText('Điểm')
        # chart.axisX().setTitleFont(QFont("MS Shell Dlg 2", 8))      
        chart.axisY().setTitleText('Sai số(Độ)')
        chart.axisY().setTitleFont(QFont("MS Shell Dlg 2", 8))
        
    def chart_config(self, widget):
        '''
        Creates chart onto widget.
        '''
        chart = QChart()

        chart_view = QChartView(chart)
        chart_view.setRenderHint(QPainter.Antialiasing)
        
        widget.setContentsMargins(0, 0, 0, 0)
        lay = QHBoxLayout(widget)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(chart_view)
        return chart

    def report_update(self,session_data):
        '''
        Updates exercises details (name of exercise, amplitude and cycle, number of cycles or time) on the QTableWidget table_exercises using session_data
        '''
        for session_count in list(session_data.keys()):
            rowPosition = self.ui.table_exercises.rowCount()
            self.ui.table_exercises.insertRow(rowPosition)
            self.ui.table_exercises.setItem(rowPosition, 0, QTableWidgetItem(session_data[session_count]['exercise']))
            self.ui.table_exercises.setItem(rowPosition, 1, QTableWidgetItem(session_data[session_count]['details']))
            self.ui.table_exercises.setItem(rowPosition, 2, QTableWidgetItem(session_data[session_count]['info']))
            self.ui.table_exercises.resizeRowsToContents()

    def report_select_show(self, report):
        '''
        Method linked to QComboBox list_reports. Triggered when selecting from the dropdown list or when changing the index. \n
        Update the QTextEdit text_report with content of selected report.
        '''
        if self.ui.list_reports.currentIndex() == -1:
            self.ui.text_report.clear()
        else:
            path_report = os.path.join(self.path_reports, report + '.txt')
            with open(path_report, 'r', encoding='utf-8') as f:
                report_content = f.read()
                f.close()
            self.ui.text_report.setText(report_content)

    def report_save(self):
        '''
        Method linked to QPushButton button_save. Triggered when pressed. \n
        Save the session information and report data to txt file. Send log signal to mainscreen to log chart data and feedback data.
        '''
        ## Create a list of lines to use writelines to txt file
        report_info =   ['Thông tin buổi tập:',
                         'Tên buổi tập: ' + self.ui.text_sessionTitle.text(),
                         'Ngày: ' + str(self.ui.text_sessionDate.date().toPyDate()) + '\t' + 'Thời gian: ' + self.ui.text_sessionTime.time().toString('hh:mm:ss') +' đến ' + self.ui.text_sessionEndTime.time().toString('hh:mm:ss'),
                         'Bệnh nhân: ' + self.ui.text_patient.text() + '\t' + 'ID: ' + self.ui.text_patientID.text(),
                         'Bác sỹ: ' + self.ui.text_doctor.text() + '\t' + 'ID: ' + self.ui.text_doctorID.text(),
                         '',
                         'Nội dung buổi tập:'
                        ]
        for row in range(self.ui.table_exercises.rowCount()): # Iterate through the QTableWidget table_exercises to get exercises content
            content = 'Bài tập '+ str(row+1) + ': ' + self.ui.table_exercises.item(row,0).text() + '\t' + 'Nội dung: ' + self.ui.table_exercises.item(row, 1).text() + '\t' + self.ui.table_exercises.item(row, 2).text()
            if re.split(r'\d+',self.ui.text_doctorID.text())[0] == 'KTV':
                content = content + '\t' + 'T.tin tải: ' + self.ui.table_exercises.item(row, 3).text() + '\t' + 'H.số PID: ' + self.ui.table_exercises.item(row, 4).text() + '\t' + 'Dải sai số: ' + self.ui.table_exercises.item(row, 5).text()
            report_info.append(content) ## Add each exercise to the list
        report_info.append('\n') ## Add a blank line
        report_info.append('Nhận xét đánh giá:')
        report_info.append(self.ui.text_evalation.toPlainText())

        report_info = map(lambda x: x + '\n', report_info) ## Writelines doesn't actually add newline, so it needs to be added manually for each element in the list

        path_file = os.path.join(self.path_reports, self.filename +'.txt')
        with open(path_file, 'w', encoding="utf-8") as f:
            f.writelines(report_info)
            f.close()
        self.log.emit()
        
        # self.close()

class SetAngleDialog(QDialog):
    '''
    Class containing methods for set angle popup.
    '''
    angle = pyqtSignal(str,str) ## PyQT signal, send 2 str arguments when emitted
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_SetAngleDialog()
        self.ui.setupUi(self)        
        self.ui.button_save.clicked.connect(self.set_angle)

        regex = QRegExp("^-?[0-9]{3}$") ## Angle input format: real number up to 3 digits
        validator = QRegExpValidator(regex, self.ui.text_angle)
        self.ui.text_angle.setValidator(validator)

    def update_range(self, range):
        '''
        Udates minimum and maximum value for angle input from range.
        '''
        self.angle_min = range[0]
        self.angle_max = range[1]

    def set_angle(self):
        '''
        Method linked to QPushButton button_save. Triggered when pressed. \n
        Set the input angle. Check for validation. 
        '''
        angle_value = self.ui.text_angle.text()
        if angle_value == '':
            QMessageBox.warning(self, "Cảnh báo", "Xin hãy điền đủ các mục")
        elif float(angle_value) < self.angle_min or float(angle_value) > self.angle_max:
            QMessageBox.warning(self, 'Cảnh báo', 'Giá trị ngoài khoảng cho phép. Hãy nhập giá trị trong khoảng {} độ đến {} độ'.format(self.angle_min, self.angle_max))
        else: 
            self.angle.emit(angle_value,self.ui.label_joint.text())
            self.close()
        
class SetSineDialog(QDialog):
    '''
    Class containing methods for set sine specs popup.
    '''
    sine_data = pyqtSignal(str,str,str,str, bool) ## PyQT signal, send 4 str arguments and 1 bool argument when emitted
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_SetSineDialog()
        self.ui.setupUi(self)
        self.ui.button_save.clicked.connect(self.set_sine)
        self.ui.button_setDefault.clicked.connect(self.set_sine_default)

        regex = QRegExp("^[0-9]{3}$") ## input format: integer up to 3 digits
        validator = QRegExpValidator(regex, self.ui.text_amplitude)
        self.ui.text_amplitude.setValidator(validator)
        self.ui.text_cycle.setValidator(validator)
        self.ui.text_phase.setValidator(validator)
        self.ui.text_bias.setValidator(validator)

        self.set_default = False
        self.set_ok = False

    def update_limit_amplitude(self, limit_amplitude):
        '''
        Updates the limit amplitude.
        '''
        self.limit_amplitude = limit_amplitude

    def set_sine_default(self):
        '''
        Method linked to QPushButton button_setDefault. Triggered when pressed. \n
        Set the specs and send True signal to mainscreen to save the specs to file.
        '''
        self.set_default = True
        joint = self.ui.label_joint.text()
        self.set_sine()
        if self.set_ok:
            QMessageBox.information(self,'Thông báo', f'Thông số quỹ đạo góc này đã được đặt mặc định cho bài tập hình sin khớp {joint} cho bệnh nhân này')

    def set_sine(self):
        '''
        Set sine specs from input fields. Check for validation. Send specs to mainscreen to be updated.
        '''
        amplitude = self.ui.text_amplitude.text()
        cycle = self.ui.text_cycle.text()
        phase = self.ui.text_phase.text()
        bias = self.ui.text_bias.text()

        if '' in [amplitude, cycle, phase, bias]:
            QMessageBox.warning(self, "Cảnh báo", "Xin hãy điền đủ các mục")
        else:
            if int(amplitude) > self.limit_amplitude:
                QMessageBox.warning(self, 'Cảnh báo', 'Giá trị ngoài khoảng cho phép. Hãy nhập giá trị biên độ trong khoảng 0 độ đến {} độ'.format(self.limit_amplitude))
            else:
                self.sine_data.emit(amplitude, cycle, phase, bias, self.set_default)
                self.set_ok = True
                self.close()

class AccountDialog(QDialog):
    '''
    Class containing methods for account popup.
    '''
    password = pyqtSignal(str) ## PyQT signal, send 1 str argument when emitted
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_AccountDialog()
        self.ui.setupUi(self)
        self.ui.text_passwordNew.textChanged.connect(self.button_save_state)
        self.ui.button_save.clicked.connect(self.account_password_check)
        self.ui.button_pwVisibility.clicked.connect(self.toggle_pw_visibility)
        self.ui.button_pwReVisibility.clicked.connect(self.toggle_pw_re_visibility)

    def button_save_state(self):
        '''
        Method linked to QLineEdit text_paswordNew. Triggered when receiving inputs. \n
        Enables QPushButton button_save when there's input, disables it when it's empty. 
        '''
        if self.ui.text_passwordNew.text() == '':
            self.ui.button_save.setEnabled(False)
            self.ui.button_save.clearFocus()
        else:
            self.ui.button_save.setEnabled(True)

    def toggle_pw_visibility(self):
        '''
        Method linked to QPushButton button_pwVisbility. Triggered when pressed. \n
        Toggles the icon of QPushButton button_pwVisbility and the mode of QLineEdit text_passwordNew
        '''
        if self.ui.text_passwordNew.echoMode() == QLineEdit.Normal:
            self.ui.text_passwordNew.setEchoMode(QLineEdit.Password)
            self.ui.button_pwVisibility.setStyleSheet("image: url(:/icons/icons/show.png);\n"
"color: rgb(0, 0, 0); \n"
"background-color: rgba(255, 255, 255, 0);\n"
"border:0px;")
        else:
            self.ui.text_passwordNew.setEchoMode(QLineEdit.Normal)
            self.ui.button_pwVisibility.setStyleSheet("image: url(:/icons/icons/hide.png);\n"
"color: rgb(0, 0, 0); \n"
"background-color: rgba(255, 255, 255, 0);\n"
"border:0px;")

    def toggle_pw_re_visibility(self):
        '''
        Method linked to QPushButton button_pwReVisbility. Triggered when pressed. \n
        Toggles the icon of QPushButton button_pwReVisbility and the mode of QLineEdit text_passwordRe
        '''
        if self.ui.text_passwordRe.echoMode() == QLineEdit.Normal:
            self.ui.text_passwordRe.setEchoMode(QLineEdit.Password)
            self.ui.button_pwReVisibility.setStyleSheet("image: url(:/icons/icons/show.png);\n"
"color: rgb(0, 0, 0); \n"
"background-color: rgba(255, 255, 255, 0);\n"
"border:0px;")
        else:
            self.ui.text_passwordRe.setEchoMode(QLineEdit.Normal)
            self.ui.button_pwReVisibility.setStyleSheet("image: url(:/icons/icons/hide.png);\n"
"color: rgb(0, 0, 0); \n"
"background-color: rgba(255, 255, 255, 0);\n"
"border:0px;")

    # def set_visibility(self, mode):
    #     self.ui.text_passwordNew.setEchoMode(mode)
    #     self.ui.text_passwordRe.setEchoMode(mode)

    def account_password_check(self):
        '''
        Method linked to QPushButton button_save. Triggered when pressed. \n
        Checks if new password and re-enter password are the same. Send to mainscreen to be saved if valid.
        '''
        # self.set_visibility(QLineEdit.Normal)
        password_new = self.ui.text_passwordNew.text()
        password_reenter = self.ui.text_passwordRe.text()
        # self.set_visibility(QLineEdit.Password)
        if password_new == password_reenter:
            self.password.emit(password_new)
            self.close()
        else:
            QMessageBox.warning(self,'Cảnh báo', 'Bạn đã nhập mật khẩu không chính xác. Xin hãy nhập lại')

class AccountManagerDialog(QDialog):
    '''
    Class containing methods for doctor and dev account managing popup.
    '''
    acc_add = pyqtSignal(str,str) ## PyQT signal, send 2 str arguments when emitted
    acc_del = pyqtSignal(str) ## PyQT signal, send 1 str argument when emitted
    def __init__(self, data, parent=None):
        super().__init__(parent)
        self.ui = Ui_AccountManagerDialog()
        self.ui.setupUi(self)
        self.data = data

        self.account_update_data()

        self.ui.list_accounts.currentTextChanged.connect(self.account_select_show)
        self.ui.button_add.clicked.connect(self.account_add)
        self.ui.button_save.clicked.connect(self.account_save)
        self.ui.button_delete.clicked.connect(self.account_delete)

    def account_update_data(self):
        '''
        Load the list of accounts. Add to QComboBox list_accounts.
        '''
        accounts = [id for id in self.data['dir_file'].keys()]
        accounts.remove('total') ## Exclude the key 'total'
        self.ui.list_accounts.addItems(accounts)
        self.ui.list_accounts.setCurrentIndex(-1)

    def account_select_show(self, id):
        '''
        Method linked to QComboBox list_accounts. Triggered when selecting from the dropdown list or when changing the index. \n
        Updates QLineEdit text_name and text_accountID with id and the selected account's data or clear them when adding new accounts.
        '''
        if self.ui.list_accounts.currentIndex() == -1:
            self.button_set_state(False)
            self.ui.text_name.clear()
        else:
            self.ui.text_accountID.setText(id)
            self.ui.text_name.setText(self.data['dir_file'][id]['manager'])
            self.button_set_state(True)

    def account_add(self):
        '''
        Method linked to QPushButton button_add. Triggered when pressed. \n
        Add a new account.
        '''
        id_new = self.data['prefix'] + str(self.data['dir_file']['total'])
        self.ui.text_accountID.setText(id_new)

    def account_save(self):
        '''
        Method linked to QPushButton button_save. Triggered when pressed. \n
        Send the acc_add signal to mainscreen to save the information of current account.
        '''
        id = self.ui.text_accountID.text()
        name = self.ui.text_name.text()
        if name == '':
            QMessageBox.warning(self, 'Cảnh báo', 'Hãy nhập tên chủ tài khoản')
        else:
            self.acc_add.emit(id, name) ## Send id and name to mainscreen

    def account_delete(self):
        '''
        Method linked to QPushButton button_delete. Triggered when pressed. \n
        Send the acc_del signal to mainscreen to propose a deletion of current account.
        '''
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Question)
        box.setWindowTitle('Cảnh báo')
        box.setText('Bạn có chắc bạn muốn xóa bệnh nhân này khỏi danh sách?')
        box.setStandardButtons(QMessageBox.Yes|QMessageBox.No)
        buttonY = box.button(QMessageBox.Yes)
        buttonY.setText('Có')
        buttonN = box.button(QMessageBox.No)
        buttonN.setText('Không')
        box.setDefaultButton(QMessageBox.No)
        box.exec_()

        if box.clickedButton() == buttonY:
            id = self.ui.text_accountID.text()
            if self.ui.list_accounts.currentIndex() != -1:
                self.acc_del.emit(id) ## Send id to mainscreen

    def button_set_state(self, state):
        '''
        Toggles the state of enable and disable of QPushButton button_delete and button_save.
        '''
        self.ui.button_delete.setEnabled(state)
        self.ui.button_save.setEnabled(state)