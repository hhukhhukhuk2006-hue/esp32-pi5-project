import os
import json
import re

from PyQt5.QtWidgets import (QDialog, QLineEdit)

from UiScripts.login_ui import Ui_LoginScreen
from mainscreen import (DevMainScreen, PatientMainScreen, DoctorMainScreen)

class LoginScreen(QDialog):
    '''
    Class containing methods for the login screen
    '''
    def __init__(self, parent=None):
        super().__init__(parent)

        self.declare_paths()

        self.ui = Ui_LoginScreen()
        self.ui.setupUi(self)     
        self.ui.button_login.clicked.connect(self.check_ID)
        self.ui.button_visibility.clicked.connect(self.toggle_visibility)

    def declare_paths(self):
        '''
        Declares app directories.
        '''
        # Absolute path so GUI works regardless of cwd
        path_root_app = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.path_assets = os.path.join(path_root_app, 'assets')
        self.path_icons = os.path.join(self.path_assets, 'icons')
        self.path_database = os.path.join(path_root_app, 'database')
        self.path_dev = os.path.join(self.path_database, 'dev')
        
    def toggle_visibility(self):
        '''
        Method linked to QPushButton button_visbility. Triggered when pressed. \n
        Toggles the icon of QPushButton button_visbility and the mode of QLineEdit text_inputPassword
        '''
        if self.ui.text_inputPassword.echoMode() == QLineEdit.Normal:
            self.ui.text_inputPassword.setEchoMode(QLineEdit.Password)
            self.ui.button_visibility.setStyleSheet("image: url(:/icons/icons/show.png);\n"
                                                    "color: rgb(0, 0, 0); \n"
                                                    "background-color: rgba(255, 255, 255, 0);\n"
                                                    "border:0px;")
        else:
            self.ui.text_inputPassword.setEchoMode(QLineEdit.Normal)
            self.ui.button_visibility.setStyleSheet("image: url(:/icons/icons/hide.png);\n"
                                                    "color: rgb(0, 0, 0); \n"
                                                    "background-color: rgba(255, 255, 255, 0);\n"
                                                    "border:0px;")

    def check_ID(self):
        '''
        Method linked to QPushButton button_login. Triggered when pressed. \n
        Checks the id from QLineEdit text_inputID and password from QlineEdit text_inputPassword. \n
        If valid, opens mainscreen accordingly.
        '''
        id = self.ui.text_inputID.text()
        password = self.ui.text_inputPassword.text()
        dict_users = {'BN':'patient_dirs','BS':'doctor_dirs', 'KTV':'dev_dirs'}

        split = re.split(r'\d+', id) ## Seperate the index and the prefix from the id => [prefix, index]
        prefix = split[0]
        try:
            ## Check if prefix is one of the 3 accounts by indexing the prefix as a key of dict_users.
            ## If this throws an error, that means the id is not 1 of the 3 account types
            path_users_dir_file = os.path.join(self.path_dev, dict_users[prefix] + '.json') 
        except:
            self.ui.label_notification.setText('Bạn đã nhập sai ID hoặc tài khoản không tồn tại')
        else: ## If the id is 1 of the 3 account types
            dict_user_dir = {}
            if os.path.exists(path_users_dir_file): ## If the file exists, open it
                with open(path_users_dir_file, 'r') as f:
                    dict_user_dir = json.load(f)
                    f.close()
            # else: ## If not, create the file
            #     with open(path_users_dir_file, 'w') as f:
            #         json.dump(dict_user_dir, f)
            #         f.close()

            if id in list(dict_user_dir.keys()): ## If the id exits
                user = dict_user_dir[id]
                if password == user['password']: ## If the password is correct
                    dirs = [self.path_assets, self.path_icons, self.path_database, self.path_dev, path_users_dir_file, user]
                    match prefix: 
                        case 'BN':
                            path_root = os.path.join(self.path_database, 'doctors', user['manager_id'])
                            dirs.append(path_root)
                            self.mainscreen = PatientMainScreen(dirs, id)
                            
                        case 'BS':
                            path_root = os.path.join(self.path_database, 'doctors', id)
                            dirs.append(path_root)
                            self.mainscreen = DoctorMainScreen(dirs, id)
                            
                        case 'KTV':
                            path_root = os.path.join(self.path_dev, 'devs-data')
                            dirs.append(path_root)
                            self.mainscreen = DevMainScreen(dirs, id)

                    self.mainscreen.login_screen = LoginScreen() ## add the LoginScreen() as a property of the mainscreen to open it when logging out
                    self.mainscreen.showMaximized()
                    self.close()
                else:
                    self.ui.label_notification.setText('Bạn đã nhập sai mật khẩu')
            else:
                self.ui.label_notification.setText('Tài khoản không tồn tại')