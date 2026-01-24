import re
import sys
import threading
from time import sleep

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.uic.properties import QtWidgets

from ui_calibration import Ui_Calibration
from ui_client import Ui_MainWindow
from ui_add_user import Ui_Add_user
from PyQt5.QtGui import *
from Client import *
from Command import COMMAND as cmd
from Command import LegNames as ln
from Command import ServNames as sn

import cv2
import pygame

class MyWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        self.add_user_window = None
        self.calibrationWindow = None
        self.setupUi(self)
        
        self.pushButton.clicked.connect(self.connect)
        self.Button_Forward.pressed.connect(self.forward)
        self.Button_Forward.released.connect(self.stop)
        self.Button_Backward.pressed.connect(self.backward)
        self.Button_Backward.released.connect(self.stop)
        self.Button_Left.pressed.connect(self.left)
        self.Button_Left.released.connect(self.stop)
        self.Button_Right.pressed.connect(self.right)
        self.Button_Right.released.connect(self.stop)
        self.Button_Sonic.clicked.connect(self.sonic)

        self.hands_up.clicked.connect(self.hands)
        self.Play_Music.clicked.connect(self.play_music)
        self.Pause_Music.clicked.connect(self.stop_music)
        self.func1.clicked.connect(self.fnc1)
        self.func2.clicked.connect(self.fnc2)

        self.func3.pressed.connect(self.fnc3)
        self.func3.released.connect(self.lookstop)
        self.func4.pressed.connect(self.fnc4)
        self.func4.released.connect(self.lookstop)

        self.Radio_ON.clicked.connect(self.RadioPlay)
        self.Radio_OFF.clicked.connect(self.RadioStop)
        self.About0.clicked.connect(self.ABOUT0)
        self.about1.clicked.connect(self.ABOUT1)
        self.about2.clicked.connect(self.ABOUT2)
        self.about3.clicked.connect(self.ABOUT3)
        self.about4.clicked.connect(self.ABOUT4)
        self.about5.clicked.connect(self.ABOUT5)
        self.MENU.clicked.connect(self.Menu)
        self.Video_ON.clicked.connect(self.VideoON)
        self.Video_OFF.clicked.connect(self.VideoOFF)

        self.pushButton_calibration.clicked.connect(self.calibration)
        self.Take_photo_button.clicked.connect(self.add_user)
        self.Video_label.setGeometry(20, 50, 400, 300)
        self.timer_sonic = QTimer(self)
        self.timer_sonic.timeout.connect(self.getSonicData)

        self.timer_video = QTimer(self)
        self.timer_video.timeout.connect(self.get_video_frame)
        self.timer_video.start(10)
        
        self.client = Client()
        
        self.gamepad = None
        self.gamepad_timer = QTimer(self)
        self.gamepad_timer.timeout.connect(self.check_gamepad)
        self.init_gamepad()

    def init_gamepad(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.gamepad = pygame.joystick.Joystick(0)
            self.gamepad.init()
            print(f"Gamepad connected: {self.gamepad.get_name()}")
            self.gamepad_timer.start(50)

    def keyPressEvent(self, event):
        if not self.client.tcp_flag:
            return
            
        if event.key() == Qt.Key_W:
            self.forward()
        elif event.key() == Qt.Key_S:
            self.backward()
        elif event.key() == Qt.Key_A:
            self.left()
        elif event.key() == Qt.Key_D:
            self.right()
        elif event.key() == Qt.Key_1:
            self.fnc1()
        elif event.key() == Qt.Key_2:
            self.fnc2()
        elif event.key() == Qt.Key_3:
            self.fnc3()
        elif event.key() == Qt.Key_4:
            self.fnc4()
        elif event.key() == Qt.Key_Space:
            self.stop()
        elif event.key() == Qt.Key_M:
            self.play_music()
        elif event.key() == Qt.Key_N:
            self.stop_music()
        elif event.key() == Qt.Key_R:
            self.RadioPlay()
        elif event.key() == Qt.Key_T:
            self.RadioStop()
        elif event.key() == Qt.Key_H:
            self.hands()
        elif event.key() == Qt.Key_V:
            self.sonic()
        elif event.key() == Qt.Key_Escape:
            self.Menu()

    def keyReleaseEvent(self, event):
        if event.key() in (Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D, Qt.Key_3, Qt.Key_4):
            self.stop()
            if event.key() in (Qt.Key_3, Qt.Key_4):
                self.lookstop()

    def check_gamepad(self):
        if not self.gamepad or not self.client.tcp_flag:
            return
            
        pygame.event.pump()
        
        left_y = self.gamepad.get_axis(1)
        left_x = self.gamepad.get_axis(0)
        right_y = self.gamepad.get_axis(3)
        
        A = self.gamepad.get_button(0)
        B = self.gamepad.get_button(1)
        X = self.gamepad.get_button(2)
        Y = self.gamepad.get_button(3)
        LB = self.gamepad.get_button(4)
        RB = self.gamepad.get_button(5)
        back = self.gamepad.get_button(6)
        start = self.gamepad.get_button(7)
        
        deadzone = 0.2
        
        if abs(left_y) > deadzone:
            if left_y < -deadzone:
                self.forward()
            elif left_y > deadzone:
                self.backward()
        elif abs(left_x) > deadzone:
            if left_x < -deadzone:
                self.left()
            elif left_x > deadzone:
                self.right()
        else:
            self.stop()
        
        if abs(right_y) > deadzone:
            if right_y < -deadzone:
                self.fnc3()
            elif right_y > deadzone:
                self.fnc4()
        else:
            self.lookstop()
        
        if A:
            self.fnc1()
        if B:
            self.fnc2()
        if X:
            self.play_music()
        if Y:
            self.stop_music()
        if LB:
            self.RadioPlay()
        if RB:
            self.RadioStop()
        if back:
            self.hands()
        if start:
            self.Menu()

    def connect(self):
        self.lineEdit.text()
        file = open('IP.txt', 'w')
        file.write(self.lineEdit.text())
        file.close()
        pattern = r'^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$'

        if self.pushButton.text() == 'Connect':
            if re.match(pattern, self.lineEdit.text()):
                self.IP = self.lineEdit.text()

                self.client.turn_on_client(self.IP)
                self.video = threading.Thread(target=self.client.receiving_video, args=(self.IP,))
                self.instruction = threading.Thread(target=self.receive_instruction, args=(self.IP,))
                self.video.start()
                self.instruction.start()
                self.pushButton.setText('Disconnect')
                self.label_err.setText(" ")
            else:
                self.label_err.setText("Ошибка ввода ip")
        else:
            self.client.stop_video_thread = True
            self.client.stop_instruction_thread = True
            self.client.tcp_flag = False
            self.client.turn_off_client()
            self.pushButton.setText('Connect')

    def receive_instruction(self, ip):
        try:
            self.client.client_socket1.connect((ip, 5001))
            self.client.tcp_flag = True
            print("Connection Successful !")
            self.client.send_data(cmd.CMD_CALIBRATION_ALL + '\n')
        except Exception as e:
            print("Connect to server Faild!: Server IP is right? Server is opend?")
            self.client.tcp_flag = False
        while True:
            try:
                alldata = self.client.receive_data()
            except:
                self.client.tcp_flag = False
                break
            print(alldata)
            if alldata == '':
                break
            else:
                cmdArray = alldata.split('\n')
                print(cmdArray)
                if cmdArray[-1] != "":
                    cmdArray == cmdArray[:-1]
            for oneCmd in cmdArray:
                data = oneCmd.split("#")
                if data == "":
                    self.client.tcp_flag = False
                    break
                elif data[0] == cmd.CMD_SONIC:
                    self.Button_Sonic.setText(data[1] + 'cm')
                elif data[0] == cmd.CMD_CALIBRATION_ALL:
                    self.list_angle = []
                    self.list_angle.append(data[1:6])
                    self.list_angle.append(data[6:11])
                    self.list_angle.append(data[11:16])
                    self.list_angle.append(data[16:21])
                else:
                    pass

    def get_video_frame(self):
        try:
            if self.client.video_flag == False:
                height, width, bytesPerComponent = self.client.image.shape
                cv2.cvtColor(self.client.image, cv2.COLOR_BGR2RGB, self.client.image)
                QImg = QImage(self.client.image.data, width, height, 3 * width, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(QImg).scaled(390, 240)
                self.Video_label.setPixmap(pixmap)
                self.client.video_flag = True
        except Exception as e:
            print(e)

    def stop(self):
        command = cmd.CMD_MOVE_STOP + "#" + str(self.slider_speed.value()) + '\n'
        self.client.send_data(command)

    def forward(self):
        command = cmd.CMD_MOVE_FORWARD + "#" + str(self.slider_speed.value()) + '\n'
        self.client.send_data(command)

    def backward(self):
        command = cmd.CMD_MOVE_BACKWARD + "#" + str(self.slider_speed.value()) + '\n'
        self.client.send_data(command)

    def left(self):
        command = cmd.CMD_MOVE_LEFT + "#" + str(self.slider_speed.value()) + '\n'
        self.client.send_data(command)

    def right(self):
        command = cmd.CMD_MOVE_RIGHT + "#" + str(self.slider_speed.value()) + '\n'
        self.client.send_data(command)

    def sonic(self):
        if self.Button_Sonic.text() == 'Sonic':
            self.timer_sonic.start(100)
            self.Button_Sonic.setText('Close')
        else:
            self.timer_sonic.stop()
            self.Button_Sonic.setText('Sonic')

    def getSonicData(self):
        command = cmd.CMD_SONIC + '\n'
        self.client.send_data(command)

    def calibration(self):
        self.client.send_data(cmd.CMD_CALIBRATION_MOD + '#' + '1' + '\n')
        self.calibrationWindow = CalibrationWindow(self.client, self.list_angle)
        self.calibrationWindow.setWindowModality(Qt.ApplicationModal)
        self.calibrationWindow.show()

    def add_user(self):
        command = cmd.CMD_ADD_USER + "#1" + '\n'
        self.client.send_data(command)
        self.add_user_window = AddUserWindow(self.client)
        self.add_user_window.setWindowModality(Qt.ApplicationModal)
        self.add_user_window.show()

    def hands(self):
        command = cmd.CMD_HANDS_UP + '\n'
        self.client.send_data(command)

    def play_music(self):
        command = cmd.CMD_MUSIC_PLAY + '\n'
        self.client.send_data(command)

    def stop_music(self):
        command = cmd.CMD_MUSIC_STOP + '\n'
        self.client.send_data(command)

    def fnc1(self):
        command = cmd.CMD_FUNC_1 + '\n'
        self.client.send_data(command)

    def fnc2(self):
        command = cmd.CMD_FUNC_2 + '\n'
        self.client.send_data(command)

    def fnc3(self):
        command = cmd.CMD_FUNC_3 + '\n'
        self.client.send_data(command)

    def fnc4(self):
        command = cmd.CMD_FUNC_4 + '\n'
        self.client.send_data(command)

    def lookstop(self):
        command = cmd.CMD_LOOK_STOP + '\n'
        self.client.send_data(command)
        
    def Menu(self):
        command = cmd.CMD_MENU + '\n'
        self.client.send_data(command)

    def ABOUT0(self):
        command = cmd.CMD_FUNC_ABOUT + '#0'+'\n'
        self.client.send_data(command)

    def ABOUT1(self):
        command = cmd.CMD_FUNC_ABOUT + '#1'+'\n'
        self.client.send_data(command)

    def ABOUT2(self):
        command = cmd.CMD_FUNC_ABOUT + '#2'+'\n'
        self.client.send_data(command)

    def ABOUT3(self):
        command = cmd.CMD_FUNC_ABOUT + '#3'+'\n'
        self.client.send_data(command)

    def ABOUT4(self):
        command = cmd.CMD_FUNC_ABOUT + '#4'+'\n'
        self.client.send_data(command)

    def ABOUT5(self):
        command = cmd.CMD_FUNC_ABOUT + '#5'+'\n'
        self.client.send_data(command)

    def RadioPlay(self):
        command = cmd.CMD_RADIO_PLAY + '\n'
        self.client.send_data(command)

    def RadioStop(self):
        command = cmd.CMD_RADIO_STOP + '\n'
        self.client.send_data(command)

    def VideoON(self):
        command = cmd.CMD_VIDEO + '#' + '1' + '\n'
        self.client.send_data(command)

    def VideoOFF(self):
        command = cmd.CMD_VIDEO + '#' + '0' + '\n'
        self.client.send_data(command)


class CalibrationWindow(QMainWindow, Ui_Calibration):
    def __init__(self, client, list_angle):
        super(CalibrationWindow, self).__init__()
        self.setupUi(self)
        self.client = client
        self.leg = ln.L
        self.list_angle = list_angle

        self.radioButton_left.setChecked(True)
        self.radioButton_left.toggled.connect(lambda: self.show_data(ln.L))
        self.radioButton_right.setChecked(False)
        self.radioButton_right.toggled.connect(lambda: self.show_data(ln.R))
        self.radioButton_handLeft.setChecked(False)
        self.radioButton_handLeft.toggled.connect(lambda: self.show_data(ln.H))
        self.radioButton_handRight.setChecked(False)
        self.radioButton_handRight.toggled.connect(lambda: self.show_data(ln.M))
        self.show_data(self.leg)
        self.button_A_more.clicked.connect(lambda: self.serv_angle_change(sn.A, 1))
        self.button_A_less.clicked.connect(lambda: self.serv_angle_change(sn.A, -1))
        self.button_B_more.clicked.connect(lambda: self.serv_angle_change(sn.B, 1))
        self.button_B_less.clicked.connect(lambda: self.serv_angle_change(sn.B, -1))
        self.button_C_more.clicked.connect(lambda: self.serv_angle_change(sn.C, 1))
        self.button_C_less.clicked.connect(lambda: self.serv_angle_change(sn.C, -1))
        self.button_D_more.clicked.connect(lambda: self.serv_angle_change(sn.D, 1))
        self.button_D_less.clicked.connect(lambda: self.serv_angle_change(sn.D, -1))
        self.button_E_more.clicked.connect(lambda: self.serv_angle_change(sn.E, 1))
        self.button_E_less.clicked.connect(lambda: self.serv_angle_change(sn.E, -1))
        self.button_end_calibration.clicked.connect(lambda: self.calibration_start_or_end(False))

    def show_data(self, leg_name):
        self.leg = leg_name
        if leg_name == ln.L:
            self.lineEdit_A.setText(self.list_angle[0][0])
            self.lineEdit_B.setText(self.list_angle[0][1])
            self.lineEdit_C.setText(self.list_angle[0][2])
            self.lineEdit_D.setText(self.list_angle[0][3])
            self.lineEdit_E.setText(self.list_angle[0][4])
        elif leg_name == ln.R:
            self.lineEdit_A.setText(self.list_angle[1][0])
            self.lineEdit_B.setText(self.list_angle[1][1])
            self.lineEdit_C.setText(self.list_angle[1][2])
            self.lineEdit_D.setText(self.list_angle[1][3])
            self.lineEdit_E.setText(self.list_angle[1][4])
        elif leg_name == ln.H:
            self.lineEdit_A.setText(self.list_angle[2][0])
            self.lineEdit_B.setText(self.list_angle[2][1])
            self.lineEdit_C.setText(self.list_angle[2][2])
            self.lineEdit_D.setText(self.list_angle[2][3])
            self.lineEdit_E.setText(self.list_angle[2][4])
        elif leg_name == ln.M:
            self.lineEdit_A.setText(self.list_angle[3][0])
            self.lineEdit_B.setText(self.list_angle[3][1])
            self.lineEdit_C.setText(self.list_angle[3][2])
            self.lineEdit_D.setText(self.list_angle[3][3])
            self.lineEdit_E.setText(self.list_angle[3][4])
            
    def serv_angle_change(self, serv_num, more_or_less):
        self.list_angle[self.leg.value][serv_num.value] = str(
            int(self.list_angle[self.leg.value][serv_num.value]) + more_or_less)
        self.show_data(self.leg)

        command = cmd.CMD_CALIBRATION + '#' + self.leg.name + '#' + serv_num.name + '#' + str(
            self.list_angle[self.leg.value][serv_num.value]) + '\n'
        self.client.send_data(command)

    def calibration_start_or_end(self, start_or_end):
        if start_or_end:
            command = cmd.CMD_CALIBRATION_MOD + '#' + '1' + '\n'
        else:
            command = cmd.CMD_CALIBRATION_MOD + '#' + '0' + '\n'
        self.client.send_data(command)


class AddUserWindow(QMainWindow, Ui_Add_user):
    def __init__(self, client):
        super(AddUserWindow, self).__init__()
        self.setupUi(self)
        self.client = client
        self.timer_video = QTimer(self)
        self.timer_video.timeout.connect(self.get_video_frame)
        self.timer_video.start(10)
        self.takePhotoButton.pressed.connect(self.take_photo)

    def get_video_frame(self):
        try:
            if not self.client.video_flag:
                height, width, bytesPerComponent = self.client.image.shape
                cv2.cvtColor(self.client.image, cv2.COLOR_BGR2RGB, self.client.image)
                QImg = QImage(self.client.image.data, width, height, 3 * width, QImage.Format_RGB888)
                self.Video_Label.setPixmap(QPixmap.fromImage(QImg))
                self.client.video_flag = True
        except Exception as e:
            print(e)

    def take_photo(self):
        if self.lineEdit.text() != '':
            command = cmd.CMD_TAKE_PHOTO + '#' + self.lineEdit.text() + '\n'
            self.client.send_data(command)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    myshow = MyWindow()
    myshow.show()
    sys.exit(app.exec_())
