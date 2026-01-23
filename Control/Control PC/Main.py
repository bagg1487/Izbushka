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


class MyWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        """ Инициализация переменных для окон"""
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
        #self.About_me.clicked.connect(self.about)
        self.func1.clicked.connect(self.fnc1)
        self.func2.clicked.connect(self.fnc2)
        #self.func3.clicked.connect(self.fnc3)
        #self.func4.clicked.connect(self.fnc4)

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

    def connect(self):
        """событие по нажатию кнопки Connect"""
        self.lineEdit.text()
        file = open('IP.txt', 'w')  # Открыть файл на запись
        file.write(self.lineEdit.text())  # записать ip  из поля
        file.close()  # закрыть файл
        pattern = r'^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$'

        if self.pushButton.text() == 'Connect':  # Если надпись Connect
            if re.match(pattern, self.lineEdit.text()):
                self.IP = self.lineEdit.text()  # получаем ip адрес из поля

                self.client.turn_on_client(
                    self.IP)  # функция которая устанавливает 2 соединения по IP. client_socket1 для передачи инструкций и client_socket для приема видео
                self.video = threading.Thread(target=self.client.receiving_video,
                                              args=(self.IP,))  # Создается поток для приема видео
                self.instruction = threading.Thread(target=self.receive_instruction,
                                                    args=(self.IP,))  # Создается поток для передачи инструкций
                self.video.start()  # Запуск потока
                self.instruction.start()  # Запуск потока
                self.pushButton.setText('Disconnect')  # смена названия кнопки
                self.label_err.setText(" ")
            else:
                self.label_err.setText("Ошибка ввода ip")

            # self.timer_power.start(1000) #
        else:
            self.client.stop_video_thread = True
            self.client.stop_instruction_thread = True
            self.client.tcp_flag = False
            self.client.turn_off_client()
            self.pushButton.setText('Connect')
            # self.timer_power.stop()

    def receive_instruction(self, ip):
        """Открытие потока приема команд"""
        try:
            # Подключаемся к серверу по адресу ip и порту 5001
            self.client.client_socket1.connect((ip, 5001))
            self.client.tcp_flag = True  # Устанавливаем флаг TCP-соединения как активный
            print("Connection Successful !")  # Выводим сообщение об успешном подключении
            # Отправляем команду на сервер для калибровки всех устройств
            self.client.send_data(cmd.CMD_CALIBRATION_ALL + '\n')
        except Exception as e:
            print(
                "Connect to server Faild!: Server IP is right? Server is opend?")  # Выводим сообщение об ошибке подключения
            self.client.tcp_flag = False  # Устанавливаем флаг TCP-соединения как неактивный
        while True:
            try:
                # Получаем данные от сервера
                alldata = self.client.receive_data()
            except:
                self.client.tcp_flag = False  # Устанавливаем флаг TCP-соединения как неактивный
                break
            print(alldata)
            if alldata == '':
                break
            else:
                cmdArray = alldata.split('\n')  # Разбиваем полученные данные на массив по символу новой строки
                print(cmdArray)
                if cmdArray[-1] != "":
                    cmdArray == cmdArray[:-1]
            for oneCmd in cmdArray:
                data = oneCmd.split("#")  # Разбиваем команду на части по символу '#'
                if data == "":
                    self.client.tcp_flag = False  # Устанавливаем флаг TCP-соединения как неактивный
                    break
                elif data[0] == cmd.CMD_SONIC:
                    self.Button_Sonic.setText(
                        data[1] + 'cm')  # Обновляем текст кнопки с информацией об обнаруженном препятствии
                elif data[0] == cmd.CMD_CALIBRATION_ALL:
                    # Обновляем список углов калибровки для всех устройств
                    self.list_angle = []
                    self.list_angle.append(data[1:6])
                    self.list_angle.append(data[6:11])
                    self.list_angle.append(data[11:16])
                    self.list_angle.append(data[16:21])
                else:
                    pass  # Пропускаем команды, для которых нет обработчика

    def get_video_frame(self):
        try:
            if self.client.video_flag == False:
                # Получаем высоту, ширину и количество байт на компоненту изображения
                height, width, bytesPerComponent = self.client.image.shape
                # Конвертируем изображение из формата BGR в RGB
                cv2.cvtColor(self.client.image, cv2.COLOR_BGR2RGB, self.client.image)
                # Создаем объект QImage из данных изображения
                QImg = QImage(self.client.image.data, width, height, 3 * width, QImage.Format_RGB888)
                # Создаем объект QPixmap из QImage и масштабируем его до размеров 400x240
                pixmap = QPixmap.fromImage(QImg).scaled(390, 240)
                # Устанавливаем QPixmap в виджет QLabel для отображения видеопотока
                self.Video_label.setPixmap(pixmap)
                self.client.video_flag = True  # Устанавливаем флаг видео как активный
        except Exception as e:
            print(e)  # Выводим ошибку, если она возникает

    def stop(self):
        command = cmd.CMD_MOVE_STOP + "#" + str(self.slider_speed.value()) + '\n'
        self.client.send_data(command)
        # print (command)

    def forward(self):
        command = cmd.CMD_MOVE_FORWARD + "#" + str(self.slider_speed.value()) + '\n'
        # command = cmd.CMD_SONIC + '\n'
        self.client.send_data(command)
        # print (command)

    def backward(self):
        command = cmd.CMD_MOVE_BACKWARD + "#" + str(self.slider_speed.value()) + '\n'
        self.client.send_data(command)
        # print (command)

    def left(self):
        command = cmd.CMD_MOVE_LEFT + "#" + str(self.slider_speed.value()) + '\n'
        self.client.send_data(command)
        # print (command)

    def right(self):
        command = cmd.CMD_MOVE_RIGHT + "#" + str(self.slider_speed.value()) + '\n'
        self.client.send_data(command)
        # print (command)

    def sonic(self):
        if self.Button_Sonic.text() == 'Sonic':
            self.timer_sonic.start(100)
            self.Button_Sonic.setText('Close')

        else:
            self.timer_sonic.stop()
            self.Button_Sonic.setText('Sonic')
            #

    def getSonicData(self):
        command = cmd.CMD_SONIC + '\n'
        self.client.send_data(command)
        # print (command)

    def calibration(self):
        # self.stop()
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
        """Инициализация интерфейса"""
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
        """Отображение данных о текущих углах калибровки для выбранной ноги"""
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
        """Изменение угла сервопривода и отправка команды на сервер"""
        # self.get_point()
        self.list_angle[self.leg.value][serv_num.value] = str(
            int(self.list_angle[self.leg.value][serv_num.value]) + more_or_less)
        self.show_data(self.leg)

        command = cmd.CMD_CALIBRATION + '#' + self.leg.name + '#' + serv_num.name + '#' + str(
            self.list_angle[self.leg.value][serv_num.value]) + '\n'
        self.client.send_data(command)
        # print(command)

    def calibration_start_or_end(self, start_or_end):
        """Запуск или завершение процесса калибровки"""
        if start_or_end:
            command = cmd.CMD_CALIBRATION_MOD + '#' + '1' + '\n'
        else:
            command = cmd.CMD_CALIBRATION_MOD + '#' + '0' + '\n'
        self.client.send_data(command)


class AddUserWindow(QMainWindow, Ui_Add_user):
    def __init__(self, client):
        """Инициализация интерфейса"""
        super(AddUserWindow, self).__init__()
        # Инициализация интерфейса
        self.setupUi(self)
        # Передача экземпляра клиента
        self.client = client
        # Настройка таймера для получения видеокадров и его запуск
        self.timer_video = QTimer(self)
        self.timer_video.timeout.connect(self.get_video_frame)
        self.timer_video.start(10)
        # Привязка обработчика события к кнопке для съемки фото
        self.takePhotoButton.pressed.connect(self.take_photo)

    def get_video_frame(self):
        """Получение кадра трансляции"""
        try:
            if not self.client.video_flag:
                # Получение высоты, ширины и количества байт на компоненту изображения
                height, width, bytesPerComponent = self.client.image.shape
                # Конвертация изображения из формата BGR в RGB
                cv2.cvtColor(self.client.image, cv2.COLOR_BGR2RGB, self.client.image)
                # Создание объекта QImage из данных изображения
                QImg = QImage(self.client.image.data, width, height, 3 * width, QImage.Format_RGB888)
                # Установка изображения в виджет QLabel
                self.Video_Label.setPixmap(QPixmap.fromImage(QImg))
                self.client.video_flag = True
        except Exception as e:
            print(e)

    def take_photo(self):
        """Съемка фото при нажатии кнопки"""
        if self.lineEdit.text() != '':
            command = cmd.CMD_TAKE_PHOTO + '#' + self.lineEdit.text() + '\n'
            self.client.send_data(command)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    myshow = MyWindow()
    myshow.show()
    sys.exit(app.exec_())
