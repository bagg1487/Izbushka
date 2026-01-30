
import random
import socket
import struct
import threading
import time

from PyQt5.QtGui import QPixmap

# import imutils
from FaceAction import *
from Service import *

socket.SO_REUSEPORT = 15
from UART import *
import cv2
from imutils.video import VideoStream
from DatabaseManager import *
from KeyPhraseAnalizer import *
from ServoControl import *
from Face import *
import subprocess
# Класс сервера и все что с ним связано: открытия соединения, закрытия соединения, перезагрузки соединения,
# безопасной приема и передачи данных
#sudo fuser -k 5001/tcp && 8001/tcp

class Server:
    def __init__(self, outside_layout):

        self.outside_layout = outside_layout  # QLabel поверх слоя видео, на котором можно рисовать
        self.connection1 = None  # Соединение для клиента
        self.command = None  # Команда посылаемая на клиента или STM32
        self.video_flag = True  # включение видео трансляции
        self.video_translation_flag = True  # передача видео на экран сервера
        self.tcp_flag = False  # флаг занятости передачи команд
        self.instruction_active_flag = False  # Флаг передачи команд с клиента
        self.frame_row = None  # сырой кадр, который может транслироваться на экран или передаться по каналу
        self.neuro_talk_trig = False  # Флаг режима разговора с нейросетью
        self.about_me_trig = False  # Флаг режима рассказа о себе
        self.possibilities_trig = False  # Флаг режима рассказа о себе
        self.animation_active_trig = True
        self.video_transmit_trig = 0  # включение/выключение передачи видео на клиента
        self.active = False

        self.about_me_begin = 'Я являюсь робототехническим конструктором, созданным с использованием самых прогрессивных технологий для обучения будущих поколений инженеров разных направлений. И я рада своей миссии в этом мире! Могу рассказать вам подробнее о своём программном коде, разводке питания, платах, которые мною используются, а также о нейрос+етях, с которыми я умею работать. О чём Вам рассказать в первую очередь?'
        self.about_me_code = 'мой программный код состоит из трёх частей. Я умею ходить благодаря коду на языке C, ведь именно на нём работает моя плата с чипом STM 32. Также я умею взаимодействовать с окружающим миром с помощью клиент-серверного приложения, написанном на языке Пайтон'
        self.about_me_power = 'у меня есть 4 встроенных мощных аккумулятора, которые подключены к плате управления для безопасного заряда и разряда. Также во мне есть несколько понижающих преобразователей напряжения, чтобы все мои платы, датчики и сервомеханизмы функционировали без перегрузок'
        self.about_me_chip = 'я использую 3 разных платы. Первая – это плата с чипом STM32. Эти чипы сложны в программировании и используются в промышленной робототехнике. Вторая плата это orange pi, которая является более мощным аналогом популярной raspberry pi. Третья плата из линейки Arduino, она для начинающих пользователей, чтобы они могли легко научиться со мной взаимодействовать на программном уровне и шаг за шагом переходить к более сложным платам и задачам'
        self.about_me_neural = 'я использую 5 разных нейрос+етей, которые могут себя показать, если вы предложите мне поболтать или сыграть с Вами в игру камень ножницы бумага. Также я использую одну нейрос+еть для распознавания вашего голоса и речи, вторую для разговора, а третью вы слышите прямо сейчас, ведь она озвучивает мою речь'
        self.about_me_more = 'если вы хотите, то я могу рассказать о другой своей составляющей. Просто попросите меня рассказать о моём программном коде, разводке питания, микросхемах или нейрос+етях. Если вы хотите закончить, то скажите выход'
        self.about_me_exit = 'возвращаюсь в штатный режим. Если хотите, то мы можем сыграть во что-нибудь или поболтать'
        self.meaning_of_live = 'я создана для обучения специалистов в сферах робототехники, электроники и программирования. Моя основная задача – научить любого желающего конструированию, программированию, взаимодействию с искусственным интеллектом, а также всем сопутствующим навыкам. Вы научитесь очень многому, начиная от пайки схем питания, заканчивая созданием виртуальных серверов на операционной системе Linux'
        self.possibilities_last = 1
        self.possibilities = [
            'я могу выполнять не только простые задачи, по типу сказать сколько время, посчитать пример или рассказать анекдот, но также могу выполнить более сложные задачи. Например, включить вам музыку, найти нужное видео на рутьюбе или статью на википедии, а также сгенерировать QR-код, чтобы вы могли легко получить ссылку на видео или статью. А ещё могу включить вам мультик про моих сородичей из фольклора']
        self.possibilities.append(
            'я не полноценный искусственный интеллект, но я буду рада пообщаться с вами с помощью нейрос+ети. Просто скажите. Избушка, давай поболтаем')
        self.possibilities.append(
            'я могу поиграть с вами в разные игры. Например, мы можем сыграть в камень ножницы бумага')
        self.possibilities_more = 'рассказать что я ещё умею?'
        self.possibilities_end = 'если вы захотите, чтобы я напомнила что я умею, то просто спросите об этом ещё раз'

        self.current_dir = os.path.dirname(__file__)  # абсолютный путь до текущей директории
        self.servoControl = ServoControl()  # Подключение модуля для управления сервоприводами
        self.uart = UART()  # Подключение UART
        self.key_phrase_analyzer = KeyPhraseAnalizer()  # Подключение анализатора ключевых фраз
        self.service = Service()  # Подключение класса сервис
        # Пример использования класса
        self.db_manager = DatabaseManager('main_database.db')  # Подключение базы данных
        # Создание таблицы пользователей
        self.db_manager.create_user_table()
        # Создание таблицы аудиофайлов
        self.db_manager.create_audio_table()

        # self.new_face_recognition = Face()
        # self.new_face_recognition.trainImage()

        #self.service.face_action.train_all()
    def turn_on_server(self):
        """"Открытие 2 сокетов на адресе HOST и портах 8001 и  5001"""
        # ip address
        # HOST = "192.168.1.130"
        HOST = self.get_internet_accessible_ip()
        # Порт 8001 для передачи видео
        self.server_socket = socket.socket()
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((HOST, 8001))
        self.server_socket.listen(1)

        # Порт 5001 для приема и передачи инструкций
        self.server_socket1 = socket.socket()
        self.server_socket1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket1.bind((HOST, 5001))
        self.server_socket1.listen(1)
        print('Server address: ' + HOST)
        # пример работы с базой
        # self.db_manager.insert_user('Alexandr',blob_data)
        # blob = self.db_manager.GetBlob('test.wav')
        # self.db_manager.insert_audio_file(1, '2024-01-26', blob)

        self.service.music_player.player_active_trig = False
        self.service.music_player.play_pause()

    def turn_off_server(self):
        """Закрытие сервера и всех соединений"""
        try:
            self.connection.close()
            self.connection1.close()
        except:
            print('\n' + "No client connection")

    def get_internet_accessible_ip(self):
        """Возвращает IP активного соединения"""
        try:
            # Создаем временный сокет
            temp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            temp_socket.connect(("8.8.8.8", 80))  # Подключаемся к известному внешнему серверу
            ip_address = temp_socket.getsockname()[0]  # Получаем IP-адрес устройства
            temp_socket.close()  # Закрываем сокет
            return ip_address
        except Exception as e:
            print("Error:", e)
            return None

    def reset_server(self):
        """
        Перезагрузка сервера. Состоит из вызова методов выкл и вкл сервера,
        а так же создания потоков для передачи видео и инструкций
        """
        self.turn_off_server()
        self.turn_on_server()
        self.video = threading.Thread(target=self.transmission_video, daemon=True)
        self.instruction = threading.Thread(target=self.receive_instruction, daemon=True)
        self.video.start()
        self.instruction.start()

    def send_data(self, connect, data):
        """Передача данных по UART, обернутая в try except для безопасной передачи"""
        try:
            connect.send(data.encode('utf-8'))
            # print("send",data)
        except Exception as e:
            print(e)

    def translate_video_on_screen(self):
        """Трансляция видео на экран"""
        # Установить разрешение с помощью v4l2-ctl
        #subprocess.run(["v4l2-ctl", "--set-fmt-video=width=640,height=480"])

        # Запуск видеозахвата с установленным разрешением
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # vs = VideoStream(src=0,
        #                  framerate=60,resolution=(800, 480)).start()  # Запускает видеопоток с источника 20(/dev/video20) с фреймрейтом 25
        time.sleep(2.0)  # Задержка для инициализации
        while True:
            try:
                # Если анимация лица выключена или включена передача

                if self.video_flag:
                    #self.frame_row = vs.read()  # Считать кадр с потока
                    ret, self.frame_row = cap.read()
                    self.service.face_action.face_detect(self.frame_row)
                    self.service.hand.findpostion(self.frame_row)
                    self.frameManager()
                    self.video_flag = False

            except Exception as e:
                print("End transmit ... ")
                break
        print("Видеопоток закрыт")

    def transmission_video(self):
        """Передача видеопотока по сокету"""
        # Попытка установить соединение с клиентом и начать передачу видео
        try:
            # Принимаем соединение и создаем файловый объект для записи в двоичном режиме
            self.connection, self.client_address = self.server_socket.accept()
            self.connection = self.connection.makefile('wb')
            # Устанавливаем флаг трансляции видео в 1 и выводим сообщение об установленном соединении
            self.video_transmit_trig = 1
            print("Видео соединение установлено !")
        # Обработка исключений
        except:
            pass

        # Бесконечный цикл передачи видеопотока
        while True:
            try:
                # Если флаг видео установлен в False:
                if not self.video_flag:
                    # Кодируем кадр в формат JPEG и преобразуем в байты
                    frame = cv2.imencode('.jpg', self.frame_row)[1].tobytes()
                    lenFrame = len(frame)
                    lengthBin = struct.pack('<I', lenFrame)  # упаковываем длину кадра в бинарный формат
                    # Передаем длину кадра в бинарном виде
                    self.connection.write(lengthBin)
                    # Передаем сам кадр
                    self.connection.write(frame)
            except Exception as e:
                print("End transmit ... ")
                print(e)
                break

        print("Видеопоток закрыт")

    # Функция для приема и передачи инструкций в отдельном потоке instruction
    def receive_instruction(self):
        """Функция для приема и передачи инструкций в отдельном потоке instruction"""
        allData= None
        try:
            self.connection1, self.client_address1 = self.server_socket1.accept()
            print("Клиент успешно присоединен !")
            self.instruction_active_flag = True
        except:
            print("Client connect failed")
            self.server_socket1.close()
        while True:
            # Пытаемся принять информацию от клиента
            try:
                allData = self.connection1.recv(1024).decode('utf-8')
                print(allData)
            except Exception as e:
                print(e)
            # Проверяем не пуста ли пришедшая информация и стоит ли флаг передачи. Если пуста перезапускаем сервер
            if allData == "" and self.tcp_flag:
                self.reset_server()
                break
            else:
                cmdArray = allData.split('\n')  # Разделяет команды по разделителю \n (переход на новую строку)
                print(cmdArray)
                if cmdArray[-1] != "":
                    cmdArray == cmdArray[:-1]
            for oneCmd in cmdArray:
                data = oneCmd.split("#")
                if data == None or data[0] == '':
                    continue
                elif cmd.CMD_SONIC in data:  # Данные с датчика расстояния
                    if self.uart.uart_connection_flag:
                        self.command = cmd.CMD_SONIC + '#' + str(self.uart.getDistance()) + "\n"
                    self.send_data(self.connection1, self.command)
                elif cmd.CMD_CALIBRATION_MOD in data:  # Включение режима калибровки
                    if int(data[1]) == 0:
                        self.servoControl.saveToFile(self.servoControl.calibration_point, "angles")
                    self.uart.serial_send(oneCmd + "\n")
                elif cmd.CMD_CALIBRATION in data:  # Управление сервоприводами
                    self.servoControl.setAngle(data)
                    self.uart.serial_send(oneCmd + "\n")
                elif cmd.CMD_CALIBRATION_ALL in data:
                    self.command = cmd.CMD_CALIBRATION_ALL + self.servoControl.getCalibrationCommand() + "\n"
                    self.send_data(self.connection1, self.command)
                elif cmd.CMD_UART in data:
                    self.send_data(self.connection1, cmd.CMD_UART + "#" + self.UARTCheck())
                elif cmd.CMD_MOVE_FORWARD in data:  # Идти вперед
                    if self.uart.uart_connection_flag:
                        self.uart.serial_send(cmd.CMD_MOVE_FORWARD + "\n")
                elif cmd.CMD_MOVE_BACKWARD in data:  # Идти назад
                    if self.uart.uart_connection_flag:
                        self.uart.serial_send(cmd.CMD_MOVE_BACKWARD + "\n")
                elif cmd.CMD_MOVE_LEFT in data:  # Идти влево
                    if self.uart.uart_connection_flag:
                        self.uart.serial_send(cmd.CMD_MOVE_LEFT + "\n")
                elif cmd.CMD_MOVE_RIGHT in data:  # Идти вправо
                    if self.uart.uart_connection_flag:
                        self.uart.serial_send(cmd.CMD_MOVE_RIGHT + "\n")
                elif cmd.CMD_MOVE_STOP in data:  # остановиться
                    if self.uart.uart_connection_flag:
                        self.uart.serial_send(cmd.CMD_MOVE_STOP + "\n")
                elif cmd.CMD_HANDS_UP in data:  # Помахать
                    if self.uart.uart_connection_flag:
                        self.uart.serial_send(cmd.CMD_HANDS_UP + "\n")
                elif cmd.CMD_CLENCH_LEFT in data:  # закрыть/открыть пальцы на левой руке
                    if self.uart.uart_connection_flag:
                        self.uart.serial_send(cmd.CMD_CLENCH_LEFT + "\n")
                elif cmd.CMD_CLENCH_RIGHT in data:  # закрыть/открыть пальцы на правой руке
                    if self.uart.uart_connection_flag:
                        self.uart.serial_send(cmd.CMD_CLENCH_RIGHT + "\n")
                elif cmd.CMD_LOOK_UP in data:  # Посмотреть вверх
                    if self.uart.uart_connection_flag:
                        self.uart.serial_send(cmd.CMD_LOOK_UP + "\n")
                elif cmd.CMD_LOOK_DOWN in data:  # Посмотреть вниз
                    if self.uart.uart_connection_flag:
                        self.uart.serial_send(cmd.CMD_LOOK_DOWN + "\n")
                elif cmd.CMD_LOOK_STOP in data:  # Остановить движение
                    if self.uart.uart_connection_flag:
                        self.uart.serial_send(cmd.CMD_LOOK_STOP + "\n")
                elif cmd.CMD_MUSIC_PLAY in data:  # Включить музыкальный плеер
                    if not self.service.music_player.player_active_trig:
                        self.service.music_player.player_active_trig = True
                        self.service.music_player.play()
                elif cmd.CMD_MUSIC_PAUSE in data:  # Выключить музыкальный плеер
                    try:
                        self.service.music_player.play_pause()
                        self.service.music_player.player_active_trig = False
                    except:
                        print("Can't stop music")
                elif cmd.CMD_RADIO_PLAY in data:  # Включить радио плеер
                    self.service.radio_player.radio_active_trig = True
                    self.service.radio_player.play()
                elif cmd.CMD_RADIO_STOP in data:  # Выключить радио плеер
                    self.service.radio_player.pause()
                    self.service.radio_player.radio_active_trig = False
                elif cmd.CMD_MENU in data:  # Выход в меню с отменой всех основных функций
                    self.service.radio_player.radio_active_trig = False
                    self.service.music_player.player_active_trig = False
                    self.service.music_player.stop()
                    self.service.test_rps.play_game_trig = False
                    self.service.video_player.video_player_active_trig = False
                    self.service.video_player.play()
                    self.neuro_talk_trig = False  # вкл/выкл режима разговора с нейросетью
                    self.about_me_trig = False  # вкл/выкл режима рассказа о себе
                    self.possibilities_trig = False  # вкл/выкл режима рассказа о себе
                    self.outside_layout.clear()
                # self.service.clear_qr(self.outside_layout)
                    self.animation_active_trig = True
                elif cmd.CMD_FUNC_ABOUT in data:  # Раскажи о себе
                    try:
                        about_mode = int(data[1])
                    except:
                        about_mode = 1
                    if about_mode == 1:
                        self.service.speach_rec.speak(self.about_me_begin)
                        # self.about_me_trig = True
                    elif about_mode == 2:
                        self.service.speach_rec.speak(self.about_me_code)
                        self.service.speach_rec.speak(self.about_me_more)
                    elif about_mode == 3:
                        self.service.speach_rec.speak(self.about_me_chip)
                        self.service.speach_rec.speak(self.about_me_more)
                    elif about_mode == 4:
                        self.service.speach_rec.speak(self.about_me_power)
                        self.service.speach_rec.speak(self.about_me_more)
                    elif about_mode == 5:
                        self.service.speach_rec.speak(self.about_me_neural)
                        self.service.speach_rec.speak(self.about_me_more)
                    else:
                        self.about_me_trig = False

                elif cmd.CMD_VIDEO_TRANSMISSION in data:  # Управление передачей видео
                    self.video_transmit_trig = int(data[1])
            # self.sendRelaxFlag()
        print("Поток инструкций закрыт")
        self.instruction_active_flag = False

    def takeCommand(self):
        """Функция распознавания голоса """
        self.service.speach_rec.wishMe()
        while True:
            self.service.recognizer.listening()
            if self.service.recognizer.get_partial_phrase() == "избушка" and not self.active:
                self.service.recognizer.stop()
                self.service.speach_rec.speak('Слушаю')
                self.service.recognizer.start()
                self.active = True
            full_phrase = self.service.recognizer.get_full_phrase()
            if full_phrase != '' and full_phrase != "избушка":
                self.recognizing(full_phrase)

    def recognizing(self, query):
        """Функция определения команд и выполнения соответствующего действия"""
        # выполнение задач в соответствии с запросом
        print(query)
        query = query.lower()
        query = query.replace("избушка ", "")
        if 'стоп' in query:  # функция полной остановки текущих задач/приложений
            self.neuro_talk_trig = False
            self.about_me_trig = False
            self.service.music_player.stop()
            self.service.music_player.player_active_trig = False
            self.active = False
        if self.active:
            # Распознавание ключевой фразы в распознаном тексте

            key_phrase = self.key_phrase_analyzer.analyzer(query)
            print(key_phrase)
            # Варианты действий в зависимости от распознаного текста или ключевой фразы
            if 'о себе' in key_phrase:
                self.service.speach_rec.speak(self.about_me_begin)
                self.about_me_trig = True
            elif 'создана' in key_phrase:
                self.service.speach_rec.speak(self.meaning_of_live)
            elif 'что ты умеешь' in key_phrase:
                self.service.speach_rec.speak(self.possibilities[0])
                self.possibilities_trig = True
                self.service.speach_rec.speak(self.possibilities_more)
            elif 'включи камеру' in query:
                self.animation_active_trig = False
                self.outside_layout.clear()
            elif 'выключи камеру' in query:
                self.animation_active_trig = True
                self.service.face_action.face_detection_trig = False

            elif 'вперед' in query:
                self.uart.serial_send(cmd.CMD_MOVE_FORWARD + "\n")
                time.sleep(2)
                self.uart.serial_send(cmd.CMD_MOVE_STOP + "\n")
                print(cmd.CMD_MOVE_FORWARD + "\n")
            elif 'назад' in query:
                self.uart.serial_send(cmd.CMD_MOVE_BACKWARD + "\n")
                time.sleep(2)
                self.uart.serial_send(cmd.CMD_MOVE_STOP + "\n")
                print(cmd.CMD_MOVE_BACKWARD + "\n")
            elif 'влево' in query:
                self.uart.serial_send(cmd.CMD_MOVE_LEFT + "\n")
                time.sleep(2)
                self.uart.serial_send(cmd.CMD_MOVE_STOP + "\n")
                print(cmd.CMD_MOVE_LEFT + "\n")
            elif 'вправо' in query:
                self.uart.serial_send(cmd.CMD_MOVE_RIGHT + "\n")
                time.sleep(2)
                self.uart.serial_send(cmd.CMD_MOVE_STOP + "\n")
                print(cmd.CMD_MOVE_RIGHT + "\n")
            elif 'добавить' in key_phrase:
                new_user = self.service.face_action.add_user(self.service.recognizer)
                self.db_manager.insert_user(new_user)
            elif 'запусти распознавание' in query:
                self.service.face_action.face_detection_trig = True
            elif 'отключи распознавание' in query:
                self.service.face_action.face_detection_trig = False
            elif 'игра' in key_phrase:
                self.service.game_mode(key_phrase)
            elif 'очистить' in key_phrase:
                self.outside_layout.clear()
                # self.service.clear_qr(self.outside_layout)
                self.animation_active_trig = True
            elif 'википедия' in key_phrase:  # если wikipedia встречается в запросе, выполнится блок:
                self.service.wikisearch(query)
            elif 'музыка' in key_phrase and not self.service.radio_player.radio_active_trig:
                self.service.music_player.player_active_trig = True
                self.service.music_player.play()
            elif 'славянка' in key_phrase and not self.service.radio_player.radio_active_trig:
                self.service.music_player.player_active_trig = True
                self.service.music_player.play(0)
                self.uart.serial_send(cmd.CMD_HANDS_UP + "\n")
            elif 'найди видео' in key_phrase and ('youtube' in key_phrase or 'vk' in key_phrase or 'rutube' in key_phrase) :
                self.service.play_video.find_video_browser(query, key_phrase)
            elif 'абба' in key_phrase and not 'игра' in key_phrase and not 'радио' in key_phrase:
                self.service.video_player.video_player_active_trig = True
                self.service.video_player.play_first("abba.mp4")
            elif 'найди видео' in key_phrase and not 'игра' in key_phrase and not 'радио' in key_phrase:
                self.service.video_player.video_player_active_trig = True
                self.service.video_player.play_first()
            elif 'код' in key_phrase:
                self.animation_active_trig = False
                self.outside_layout.clear()
                self.service.makeQR("я избушка, обо мне читай здесь: ios.ru", self.outside_layout)
            elif 'радио' in key_phrase and not self.service.music_player.player_active_trig:
                self.service.radio_player.radio_active_trig = True
                self.service.radio_player.play_pause()
            elif 'анекдот' in key_phrase and not self.service.music_player.player_active_trig:
                # self.service.music_player.player_active_trig = True
                self.service.joke()
            elif 'время' in key_phrase:
                self.service.time_now()
            elif 'посчитай' in query:
                self.service.calculate(query)
            elif 'разговор' in key_phrase:
                self.neuro_talk_trig = True
                self.service.speach_rec.speak('Да, хорошо, я Вас слушаю')
            else:
                self.service.speach_rec.speak('Плохо вас слышу, повторите!!!')

            self.active = False
        if self.service.radio_player.radio_active_trig:
            if 'пауза' in query:
                # self.service.music_player.player_active_trig = True
                self.service.radio_player.pause()
                self.service.radio_player.radio_active_trig = False
            if 'следующая' in query:
                self.service.radio_player.next_rad()
                # self.service.radio_player.play()
        elif self.service.music_player.player_active_trig:
            if 'включи' in query and 'трек' in query:
                self.service.music_player.play_pause()
            elif 'следующий' in query and 'трек' in query:
                self.service.music_player.next_track()
            elif 'предыдущий' in query and 'трек' in query:
                self.service.music_player.previous_track()
            elif 'выключи' in query and 'трек' in query:
                self.service.music_player.pause()
            elif 'выключи' in query and 'музыку' in query:
                self.service.music_player.stop()
                self.service.music_player.player_active_trig = False
        elif self.service.video_player.video_player_active_trig:
            if 'выключи' in query and 'видео' in query:
                self.service.video_player.video_player_active_trig = False
                self.service.video_player.play()


        if self.neuro_talk_trig:  # если мы болтаем, то другие команды не сработают
                self.service.video_player.play()

        if self.neuro_talk_trig:  # если мы болтаем, то другие команды не сработают
            if ('закончим' in query and 'болтать' in query) or 'стоп' in query:
                self.neuro_talk_trig = False
                self.service.speach_rec.speak('хорошо. Зовите, когда захотите ещё поболтать')
            else:
                answer = self.service.lets_talk(query)
                # print("мой ответ: " + answer)
                self.service.speach_rec.speak(answer)

        elif self.about_me_trig:  # если избушка рассказывает о себе, то другие команды не сработают
            if 'код' in query:
                self.service.speach_rec.speak(self.about_me_code)
                self.service.speach_rec.speak(self.about_me_more)
            elif 'плат' in query:
                self.service.speach_rec.speak(self.about_me_chip)
                self.service.speach_rec.speak(self.about_me_more)
            elif 'питания' in query:
                self.service.speach_rec.speak(self.about_me_power)
                self.service.speach_rec.speak(self.about_me_more)
            elif 'нейро' in query:
                self.service.speach_rec.speak(self.about_me_neural)
                self.service.speach_rec.speak(self.about_me_more)
            elif 'выход' in query:
                self.about_me_trig = False

        elif self.possibilities_trig:
            if 'ещё' in query or 'да' in query:
                self.service.speach_rec.speak(self.possibilities[self.possibilities_next()])
                self.service.speach_rec.speak(self.possibilities_more)
            elif 'выход' in query:
                self.possibilities_trig = False
                self.service.speach_rec.speak(self.possibilities_end)

    def UARTCheck(self):
        """Подключение UART"""
        self.uart.connect()

    def hold_3(self):
        """
        Анимации ожидания через 3 минуты. Проверяется не активен ли один из модулей прерывающих ожидание.
        Случайным образом выбирается одна из трех анимаций.
        """
        if not self.instruction_active_flag and not self.service.music_player.player_active_trig and \
                not self.service.video_player.video_player_active_trig and not self.neuro_talk_trig and \
                not self.service.test_rps.play_game_trig:
            print("check")
            anim = random.randint(1, 3)
            if anim == 1:
                self.uart.serial_send(cmd.CMD_HOLD_3 + "#1" + "\n")
                self.service.speach_rec.speak("Хозяин где вы? Может поиграем или поболтаем?")
            elif anim == 2:
                self.uart.serial_send(cmd.CMD_HOLD_3 + "#2" + "\n")
                self.service.speach_rec.speak("Мне скучно, вот бы кто со мной поговорил.")
            else:
                self.uart.serial_send(cmd.CMD_HOLD_3 + "#3" + "\n")
                self.service.speach_rec.speak("Эх вот бы мне друга. Хочу с другой избушкой погулять, как в том фильме")

    def hold_10(self):
        """
        Анимации ожидания через 10 минут. Проверяется не активен ли один из модулей прерывающих ожидание.
        Случайным образом выбирается одна из трех анимаций.
        """
        if not self.instruction_active_flag and not self.service.music_player.player_active_trig and \
                not self.service.video_player.video_player_active_trig and not self.neuro_talk_trig and \
                not self.service.test_rps.play_game_trig:
            anim = random.randint(1, 3)
            if anim == 1:
                self.uart.serial_send(cmd.CMD_HOLD_10 + "#1" + "\n")
                self.service.speach_rec.speak("Неужели все про меня забыли?")
            elif anim == 2:
                self.service.speach_rec.speak("Интересно можно ли мне пройти вперед?")
                self.uart.serial_send(cmd.CMD_HOLD_10 + "#2" + "\n")
                time.sleep(1000)
                self.service.speach_rec.speak(
                    "Пожалуй, не буду двигаться, а то вдруг хозяину не понравится, что я ушла")
            else:
                self.uart.serial_send(cmd.CMD_HOLD_10 + "#3" + "\n")
                self.service.speach_rec.speak(
                    "Как жаль, мои ручки не могут дотянуться до кнопки выключения, а то я с удовольствием бы отдохнула")

    def frameManager(self):
        """Функция отправки текущего кадра с камеры в соответствующие модули"""
        if self.frame_row is not None:
            self.service.face_action.frame = cv2.resize(self.frame_row, (600, 480))
            self.service.test_rps.frame = self.frame_row

    def possibilities_next(self):
        next_num = random.randint(1, len(self.possibilities) - 1)
        while next_num == self.possibilities_last:
            next_num = random.randint(1, len(self.possibilities) - 1)
        self.possibilities_last = next_num
        return next_num
