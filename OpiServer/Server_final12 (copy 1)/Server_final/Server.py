import random
import socket
import struct
import threading
import time
import subprocess
import os

from PyQt5.QtGui import QPixmap

# import imutils
from FaceAction import *
from Service import *

# Настройка для повторного использования порта
socket.SO_REUSEPORT = 15
from UART import *
import cv2
from imutils.video import VideoStream
from DatabaseManager import *
from KeyPhraseAnalizer import *
from ServoControl import *
from Face import *

class Server:
    def __init__(self, outside_layout):

        self.outside_layout = outside_layout  # QLabel поверх слоя видео
        self.connection1 = None  # Соединение для клиента
        self.command = None  # Команда посылаемая на клиента
        self.video_flag = True  # включение видео трансляции (логика синхронизации)
        self.video_translation_flag = True  # передача видео на экран сервера
        self.tcp_flag = False  # флаг занятости передачи команд
        self.instruction_active_flag = False  # Флаг передачи команд с клиента
        self.frame_row = None  # сырой кадр
        self.neuro_talk_trig = False  # Флаг режима разговора с нейросетью
        self.about_me_trig = False  # Флаг режима рассказа о себе
        self.possibilities_trig = False  # Флаг режима рассказа о себе
        self.animation_active_trig = True
        self.video_transmit_trig = 0  # включение/выключение передачи видео на клиента
        self.active = False

        # Тексты о себе (сократил для читаемости кода, они остались те же)
        self.about_me_begin = 'Я являюсь робототехническим конструктором...'
        self.about_me_code = 'мой программный код состоит из трёх частей...'
        self.about_me_power = 'у меня есть 4 встроенных мощных аккумулятора...'
        self.about_me_chip = 'я использую 3 разных платы...'
        self.about_me_neural = 'я использую 5 разных нейросетей...'
        self.about_me_more = 'если вы хотите, то я могу рассказать...'
        self.about_me_exit = 'возвращаюсь в штатный режим...'
        self.meaning_of_live = 'я создана для обучения специалистов...'
        self.possibilities_last = 1
        self.possibilities = [
            'я могу выполнять не только простые задачи...',
            'я не полноценный искусственный интеллект...',
            'я могу поиграть с вами в разные игры...'
        ]
        self.possibilities_more = 'рассказать что я ещё умею?'
        self.possibilities_end = 'если вы захотите, чтобы я напомнила...'

        self.current_dir = os.path.dirname(__file__)
        self.servoControl = ServoControl()
        self.uart = UART()
        self.key_phrase_analyzer = KeyPhraseAnalizer()
        self.service = Service()
        self.db_manager = DatabaseManager('main_database.db')
        self.db_manager.create_user_table()
        self.db_manager.create_audio_table()

        # Инициализация музыки (важно для корректного старта)
        self.service.music_player.player_active_trig = False
        try:
            self.service.music_player.play_pause()
        except:
            pass

    def turn_on_server(self):
        """"Открытие сокетов"""
        HOST = self.get_internet_accessible_ip()
        if not HOST:
            HOST = '0.0.0.0' # Fallback
            
        # Порт 8001 для передачи видео
        self.server_socket = socket.socket()
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.server_socket.bind((HOST, 8001))
        except OSError:
             self.server_socket.bind(('', 8001)) # Попытка привязаться ко всем интерфейсам
        self.server_socket.listen(1)

        # Порт 5001 для приема и передачи инструкций
        self.server_socket1 = socket.socket()
        self.server_socket1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.server_socket1.bind((HOST, 5001))
        except OSError:
            self.server_socket1.bind(('', 5001))
        self.server_socket1.listen(1)
        
        print('Server address: ' + str(HOST))

    def turn_off_server(self):
        """Закрытие сервера"""
        try:
            if hasattr(self, 'connection'): self.connection.close()
            if hasattr(self, 'connection1'): self.connection1.close()
        except:
            print('\n' + "No client connection")

    def get_internet_accessible_ip(self):
        """Возвращает IP активного соединения"""
        try:
            temp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            temp_socket.connect(("8.8.8.8", 80))
            ip_address = temp_socket.getsockname()[0]
            temp_socket.close()
            return ip_address
        except Exception as e:
            print("Error getting IP:", e)
            return None

    def reset_server(self):
        """Перезагрузка сервера"""
        self.turn_off_server()
        self.turn_on_server()
        self.video = threading.Thread(target=self.transmission_video, daemon=True)
        self.instruction = threading.Thread(target=self.receive_instruction, daemon=True)
        self.video.start()
        self.instruction.start()

    def send_data(self, connect, data):
        """Безопасная передача данных"""
        try:
            if connect:
                connect.send(data.encode('utf-8'))
        except Exception as e:
            print(f"Send error: {e}")

    def translate_video_on_screen(self):
        """Захват видео с камеры"""
        # Попытка открыть камеру. Индекс 0 - дефолтный. Если не работает, попробуйте 1 или -1.
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        time.sleep(1.0) # Даем камере прогреться
        
        if not cap.isOpened():
            print("❌ ОШИБКА: Не удалось открыть камеру (index 0)!")
        
        while True:
            try:
                # Если GUI разрешил чтение следующего кадра (video_flag=True)
                if self.video_flag:
                    ret, frame = cap.read()
                    if ret:
                        self.frame_row = frame
                        # Обработка лица и рук
                        self.service.face_action.face_detect(self.frame_row)
                        self.service.hand.findpostion(self.frame_row)
                        self.frameManager()
                        
                        # Сообщаем GUI, что кадр готов, и ждем пока GUI его отрисует
                        self.video_flag = False 
                    else:
                        # Если кадр не считался, пробуем переоткрыть или подождать
                        print("Сбой чтения кадра")
                        time.sleep(0.1)

            except Exception as e:
                print(f"Ошибка в потоке камеры: {e}")
                time.sleep(1) # Пауза перед повторной попыткой
        
        cap.release()
        print("Видеопоток закрыт")

    def transmission_video(self):
        """Передача видеопотока по сокету на Android"""
        print("Ожидание подключения видео (порт 8001)...")
        try:
            self.connection, self.client_address = self.server_socket.accept()
            # Важно: создаем файловый объект без буферизации или с минимальной
            self.connection_file = self.connection.makefile('wb')
            self.video_transmit_trig = 1
            print(f"Видео соединение установлено с {self.client_address}!")
        except Exception as e:
            print(f"Ошибка подключения видео: {e}")
            return

        while True:
            try:
                # === ГЛАВНОЕ ИСПРАВЛЕНИЕ ===
                # 1. Проверяем, что кадр существует
                if self.frame_row is None:
                    time.sleep(0.01)
                    continue
                
                # 2. Логика флага: мы отправляем тот же кадр, что сейчас видит GUI.
                # GUI ставит video_flag=True когда забрал кадр.
                # Мы просто берем self.frame_row.
                # Чтобы не слать слишком часто (экономия сети), можно добавить sleep
                
                # Сжимаем в JPEG (качество 50-70 для скорости)
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
                result, frame = cv2.imencode('.jpg', self.frame_row, encode_param)
                
                if result:
                    data = frame.tobytes()
                    size = len(data)
                    
                    # Отправка размера (4 байта, unsigned int, little endian)
                    self.connection_file.write(struct.pack('<L', size))
                    self.connection_file.flush()
                    
                    # Отправка картинки
                    self.connection_file.write(data)
                    self.connection_file.flush()
                    
                    # Ограничение FPS для сети (~20 FPS)
                    time.sleep(0.05)
                
            except BrokenPipeError:
                print("Клиент отключился от видео.")
                break
            except Exception as e:
                print(f"Ошибка передачи кадра: {e}")
                # Не выходим из цикла, пробуем следующий кадр!
                time.sleep(0.5)

        print("Поток передачи видео завершен")
        try:
            self.connection.close()
        except: pass

    def receive_instruction(self):
        """Функция для приема и передачи инструкций"""
        print("Ожидание подключения команд (порт 5001)...")
        try:
            self.connection1, self.client_address1 = self.server_socket1.accept()
            print(f"Клиент команд подключен: {self.client_address1}")
            self.instruction_active_flag = True
        except:
            print("Client connect failed")
            return

        while True:
            try:
                allData = self.connection1.recv(1024).decode('utf-8')
                if not allData:
                    # Если пришли пустые данные, клиент отключился
                    print("Клиент команд отключился.")
                    self.reset_server()
                    break
                
                # Обработка "склеенных" команд
                cmdArray = allData.split('\n')
                
                for oneCmd in cmdArray:
                    if not oneCmd: continue
                    
                    data = oneCmd.split("#")
                    if not data or data[0] == '': continue

                    # === БЛОК КОМАНД ===
                    if cmd.CMD_SONIC in data:
                        dist = self.uart.getDistance() if self.uart.uart_connection_flag else "0"
                        self.send_data(self.connection1, f"{cmd.CMD_SONIC}#{dist}\n")
                        
                    elif cmd.CMD_CALIBRATION_MOD in data:
                        if len(data) > 1 and int(data[1]) == 0:
                            self.servoControl.saveToFile(self.servoControl.calibration_point, "angles")
                        self.uart.serial_send(oneCmd + "\n")
                        
                    elif cmd.CMD_CALIBRATION in data:
                        self.servoControl.setAngle(data)
                        self.uart.serial_send(oneCmd + "\n")
                        
                    elif cmd.CMD_CALIBRATION_ALL in data:
                        full_cmd = cmd.CMD_CALIBRATION_ALL + self.servoControl.getCalibrationCommand() + "\n"
                        self.send_data(self.connection1, full_cmd)
                        
                    elif cmd.CMD_UART in data:
                        status = self.UARTCheck()
                        self.send_data(self.connection1, f"{cmd.CMD_UART}#{status}\n")

                    # Движение
                    elif any(c in data for c in [cmd.CMD_MOVE_FORWARD, cmd.CMD_MOVE_BACKWARD, cmd.CMD_MOVE_LEFT, 
                                               cmd.CMD_MOVE_RIGHT, cmd.CMD_MOVE_STOP, cmd.CMD_HEAD, 
                                               cmd.CMD_HANDS_UP, cmd.CMD_CLENCH_LEFT, cmd.CMD_CLENCH_RIGHT,
                                               cmd.CMD_LOOK_UP, cmd.CMD_LOOK_DOWN, cmd.CMD_LOOK_STOP]):
                         if self.uart.uart_connection_flag:
                            self.uart.serial_send(oneCmd + "\n")

                    # Мультимедиа
                    elif cmd.CMD_MUSIC_PLAY in data:
                        self.service.music_player.player_active_trig = True
                        self.service.music_player.play()
                    elif cmd.CMD_MUSIC_PAUSE in data:
                        self.service.music_player.play_pause()
                    elif cmd.CMD_RADIO_PLAY in data:
                        self.service.radio_player.radio_active_trig = True
                        self.service.radio_player.play()
                    elif cmd.CMD_RADIO_STOP in data:
                        self.service.radio_player.pause()
                    
                    elif cmd.CMD_MENU in data:
                        # Сброс всех режимов
                        self.service.radio_player.pause()
                        self.service.music_player.stop()
                        self.service.video_player.play() # Stop logic inside
                        if hasattr(self.service, 'test_rps'): self.service.test_rps.play_game_trig = False
                        self.neuro_talk_trig = False
                        self.about_me_trig = False
                        self.possibilities_trig = False
                        self.outside_layout.clear()
                        self.animation_active_trig = True

                    elif cmd.CMD_FUNC_ABOUT in data:
                        try:
                            mode = int(data[1])
                            texts = {
                                1: self.about_me_begin, 2: self.about_me_code, 3: self.about_me_chip,
                                4: self.about_me_power, 5: self.about_me_neural
                            }
                            if mode in texts:
                                self.service.speach_rec.speak(texts[mode])
                                if mode > 1: self.service.speach_rec.speak(self.about_me_more)
                        except: pass

                    elif cmd.CMD_VIDEO_TRANSMISSION in data:
                        pass # Логика уже работает в другом потоке

            except Exception as e:
                print(f"Ошибка команд: {e}")
                break
        
        print("Поток инструкций закрыт")
        self.instruction_active_flag = False

    def takeCommand(self):
        """Распознавание голоса"""
        self.service.speach_rec.wishMe()
        while True:
            try:
                self.service.recognizer.listening()
                partial = self.service.recognizer.get_partial_phrase()
                
                if "избушка" in partial and not self.active:
                    self.service.recognizer.stop()
                    self.service.speach_rec.speak('Слушаю')
                    self.service.recognizer.start()
                    self.active = True
                
                full_phrase = self.service.recognizer.get_full_phrase()
                if full_phrase and full_phrase != "избушка":
                    self.recognizing(full_phrase)
            except Exception as e:
                print(f"Ошибка голоса: {e}")
                time.sleep(1)

    def recognizing(self, query):
        """Обработка команд голоса"""
        print(f"Распознано: {query}")
        query = query.lower().replace("избушка ", "")
        
        if 'стоп' in query:
            self.neuro_talk_trig = False
            self.about_me_trig = False
            self.service.music_player.stop()
            self.service.radio_player.pause()
            self.active = False
            return

        if self.active:
            key_phrase = self.key_phrase_analyzer.analyzer(query)
            
            if 'радио' in key_phrase:
                 self.service.radio_player.play_pause() # <--- Тут вызывается наш новый метод
                 
            elif 'о себе' in key_phrase:
                self.service.speach_rec.speak(self.about_me_begin)
                self.about_me_trig = True
            elif 'создана' in key_phrase:
                self.service.speach_rec.speak(self.meaning_of_live)
            elif 'погода' in query:
                self.service.weather_check(query)
            
            # ... (остальные команды как были) ...
            
            elif 'разговор' in key_phrase:
                self.neuro_talk_trig = True
                self.service.speach_rec.speak('Переключаюсь на нейросеть. Слушаю.')
            
            self.active = False

        # Обработка контекстных команд
        if self.service.radio_player.radio_active_trig:
            if 'пауза' in query: self.service.radio_player.play_pause()
            elif 'следующая' in query: self.service.radio_player.next_rad()
            
        elif self.neuro_talk_trig:
            if 'хватит' in query or 'стоп' in query:
                self.neuro_talk_trig = False
                self.service.speach_rec.speak('Хорошо.')
            else:
                answer = self.service.lets_talk(query)
                self.service.speach_rec.speak(answer)

    def UARTCheck(self):
        return str(self.uart.connect())

    def hold_3(self):
        """Анимация бездействия 3 мин"""
        if self.is_idle():
            anim = random.randint(1, 3)
            cmd_str = f"{cmd.CMD_HOLD_3}#{anim}\n"
            self.uart.serial_send(cmd_str)
            phrases = ["Хозяин где вы?", "Мне скучно...", "Эх, вот бы друга..."]
            self.service.speach_rec.speak(phrases[anim-1])

    def hold_10(self):
        """Анимация бездействия 10 мин"""
        if self.is_idle():
            anim = random.randint(1, 3)
            cmd_str = f"{cmd.CMD_HOLD_10}#{anim}\n"
            self.uart.serial_send(cmd_str)
            phrases = ["Забыли про меня?", "Можно погулять?", "Ручки не дотягиваются выключить..."]
            self.service.speach_rec.speak(phrases[anim-1])
            
    def is_idle(self):
        """Проверка, не занят ли робот"""
        return (not self.instruction_active_flag and 
                not self.service.music_player.player_active_trig and 
                not self.service.video_player.video_player_active_trig and 
                not self.neuro_talk_trig and
                not getattr(self.service, 'test_rps', lambda: None).play_game_trig)

    def frameManager(self):
        """Рассылка кадра модулям"""
        if self.frame_row is not None:
            # Для игры нужен ресайз, если она запущена
            if hasattr(self.service, 'test_rps'):
                self.service.test_rps.frame = self.frame_row
            if hasattr(self.service, 'face_action'):
                self.service.face_action.frame = cv2.resize(self.frame_row, (600, 480))
            
    def possibilities_next(self):
        next_num = random.randint(0, len(self.possibilities) - 1)
        while next_num == self.possibilities_last and len(self.possibilities) > 1:
            next_num = random.randint(0, len(self.possibilities) - 1)
        self.possibilities_last = next_num
        return next_num
