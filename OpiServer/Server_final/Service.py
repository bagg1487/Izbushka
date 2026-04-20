from datetime import datetime
import segno
from PyQt5.QtCore import QPoint, Qt
from PyQt5.QtGui import QImage, QPainter, QColor, QPixmap
from wikipedia import wikipedia
from DatabaseManager import *
from HandAndFingerDetectionModule import *
from MusicPlayer import *
from GeminiAssistant import GeminiAssistant
from RadioPlayer import *
from FaceAction import *
from PlayVideoViaBrowser import *
from VideoPlayer import *
from KeyPhraseAnalizer import *
from Recognizer import *
from Speaker import Speaker
from enum import Enum


class LEDMode(Enum):
    RADIO = "radio"
    ABOUT_ME = "about_me"
    VIDEO = "video"
    MUSIC = "music"
    NEURO_TALK = "neuro_talk"
    IDLE = "idle"


class LEDController:
    COLORS = {
        LEDMode.RADIO: (0, 255, 0),
        LEDMode.ABOUT_ME: (255, 0, 0),
        LEDMode.VIDEO: (0, 0, 255),
        LEDMode.MUSIC: (255, 255, 0),
        LEDMode.NEURO_TALK: (255, 0, 255),
        LEDMode.IDLE: (128, 128, 128)
    }

    def __init__(self, uart_instance):
        self.uart = uart_instance
        self.active_modes = set()
        self.current_color = (0, 0, 0)

    def add_mode(self, mode):
        self.active_modes.add(mode)
        self._update_led()

    def remove_mode(self, mode):
        if mode in self.active_modes:
            self.active_modes.remove(mode)
        self._update_led()

    def _update_led(self):
        if not self.active_modes:
            self._set_color(LEDMode.IDLE)
        elif len(self.active_modes) == 1:
            mode = next(iter(self.active_modes))
            self._set_color(mode)
        else:
            self._mix_colors()

    def _set_color(self, mode):
        color = self.COLORS.get(mode, (255, 255, 255))
        if color != self.current_color:
            self.current_color = color
            self._send_led_command(*color)

    def _mix_colors(self):
        mixed = [0, 0, 0]
        for mode in self.active_modes:
            color = self.COLORS.get(mode, (0, 0, 0))
            for i in range(3):
                mixed[i] = min(255, mixed[i] + color[i])
        mixed = tuple(255 if v > 127 else 0 for v in mixed)
        if mixed != self.current_color:
            self.current_color = mixed
            self._send_led_command(*mixed)

    def _send_led_command(self, r, g, b):
        if self.uart and self.uart.uart_connection_flag:
            command = f"CMD_LED#{r}#{g}#{b}"
            self.uart.serial_send(command)


class Service:
    def __init__(self, uart_instance=None):
        self.speach_rec = Speaker()
        self.face_action = FaceAction()
        self.hand = HandAndFingerDetectionModule()
        self.music_player = MusicPlayer()
        self.video_player = VideoPlayer()
        self.radio_player = RadioPlayer()
        self.recognizer = Recognizer()
        self.db_manager = DatabaseManager('main_database.db')
        self.db_manager.create_user_table()
        self.db_manager.create_audio_table()
        self.play_video = PlayVideoViaBrowser()
        self.key_phrase_analizer = KeyPhraseAnalizer()
        self.os_name = platform.system()
        self.ai_assistant = GeminiAssistant()
        self.led_controller = LEDController(uart_instance)
        self.radio_active = False
        self.about_me_active = False
        self.video_active = False
        self.music_active = False

    def start_radio(self):
        if not self.radio_active:
            self.radio_active = True
            self.led_controller.add_mode(LEDMode.RADIO)
            self.radio_player.radio_active_trig = True
            self.radio_player.play()

    def stop_radio(self):
        if self.radio_active:
            self.radio_active = False
            self.led_controller.remove_mode(LEDMode.RADIO)
            self.radio_player.pause()
            self.radio_player.radio_active_trig = False

    def start_music(self):
        if not self.music_active:
            self.music_active = True
            self.led_controller.add_mode(LEDMode.MUSIC)
            self.music_player.player_active_trig = True
            self.music_player.play()

    def stop_music(self):
        if self.music_active:
            self.music_active = False
            self.led_controller.remove_mode(LEDMode.MUSIC)
            self.music_player.stop()
            self.music_player.player_active_trig = False

    def start_video(self):
        if not self.video_active:
            self.video_active = True
            self.led_controller.add_mode(LEDMode.VIDEO)
            self.video_player.video_player_active_trig = True
            self.video_player.play_first()

    def stop_video(self):
        if self.video_active:
            self.video_active = False
            self.led_controller.remove_mode(LEDMode.VIDEO)
            self.video_player.video_player_active_trig = False
            self.video_player.close_event()

    def start_about_me(self):
        if not self.about_me_active:
            self.about_me_active = True
            self.led_controller.add_mode(LEDMode.ABOUT_ME)

    def stop_about_me(self):
        if self.about_me_active:
            self.about_me_active = False
            self.led_controller.remove_mode(LEDMode.ABOUT_ME)

    def weather_check(self, query):
        text = self.ai_assistant.get_weather(query)
        print(text)
        self.speach_rec.speak(text)

    def lets_talk(self, data):
        if "погод" in data.lower():
            return self.ai_assistant.get_weather(data)
        answer_text = self.ai_assistant.ask(data)
        return answer_text

    def get_face_frame(self):
        if self.face_action.face_detection_trig:
            all_users_BLOB = self.db_manager.get_users()
            know_names = []
            metrix = []
            for user in all_users_BLOB:
                encodings = pickle.loads(user[2])
                for encoding in encodings:
                    metrix.append(encoding)
                    know_names.append(user[1])
            data = {"encodings": metrix, "names": know_names}
            boxes, names = self.face_action.face_detection(data)
            image = QImage(800, 480, QImage.Format_ARGB32_Premultiplied)
            image.fill(Qt.transparent)
            if names:
                for ((top, right, bottom, left), name) in zip(boxes, names):
                    y = top - 15 if top - 15 > 15 else top + 15
                    painter = QPainter(image)
                    painter.setBrush(QColor(255, 0, 0, 0))
                    painter.drawRect(left, top, right - left, bottom - top)
                    painter.drawText(QPoint(left, y), name)
                    painter.end()
            return image

    def game_mode(self, command):
        if not 'кнб' in command:
            print("Давайте! скажите камень-ножницы-бумага!")
            while True:
                self.recognizer.listening()
                full_phrase = self.recognizer.get_full_phrase()
                if full_phrase:
                    key_phrase = self.key_phrase_analizer.analyzer(full_phrase)
                    if 'кнб' in key_phrase:
                        self.test_rps.play_game_trig = True
                        self.test_rps.start_game_RPS(self.recognizer)
                        break
                    print(full_phrase)
        else:
            key_phrase = command
            if 'кнб' in key_phrase:
                self.test_rps.play_game_trig = True
                self.test_rps.start_game_RPS(self.recognizer)

    def time_now(self):
        dt = None
        if self.os_name == "Windows":
            dt = datetime.datetime.now().strftime("%H %M")
        elif self.os_name == "Linux":
            dt = datetime.datetime.now().strftime("%H %M")
        print('Сейчас' + dt)

    def calculate(self, str1):
        words = str1.split()
        print(words)
        try:
            number1 = None
            number2 = None
            for number in words:
                try:
                    number = int(number)
                    if number1 is None:
                        number1 = int(number)
                    elif number2 is None:
                        number2 = int(number)
                        break
                except:
                    pass
            operator = None
            for item in words:
                if item in ['х', '/', '+', '-']:
                    operator = item
            if operator == '+':
                result = number1 + number2
            elif operator == '-':
                result = number1 - number2
            elif operator == 'х':
                result = number1 * number2
            elif operator == '/':
                result = number1 / number2
            else:
                result = None
            if result is None:
                print('Я не поняла, повторите')
            else:
                res = 'Результат ' + str(result)
                print(res)
        except:
            print('Я не поняла, повторите')

    def wikisearch(self, order):
        words = order.split()
        try:
            index = words.index("википедии")
        except ValueError:
            return []
        input_list = words[index + 1:]
        sentence = " ".join(map(str, input_list))
        print(sentence)
        print('Ищу...')
        results = wikipedia.summary([sentence], sentences=1)
        print("В википедии написано")
        print(results)

    def makeQR(self, qrtext, OL):
        qrcode = segno.make_qr(qrtext)
        qrcode.save("qr.png")
        current_dir = os.path.dirname(__file__)
        path_to_pngxxx = os.path.join(current_dir, "qr.png")
        image = QImage(path_to_pngxxx)
        pixmap = QPixmap.fromImage(image)
        pixmap = pixmap.scaled(400, 400)
        OL.setAlignment(Qt.AlignCenter)
        OL.setPixmap(pixmap)
        print("Сделано!")

    def joke(self):
        elements = []
        f = open('jokes.txt', 'r+', encoding='UTF-8')
        elements = f.readlines()
        lenjokes = len(elements)
        x = random.randint(0, lenjokes - 1)
        print(elements[x])