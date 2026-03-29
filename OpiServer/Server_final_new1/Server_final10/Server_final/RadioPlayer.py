import os
import platform
import pygame

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QPropertyAnimation, QTimer, QDir
from PyQt5.QtGui import QPixmap

os_name = platform.system()
if os_name == "Windows":
    os.add_dll_directory(r'C:\Program Files\VideoLAN\VLC')
import vlc


class RadioPlayer(QtWidgets.QFrame):
    """Класс музыкального плеера"""

    def __init__(self, parent=None):
        """Функция инициализации"""
        super(RadioPlayer, self).__init__(parent)
        
        CSS = """
        QLabel {
            font-family: Ubuntu-Regular;
            font-size: 12px;
            qproperty-alignment: AlignCenter;
            color: yellow;
            border-radius: 4px;
            min-height: 40px;
            max-height: 40px;
            min-width: 48px;
            max-width: 100px;
        }
        """
        self.setStyleSheet(CSS)
        self.setGeometry(QtCore.QRect(round((900 - 401) / 2), 100, 401, 111))

        self.play_pause_button = QtWidgets.QLabel(self)
        self.play_pause_button.setGeometry(QtCore.QRect(160, 15, 50, 50))
        self.play_pause_button.setObjectName("play_pause_button")
        self.play_pause_button.setStyleSheet("background: rgba(8, 91, 169, 1);")
        self.play_pause_button.setScaledContents(True)
        try:
            self.play_pause_button.setPixmap(QPixmap(QDir.currentPath() + "/player_button/play.png"))
        except: pass
        self.play_pause_button.mouseReleaseEvent = self.play_pause

        self.timer_update = QTimer(self)
        self.timer_update.timeout.connect(self.update_event)
        self.timer_update.start()

        self.timer = QTimer()
        self.timer.timeout.connect(self.reset_timer)
        self.timer.start(5000)

        self.installEventFilter(self)

        self.radio_active_trig = False
        self.current = 0
        self.player_stat = 1
        self.animation_trig = True

        # НАСТРОЙКИ VLC (Ваши рабочие настройки)
        # Используем ALSA и конкретную карту
        self.instance = vlc.Instance("--aout=alsa", "--alsa-audio-device=plughw:3,0", "--no-video")
        self.player = self.instance.media_player_new()
        
        # Чтение списка станций
        self.elements = []
        try:
            parent_dir = os.path.dirname(os.path.abspath(__file__))
            radio_path = os.path.join(parent_dir, 'radio.txt')
            with open(radio_path, 'r', encoding='UTF-8') as f:
                content = f.read().split()
                if content:
                    self.elements = content
                else:
                    self.elements = ["http://stream.nonstopplay.co.uk/nsp-128k-mp3"]
        except Exception as e:
            print(f"Ошибка загрузки radio.txt: {e}")
            self.elements = ["http://stream.nonstopplay.co.uk/nsp-128k-mp3"]

    def update_event(self):
        if self.radio_active_trig:
            self.setVisible(True)
        else:
            self.setVisible(False)

    def play_pause(self, event=None):
        if not self.radio_active_trig:
            self.radio_active_trig = True

        if self.player_stat == 1:
            self.play()
            self.player_stat = 0
            try:
                self.play_pause_button.setPixmap(QPixmap(QDir.currentPath() + "/player_button/pause.png"))
            except: pass
        else:
            self.pause()
            self.player_stat = 1
            try:
                self.play_pause_button.setPixmap(QPixmap(QDir.currentPath() + "/player_button/play.png"))
            except: pass

    def play(self):
        """Функция запуска радио"""
        if not self.elements:
            print("Список радиостанций пуст!")
            return

        # === ВАЖНОЕ ИСПРАВЛЕНИЕ ===
        # Останавливаем Pygame (голос), чтобы освободить звуковую карту для VLC
        try:
            if pygame.mixer.get_init():
                pygame.mixer.quit()
                print("🔊 Pygame mixer остановлен. Карта освобождена для радио.")
        except Exception as e:
            print(f"⚠️ Ошибка освобождения аудиокарты: {e}")

        # Запуск VLC
        try:
            url = self.elements[self.current]
            self.media = self.instance.media_new(url)
            self.player.set_media(self.media)
            self.player.audio_set_volume(60)
            self.player.play()
            print(f"📻 Радио запущено: {url}")
        except Exception as e:
            print(f"❌ Ошибка VLC: {e}")

    def pause(self):
        self.player.stop()

    def next_rad(self, event=None):
        self.player.stop()
        if self.elements:
            self.current += 1
            if self.current >= len(self.elements):
                self.current = 0
            self.play()

    def previous_rad(self, event=None):
        self.player.stop()
        if self.elements:
            self.current -= 1
            if self.current < 0:
                self.current = len(self.elements) - 1
            self.play()

    def reset_timer(self):
        self.timer.stop()
        self.close_animation()

    def keyPressEvent(self, event):
        self.timer.start(5000)
        self.open_animation()

    def event_filter(self):
        self.timer.start(5000)
        self.open_animation()

    def open_animation(self):
        if self.animation_trig:
            self.animation = QPropertyAnimation(self, b'geometry')
            self.animation.setDuration(500)
            self.animation.setStartValue(QtCore.QRect(0, -111, 401, 111))
            self.animation.setEndValue(QtCore.QRect(0, 0, 401, 111))
            self.animation.start()
            self.animation_trig = False

    def close_animation(self):
        if not self.animation_trig:
            self.animation = QPropertyAnimation(self, b'geometry')
            self.animation.setDuration(500)
            self.animation.setStartValue(QtCore.QRect(0, 0, 401, 111))
            self.animation.setEndValue(QtCore.QRect(0, -111, 401, 111))
            self.animation.start()
            self.animation_trig = True
