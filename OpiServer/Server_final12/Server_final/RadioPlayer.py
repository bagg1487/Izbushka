import os
import platform

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QPropertyAnimation, QTimer, QDir
from PyQt5.QtGui import QPixmap

os_name = platform.system()
if os_name == "Windows":
    os.add_dll_directory(r'C:\Program Files\VideoLAN\VLC')
import vlc


class RadioPlayer(QtWidgets.QFrame):
    """Класс музыкального плеера"""

    # Инициализация плеера
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
                        QSlider::groove:horizontal {
                            border: 1px solid #565a5e;
                            height: 10px;
                            background: #eee;
                            display: table-cell; 
                            vertical-align: middle;
                            border-radius: 4px;
                        }
                        QSlider::handle:horizontal {
                            background: red;
                            border: 1px solid #565a5e;
                            width: 24px;
                            height: 8px;
                            border-radius: 8px;
                        }
                        }
                        """
        # Установка стилей
        self.setStyleSheet(CSS)
        # Установка геометрии окна
        self.setGeometry(QtCore.QRect(round((900 - 401) / 2), 100, 401, 111))

        # Создание кнопки "Воспроизвести/Пауза"
        self.play_pause_button = QtWidgets.QLabel(self)
        self.play_pause_button.setGeometry(QtCore.QRect(160, 15, 50, 50))
        self.play_pause_button.setObjectName("play_pause_button")
        self.play_pause_button.setStyleSheet("background: rgba(8, 91, 169, 1);")
        self.play_pause_button.setScaledContents(True)
        self.play_pause_button.setPixmap(QPixmap(QDir.currentPath() + "/player_button/play.png"))
        self.play_pause_button.mouseReleaseEvent = self.play_pause

        # Таймер для обновления событий
        self.timer_update = QTimer(self)
        self.timer_update.timeout.connect(self.update_event)
        self.timer_update.start()

        # Создаем таймер и подключаем его к методу reset_timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.reset_timer)

        # Запускаем таймер
        self.timer.start(5000)

        # Устанавливаем фильтр событий для отслеживания события перемещения мыши
        self.installEventFilter(self)

        # триггер состояния плеера (ключен или нет)
        self.radio_active_trig = False
        # плеер библиотеки vlc
        self.player = vlc.MediaPlayer()
        # здесь будет храниться список всех радиостанций
        self.elements = []
        # Открытие текстового документа
        parent_dir = os.path.dirname(os.path.abspath(__file__))
        f = open(parent_dir + '/radio.txt', 'r+', encoding='UTF-8')
        # Читение всех строк из файла
        self.elements = f.readline().split()
        # Текущий в начале всегда 0
        self.current = 0
        self.player_stat = 1

        # триггер проигрывания анимации
        self.animation_trig = True

    def update_event(self):
        """Функция показывает или скрывает плеер в зависимости от состояния (запущен или нет)"""
        if self.radio_active_trig:
            self.setVisible(True)
        else:
            self.setVisible(False)

    def play_pause(self, event=None):
        """Функция изменения интерфейса плеера"""
        # Если активировано радио
        if self.radio_active_trig:
            # Если плеер на паузе, воспроизвести радио
            if self.player_stat == 1:
                self.play()
                self.player_stat = 0
                self.play_pause_button.setPixmap(QPixmap(QDir.currentPath() + "/player_button/pause.png"))
            # Если плеер воспроизводит, поставить на паузу
            else:
                self.pause()
                self.player_stat = 1
                self.play_pause_button.setPixmap(QPixmap(QDir.currentPath() + "/player_button/play.png"))
        # Установка флага окончания трека
        self.end_track_trig = True

    def play(self):  # включение радио
        """Функция запуска радио"""
        # выбираем радиостанцию
        self.media = vlc.Media(self.elements[self.current])
        # передаем плееру радиостанцию
        self.player.set_media(self.media)
        self.player.audio_set_volume(30)
        # Воспроизводим радиостанцию
        self.player.play()

    def pause(self):
        """Функция постановки радио на паузу"""
        if self.radio_active_trig:
            self.player_stat = 1
            self.player.stop()
            self.radio_active_trig = False

    def next_rad(self, event=None):
        pass
        """Функция переключения радио на следующую волну
        # останавливаем радио, чтобы запустить его с новой волной
        self.player.stop()
        if self.radio_active_trig:
            # Увеличиваем индекс текущего трека
            self.current += 1
            # Проверяем, не превышает ли индекс длину списка аудиофайлов
            if self.current >= len(self.elements):
                self.current = 0
            self.player.audio_set_volume(10)
            self.player.play()"""

    def previous_rad(self, event=None):  # прошлая волна
        pass
        """Функция переключения радио на прошлую волну
        # останавливаем радио, чтобы запустить его с новой волной
        self.player.stop()
        if self.radio_active_trig:
            # Уменьшаем индекс текущего трека
            self.current -= 1
            # Проверяем, не превышает ли индекс длину списка аудиофайлов
            if self.current < 0:
                self.current = len(self.elements)
            self.player.audio_set_volume(10)
            self.player.play()"""

    def reset_timer(self):
        """Функция сброса таймера анимаций"""
        # Сбрасываем таймер и выводим сообщение
        self.timer.stop()
        # print('Timer reset due to inactivity')
        self.close_animation()

    def keyPressEvent(self, event):
        """Обработчик события нажатия клавиши"""
        # Перезапускаем таймер при нажатии клавиши
        self.timer.start(5000)
        # print('Key pressed, timer reset')
        self.open_animation()

    def event_filter(self):
        """Обработчик события перемещения мыши"""
        # Перезапускаем таймер при перемещении мыши
        self.timer.start(5000)
        # print('Mouse moved, timer reset')
        self.open_animation()

    def open_animation(self):
        """Открытие анимации"""
        if self.animation_trig:
            self.animation = QPropertyAnimation(self, b'geometry')
            self.animation.setDuration(500)
            self.animation.setStartValue(QtCore.QRect(0, -111, 401, 111))
            self.animation.setEndValue(QtCore.QRect(0, 0, 401, 111))
            self.animation.start()
            self.animation_trig = False

    def close_animation(self):
        """Закрытие анимации"""
        if not self.animation_trig:
            self.animation = QPropertyAnimation(self, b'geometry')
            self.animation.setDuration(500)
            self.animation.setStartValue(QtCore.QRect(0, 0, 401, 111))
            self.animation.setEndValue(QtCore.QRect(0, -111, 401, 111))
            self.animation.start()
            self.animation_trig = True
