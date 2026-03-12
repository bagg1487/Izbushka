import os
import sys

import psutil
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QSlider
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtCore import Qt, QUrl, QDir, QTimer, QEvent, QPropertyAnimation


class MusicPlayer(QtWidgets.QFrame):
    """Класс музыкального плеера"""
    def __init__(self, parent=None):
        super(MusicPlayer, self).__init__(parent)
        #self.player_frame = QtWidgets.QFrame()
        #self.setEnabled(True)
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
                QSlider::groove:vertical {
                    border: 1px solid #565a5e;
                    height: 71px;
                    background: #eee;
                    display: table-cell; 
                    vertical-align: middle;
                    border-radius: 4px;
                }
                QSlider::handle:vertical {
                    background: red;
                    border: 1px solid #565a5e;
                    width: 22px;
                    height: 8px;
                    border-radius: 8px;
                }
                }
                """
        # Установка стилей для виджета
        self.setStyleSheet(CSS)

        # Установка геометрии виджета
        self.setGeometry(QtCore.QRect(round((900 - 401) / 2), 100, 401, 111))

        # Создание кнопки "Назад" и установка ее геометрии и стилей
        self.back_button = QtWidgets.QLabel(self)
        self.back_button.setGeometry(QtCore.QRect(90, 15, 50, 50))
        self.back_button.setObjectName("back_button")
        self.back_button.setStyleSheet("background: rgba(8, 91, 169, 1);")

        # Создание кнопки "Воспроизведение/Пауза" и установка ее геометрии и стилей
        self.play_pause_button = QtWidgets.QLabel(self)
        self.play_pause_button.setGeometry(QtCore.QRect(160, 15, 50, 50))
        self.play_pause_button.setObjectName("play_pause_button")
        self.play_pause_button.setStyleSheet("background: rgba(8, 91, 169, 1);")

        # Создание кнопки "Далее" и установка ее геометрии и стилей
        self.next_button = QtWidgets.QLabel(self)
        self.next_button.setGeometry(QtCore.QRect(230, 15, 50, 50))
        self.next_button.setObjectName("next_button")
        self.next_button.setStyleSheet("background: rgba(8, 91, 169, 1);")

        # Создание слайдера позиции и установка его геометрии и атрибутов
        self.position_slider = QtWidgets.QSlider(self)
        self.position_slider.setGeometry(QtCore.QRect(70, 80, 221, 22))
        self.position_slider.setOrientation(QtCore.Qt.Horizontal)
        self.position_slider.setObjectName("position_slider")

        # Создание слайдера громкости и установка его геометрии и атрибутов
        self.volume_slider = QtWidgets.QSlider(self)
        self.volume_slider.setGeometry(QtCore.QRect(340, 15, 22, 71))
        self.volume_slider.setOrientation(QtCore.Qt.Vertical)
        self.volume_slider.setObjectName("volume_slider")

        # Создание таймера обновления и установка его интервала и соединения с методом обновления
        self.timer_update = QTimer(self)
        self.timer_update.timeout.connect(self.update_event)
        self.timer_update.start()

        # Установка изображений кнопок и привязка методов к событиям щелчка мыши
        self.back_button.setScaledContents(True)
        self.back_button.setPixmap(QPixmap(QDir.currentPath() + "/player_button/previous.png"))
        self.back_button.mouseReleaseEvent = self.previous_track

        self.next_button.setScaledContents(True)
        self.next_button.setPixmap(QPixmap(QDir.currentPath() + "/player_button/next.png"))
        self.next_button.mouseReleaseEvent = self.next_track

        self.play_pause_button.setScaledContents(True)
        self.play_pause_button.setPixmap(QPixmap(QDir.currentPath() + "/player_button/play.png"))
        self.play_pause_button.mouseReleaseEvent = self.play_pause

        # Привязка событий перемещения слайдера позиции и изменения громкости к соответствующим методам
        self.volume_slider.mouseReleaseEvent = self.set_volume
        self.player_active_trig = False
        self.player_stat = 1
        self.player = QMediaPlayer()

        # Привязка событий изменения позиции и длительности медиафайла, ошибок, смены статуса и смены медиафайла к соответствующим методам
        self.player.positionChanged.connect(self.positionChanged)
        self.player.durationChanged.connect(self.durationChanged)
        self.player.error.connect(self.handleError)
        self.player.mediaStatusChanged.connect(self.handle_media_status_changed)
        self.player.mediaChanged.connect(self.handle_media_changed)
        self.position_slider.sliderMoved.connect(self.setPosition)

        # Создаем таймер и подключаем его к методу reset_timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.reset_timer)

        # Запускаем таймер
        self.timer.start(5000)

        # Устанавливаем фильтр событий для отслеживания события перемещения мыши
        self.installEventFilter(self)


        # Переменная для хранения индекса текущего трека
        self.current_track_index = 0

        # Список для хранения всех аудиофайлов на флеш-карте
        self.audio_files = []

        #триггер проигрывания анимации
        self.animation_trig = True

        self.end_track_trig = True

        # Вызываем функцию для поиска флеш-карты
        # self.path = self.find_flash_drive()
        current_dir = os.path.dirname(__file__)
        self.path = current_dir+"/audio"
        # Если флеш-карта найдена, то вызываем функцию для поиска всех аудиофайлов на ней
        if self.path:
            self.find_all_audio()

        # Если есть аудиофайлы на флеш-карте, то вызываем функцию для воспроизведения текущего трека
        if self.audio_files:
            self.play_pause()



    def update_event(self):
        """Функция показывает или скрывает плеер в зависимости от состояния (запущен или нет)"""
        if self.player_active_trig:
            self.setVisible(True)
        else:
            self.setVisible(False)
    def play(self,index=0):
        """Функция запуска радио
        Аргумент - индекс трека, по умолчанию 0"""
        self.current_track_index=index
        #self.set_track(self.audio_files[self.current_track_index])
        self.play_pause()
    def play_pause(self, event=None):
        """Функция изменения интерфейса плеера"""
        # Если проигрыватель активен
        if self.player_active_trig:
            # Если проигрывается трек
            if self.player_stat == 1:
                # Установка текущего трека и воспроизведение
                self.set_track(self.audio_files[self.current_track_index])
                self.player.play()
                self.player_stat = 0
                self.play_pause_button.setPixmap(QPixmap(QDir.currentPath() + "/player_button/pause.png"))
            else:
                # Пауза проигрывания
                self.player.pause()
                self.player_stat = 1
                self.play_pause_button.setPixmap(QPixmap(QDir.currentPath() + "/player_button/play.png"))

        # Установка флага окончания трека
        self.end_track_trig = True
    def set_volume(self, event):
        """Функция установки громкости плеера"""
        if self.player_active_trig:
            self.player.setVolume(self.volume_slider.value())


    def stop(self):
        """Функция выключения музыки"""
        self.player.stop()
        self.player_stat = 1


    def set_track(self, track_url):
        """Устанавливает олпределенный трек
        Аргумент - путь к файлу"""
        if self.player_active_trig:
            self.player.setMedia(QMediaContent(QUrl.fromLocalFile(track_url)))
            # self.track_label.setText(track_url.split('/')[-1])

    # Функция для поиска всех аудиофайлов на флеш-карте
    def find_all_audio(self):
        """Поиск всех аудиофайлов в директории приложения"""
        # Обходим все файлы и папки в указанном пути
        for root, dirs, files in os.walk(self.path):
            for file in files:
                # Проверяем, является ли файл аудиофайлом
                if file.endswith('.mp3') or file.endswith('.wav'):
                    # Получаем полный путь к аудиофайлу
                    audio_file_path = os.path.join(root, file)
                    # Добавляем аудиофайл в список
                    self.audio_files.append(audio_file_path)

    def find_flash_drive(self):
        """Функция для автоматического поиска флеш-карты"""
        # Получаем информацию о всех дисках
        disks = psutil.disk_partitions()
        # Обходим все диски
        for disk in disks:
            # Если диск является съемным и доступен, то это флеш-карта
            if 'removable' in disk.opts and 'rw' in disk.opts:
                # Возвращаем путь к флеш-карте
                return disk.mountpoint

    def next_track(self, event=None):
        """Функция для переключения на следующий трек"""
        if self.player_active_trig:
            # Увеличиваем индекс текущего трека
            self.current_track_index += 1
            # Проверяем, не превышает ли индекс длину списка аудиофайлов
            if self.current_track_index >= len(self.audio_files):
                self.current_track_index = 0
            # Воспроизводим текущий трек
            self.set_track(self.audio_files[self.current_track_index])
            self.player.play()

    def previous_track(self, event=None):
        """Функция для переключения на предыдущий трек"""
        if self.player_active_trig:
            # Уменьшаем индекс текущего трека
            self.current_track_index -= 1
            # Проверяем, не меньше ли индекс 0
            if self.current_track_index < 0:
                self.current_track_index = len(self.audio_files) - 1
            # Воспроизводим текущий трек
            self.set_track(self.audio_files[self.current_track_index])
            self.player.play()

    def positionChanged(self, position):
        self.position_slider.setValue(position)

    def durationChanged(self, duration):
        self.position_slider.setRange(0, duration)

    def setPosition(self, position):
        self.player.setPosition(position)

    def handleError(self):
        """Обработчик ошибок для плеера"""
        self.playButton.setEnabled(False)
        self.statusBar.showMessage("Error: " + self.mediaPlayer.errorString())

    def handle_media_status_changed(self, status):
        """Обработчик статуса плеера, если трек закончился, включается следующий"""
        if status == QMediaPlayer.EndOfMedia:
            self.end_track_trig = True
            self.next_track()

    def handle_media_changed(self):
        self.end_track_trig = True

    def event_filter(self):
        """Отмена скрытия плеера при движении мыши"""
        # Перезапускаем таймер при перемещении мыши
        self.timer.start(5000)
        # print('Mouse moved, timer reset')
        self.open_animation()

    def reset_timer(self):
        """Скрытие плеера после бездействия"""
        # Сбрасываем таймер и выводим сообщение
        self.timer.stop()
        # print('Timer reset due to inactivity')
        self.close_animation()

    # Обработчик события нажатия клавиши
    def keyPressEvent(self, event):
        """Обработка нажатия на экран, для показа плеера """
        # Перезапускаем таймер при нажатии клавиши
        self.timer.start(5000)
        # print('Key pressed, timer reset')
        self.open_animation()

    def open_animation(self):
        """Анимация появления плеера"""
        if self.animation_trig:
            # Создание анимации изменения геометрии
            self.animation = QPropertyAnimation(self, b'geometry')
            self.animation.setDuration(500)
            self.animation.setStartValue(QtCore.QRect(0, -111, 401, 111))
            self.animation.setEndValue(QtCore.QRect(0, 0, 401, 111))
            self.animation.start()
            # Отключение активации анимации
            self.animation_trig = False
    def close_animation(self):
        """Анимация скрытия плеера"""
        if not self.animation_trig:
            # Создание анимации изменения геометрии
            self.animation = QPropertyAnimation(self, b'geometry')
            self.animation.setDuration(500)
            self.animation.setStartValue(QtCore.QRect(0, 0, 401, 111))
            self.animation.setEndValue(QtCore.QRect(0, -111, 401, 111))
            self.animation.start()
            # Включение активации анимации
            self.animation_trig = True
