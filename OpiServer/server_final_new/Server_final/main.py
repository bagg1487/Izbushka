import sys
import os
import threading
import time
import cv2  # Явный импорт для работы с видео
import random
from os import path

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer, QSize, Qt, QEvent, pyqtSignal
from PyQt5.QtGui import QMovie, QImage, QPixmap

# Импорт интерфейса и сервера
from server_interface import Ui_MainWindow
from Server import Server

class MainWindow(QMainWindow, Ui_MainWindow):
    """Класс интерфейса приложения"""
    animationFinished = pyqtSignal()

    def __init__(self):
        # Инициализация приложения
        self.app = QApplication(sys.argv)
        super(MainWindow, self).__init__()
        self.setupUi(self)

        # Инициализация сервера
        # Мы передаем outside_layout, чтобы Server мог рисовать на экране (например, QR-коды)
        self.server = Server(self.outside_layout)

        # Настройка путей
        self.parent_dir = path.dirname(path.abspath(__file__))
        
        # Загрузка GIF анимаций (Лицо робота)
        self.gifs = []
        try:
            anim_path = os.path.join(self.parent_dir, "all_animation")
            prs_path = os.path.join(self.parent_dir, "PRS") # Папка анимаций для игры
            
            # Загружаем основные анимации
            if os.path.exists(anim_path):
                self.gifs = sorted(os.listdir(anim_path))
                self.gifs = [os.path.join("all_animation", g) for g in self.gifs]
            
            # Загружаем анимации для игры (если папка есть)
            if os.path.exists(prs_path):
                prs = sorted(os.listdir(prs_path))
                prs = [os.path.join("PRS", g) for g in prs]
                self.gifs.extend(prs)
        except Exception as e:
            print(f"Ошибка загрузки анимаций: {e}")

        # Установка начальной анимации (индекс 2 - обычно состояние покоя)
        self.current_index = 2 if len(self.gifs) > 2 else 0
        self.movie = QMovie(os.path.join(self.parent_dir, self.gifs[self.current_index]))
        self.movie.setScaledSize(QSize(800, 480))
        self.movie.frameChanged.connect(self.check_animation_finished)
        self.update_gif()

        # Настройка кнопки включения
        self.pushButton.clicked.connect(self.on_and_off_server)
        self.pushButton.setStyleSheet("background-color: rgba(255, 255, 255, 127);")
        self.pushButton.raise_()

        # Таймер для обновления видео с камеры (30 FPS)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.get_video_frame)
        self.timer.start(33) 

        # Таймер для обновления интерфейса
        self.timer_interface = QTimer(self)
        self.timer_interface.timeout.connect(self.interface_frame)
        self.timer_interface.start(100)

        # Настройка окна (Безрамочное, 800x480)
        self.setMaximumSize(800, 480)
        self.setMinimumSize(800, 480)
        self.centralwidget.setGeometry(0, 0, 800, 480)
        self.setWindowFlags(Qt.FramelessWindowHint)

        # Настройка слоев отображения
        self.Video_label.setGeometry(0, 0, 800, 480)
        self.Video_label.setStyleSheet("background-color: rgba(255, 255, 255, 127);")
        self.outside_layout.setGeometry(QtCore.QRect(0, 0, 800, 480))

        # Подключение виджетов плееров (Видео, Музыка, Радио)
        self.video_player = self.server.service.video_player
        self.video_player.setParent(self.centralwidget)
        self.video_player.setGeometry(QtCore.QRect(0, 0, 800, 480))
        self.video_player.videoWidget.setGeometry(QtCore.QRect(0, 0, 800, 480))

        self.server.service.music_player.setParent(self.centralwidget)
        self.server.service.radio_player.setParent(self.centralwidget)

        # Фильтры событий (для отслеживания активности мыши/нажатий)
        self.installEventFilter(self)
        
        # Таймеры "скуки" (робот начинает говорить, если его не трогать)
        self.timer_hold_3 = QTimer(self)
        self.timer_hold_3.setSingleShot(True)
        self.timer_hold_3.timeout.connect(self.server.hold_3)
        
        self.timer_hold_10 = QTimer(self)
        self.timer_hold_10.setSingleShot(True)
        self.timer_hold_10.timeout.connect(self.server.hold_10)

    def eventFilter(self, obj, event):
        """Сброс таймеров простоя при активности пользователя"""
        if event.type() in [QEvent.MouseMove, QEvent.HoverMove, QEvent.MouseButtonRelease]:
            # Уведомляем плееры об активности (чтобы они показали кнопки)
            self.server.service.music_player.event_filter()
            self.server.service.video_player.event_filter()
            self.server.service.radio_player.event_filter()
            
            # Проверяем статус игры безопасно (на случай если модуль отключен)
            game_active = False
            if hasattr(self.server.service, 'test_rps'):
                 game_active = self.server.service.test_rps.play_game_trig

            # Если ничего активного не происходит, сбрасываем таймеры скуки
            if not self.server.instruction_active_flag and \
               not self.server.service.music_player.player_active_trig and \
               not self.server.service.video_player.video_player_active_trig and \
               not self.server.neuro_talk_trig and \
               not game_active:
                
                self.timer_hold_3.start(180000) # 3 минуты
                self.timer_hold_10.start(600000) # 10 минут
                
        return super().eventFilter(obj, event)

    def on_and_off_server(self):
        """Логика кнопки ON/OFF (Запуск робота)"""
        text = self.pushButton.text()
        
        if text == 'On':
            # Запуск таймеров скуки
            self.timer_hold_3.start(180000)
            self.timer_hold_10.start(600000)
            
            # Изменение кнопки
            self.pushButton.setText('OFF_DEMO')
            self.pushButton.setGeometry(550, 0, 200, 50)
            self.label.setVisible(False)
            
            # Включение серверной части
            self.server.turn_on_server()
            self.server.tcp_flag = True
            self.server.stop_instruction_thread = False
            
            # Запуск потоков (Видео, Экран, Инструкции)
            self.video = threading.Thread(target=self.server.transmission_video, daemon=True)
            self.video.start()
            
            self.video_on_screen = threading.Thread(target=self.server.translate_video_on_screen, daemon=True)
            self.video_on_screen.start()
            
            self.instruction = threading.Thread(target=self.server.receive_instruction, daemon=True)
            self.instruction.start()
            
            # --- ВАЖНО: Запуск голосового ассистента ---
            self.voice_thread = threading.Thread(target=self.server.takeCommand, daemon=True)
            self.voice_thread.start()

        elif text == 'ON_DEMO':
            self.timer_hold_3.start(180000)
            self.timer_hold_10.start(600000)
            self.pushButton.setText('OFF_DEMO')
            self.pushButton.setGeometry(550, 0, 200, 50)
            
        else:
            # Выключение
            self.pushButton.setText('ON_DEMO')
            self.timer_hold_3.stop()
            self.timer_hold_10.stop()
            self.pushButton.setGeometry(550, 0, 200, 50)
            # При желании можно добавить self.server.turn_off_server()

    def get_video_frame(self):
        """Получение кадра с камеры и вывод на экран"""
        try:
            # Если сервер получил кадр и не занят его обработкой
            if not self.server.video_flag and self.server.video_translation_flag and self.server.frame_row is not None:
                # Конвертация BGR (OpenCV) -> RGB (Qt)
                rgb_image = cv2.cvtColor(self.server.frame_row, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                qt_img = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                
                # Масштабирование и установка в Label
                pixmap = QPixmap.fromImage(qt_img)
                pixmap = pixmap.scaled(800, 480, Qt.KeepAspectRatioByExpanding)
                self.Video_label.setPixmap(pixmap)
                
                # Разрешаем серверу обрабатывать следующий кадр
                self.server.video_flag = True
        except Exception as e:
            pass # Игнорируем ошибки отрисовки кадров

    def interface_frame(self):
        pass

    def keyPressEvent(self, event):
        """Обработка нажатий клавиатуры"""
        key = event.key()
        # 'Q' для ручного старта игры (если она подключена)
        if key == 81: # Qt.Key_Q
             if hasattr(self.server.service, 'test_rps'):
                self.server.service.test_rps.start_trig = True

    def update_gif(self):
        """Обновление текущей анимации лица"""
        if self.movie:
            self.movie.stop()
            
            if 0 <= self.current_index < len(self.gifs):
                gif_path = os.path.join(self.parent_dir, self.gifs[self.current_index])
                self.movie.setFileName(gif_path)
                self.outside_layout.setMovie(self.movie)
                self.movie.setSpeed(100)
                self.movie.start()

    def check_animation_finished(self, frame_number):
        """Проверка окончания анимации для переключения на следующую"""
        
        # Если играет музыка (кроме анимаций пения)
        if self.server.service.music_player.player_active_trig and (self.current_index != 3 and self.current_index != 4):
            self.animationFinished.emit()
            return

        # Логика игры (если модуль есть)
        if hasattr(self.server.service, 'test_rps'):
            rps = self.server.service.test_rps
            if rps.play_game_trig:
                if rps.anim_trig and self.current_index != 9:
                    self.animationFinished.emit()
                    return
                if not rps.anim_trig and self.current_index == 9:
                    self.animationFinished.emit()
                    return

        # Обычное окончание GIF
        if frame_number == self.movie.frameCount() - 1:
            self.animationFinished.emit()

    def next_gif(self):
        """Логика выбора следующей эмоции/анимации"""
        if not self.server.animation_active_trig:
            return

        # 1. Режим Музыки
        if self.server.service.music_player.player_active_trig:
            if self.server.service.music_player.end_track_trig:
                self.current_index = 2 # Покой
            else:
                self.current_index = random.randint(3, 4) if len(self.gifs) > 4 else 0 # Пение
            
            self.server.service.music_player.end_track_trig = False
            self.update_gif()
            return

        # 2. Режим Игры
        if hasattr(self.server.service, 'test_rps') and self.server.service.test_rps.play_game_trig:
            rps = self.server.service.test_rps
            if rps.anim_trig:
                self.current_index = 9 # Таймер
            else:
                # Выбор анимации по результату игры
                # Индексы GIF должны совпадать с файлами в папке
                try:
                    if rps.selected == 0:
                        if rps.won == 1: self.current_index = 14
                        elif rps.won == -1: self.current_index = 15
                        else: self.current_index = 13
                    elif rps.selected == 1:
                        if rps.won == 1: self.current_index = 11
                        elif rps.won == -1: self.current_index = 12
                        else: self.current_index = 10
                    elif rps.selected == 2:
                        if rps.won == 1: self.current_index = 17
                        elif rps.won == -1: self.current_index = 18
                        else: self.current_index = 16
                except IndexError:
                    self.current_index = 2
            self.update_gif()
            return

        # 3. Обычный режим (моргание)
        if random.randint(0, 100) > 95: 
            self.current_index = 0 # Моргание
        else:
            self.current_index = 2 # Покой
            
        self.update_gif()

if __name__ == '__main__':
    try:
        # Фикс для запуска на некоторых Linux системах без экрана (или через SSH)
        # os.environ["QT_QPA_PLATFORM"] = "xcb" 
        
        myshow = MainWindow()
        myshow.show()
        
        # Зацикливание анимации
        myshow.animationFinished.connect(myshow.next_gif)
        
        sys.exit(myshow.app.exec_())
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception as e:
        print(f"Critical Error in Main: {e}")
        sys.exit(1)
