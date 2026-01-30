import os.path
from os import path
from PyQt5.QtGui import QMovie
from Server import *
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QMainWindow
from server_interface import Ui_MainWindow
from Service import *
from VideoPlayer import *
from MusicPlayer import *


class MainWindow(QMainWindow, Ui_MainWindow):
    """Класс интерфейса приложения"""
    animationFinished = pyqtSignal()

    def __init__(self):
        self.app = QApplication(sys.argv)
        super(MainWindow, self).__init__()
        self.setupUi(self)
        self.server = Server(self.outside_layout)
        # Получение списка файлов в папке
        self.parent_dir = path.dirname(path.abspath(__file__))
        self.gifs = sorted(os.listdir(self.parent_dir+"/all_animation"))
        prs = sorted(os.listdir(self.parent_dir+"/PRS"))
        self.gifs.extend(prs)
        self.current_index = 2
        self.movie = QMovie(self.parent_dir+"/all_animation/" + self.gifs[self.current_index])
        self.movie.setScaledSize(QSize(800, 480))
        self.movie.frameChanged.connect(self.check_animation_finished)
        self.update_gif()

        self.pushButton.clicked.connect(self.on_and_off_server)
        self.pushButton.setStyleSheet("background-color: rgba(255, 255, 255, 127);")
        self.pushButton.raise_()

        self.timer=QTimer(self)
        self.timer.timeout.connect(self.get_video_frame)
        self.timer.start(10)

        self.timer_interface=QTimer(self)
        self.timer_interface.timeout.connect(self.interface_frame)
        self.timer_interface.start(10)

        # Установка политики размеров
        self.setMaximumSize(800, 480)
        self.setMinimumSize(800, 480)
        self.centralwidget.setGeometry(0, 0, 800, 480)
        self.setWindowFlags(Qt.FramelessWindowHint)

        # Устанавливаем размеры Video_label
        self.Video_label.setGeometry(0, 0, 800, 480)
        self.Video_label.setStyleSheet("background-color: rgba(255, 255, 255, 127);")

        # Устанавливаем размеры outside_layout
        self.outside_layout.setGeometry(QtCore.QRect(0, 0, 800, 480))

        # подключаем виджет видеоплеера
        self.video_player = self.server.service.video_player.setParent(self.centralwidget)
        self.server.service.video_player.setGeometry(QtCore.QRect(0, 0, 800, 480))
        self.server.service.video_player.videoWidget.setGeometry(QtCore.QRect(0, 0, 800, 480))

        # подключаем виджет музыкального плеера
        self.server.service.music_player.setParent(self.centralwidget)
        #подключаем виджет радио плеера
        self.server.service.radio_player.setParent(self.centralwidget)

        # Устанавливаем фильтр событий на главное окно
        self.installEventFilter(self)
        self.timer_hold_3 = QTimer(self)
        self.timer_hold_3.setSingleShot(True)
        self.timer_hold_3.timeout.connect(self.server.hold_3)
        
        self.timer_hold_10 = QTimer(self)
        self.timer_hold_10.setSingleShot(True)
        self.timer_hold_10.timeout.connect(self.server.hold_10)

    def eventFilter(self, obj, event):
        """Фильтр событий отслеживающий состояние триггеров
        активности различных модулей, а так же взаимодействия с экраном
        """
        if event.type() == QEvent.MouseMove or event.type() == QEvent.HoverMove or event.type() == QEvent.MouseButtonRelease:
            self.server.service.music_player.event_filter()
            self.server.service.video_player.event_filter()
            self.server.service.radio_player.event_filter()
            if not self.server.instruction_active_flag and not self.server.service.music_player.player_active_trig and \
                    not self.server.service.video_player.video_player_active_trig and not self.server.neuro_talk_trig and \
                    not self.server.service.test_rps.play_game_trig:
                # Перезапускаем таймер при перемещении мыши
                self.timer_hold_3.start(180000)
                self.timer_hold_10.start(600000)
        # Возвращаем результат вызова метода eventFilter родительского объекта
        return super().eventFilter(obj, event)

    def on_and_off_server(self):
        """Включение и отключение сервера"""
        # Если текст кнопки равен 'On':
        if self.pushButton.text() == 'On':
            self.timer_hold_3.start(180000)
            self.timer_hold_10.start(600000)
            # Устанавливаем текст кнопки в 'X'
            self.pushButton.setText('OFF_DEMO')
            # Устанавливаем новое положение кнопки
            self.pushButton.setGeometry(550, 0, 200, 50)
            # Скрываем метку
            self.label.setVisible(False)
            # Включаем сервер
            self.server.turn_on_server()
            # Устанавливаем флаг TCP в True
            self.server.tcp_flag = True
            # Сбрасываем флаг остановки потока команд
            self.server.stop_instruction_thread = False
            # Создаем и запускаем поток для передачи видео
            self.video = threading.Thread(target=self.server.transmission_video, daemon=True)
            self.video.start()
            # Создаем и запускаем поток для отображения видео на экране
            self.video_on_screen = threading.Thread(target=self.server.translate_video_on_screen, daemon=True)
            self.video_on_screen.start()
            # Создаем и запускаем поток для получения инструкций
            self.instruction = threading.Thread(target=self.server.receive_instruction, daemon=True)
            self.instruction.start()
        elif self.pushButton.text() == 'ON_DEMO':
            self.timer_hold_3.start(180000)
            self.timer_hold_10.start(600000)
            self.pushButton.setText('OFF_DEMO')
            # Устанавливаем новое положение кнопки
            self.pushButton.setGeometry(550, 0, 200, 50)
        else:
            # Устанавливаем текст кнопки в 'On'
            self.pushButton.setText('ON_DEMO')
            print("close")
            self.timer_hold_3.stop()
            self.timer_hold_10.stop()
            # Устанавливаем новое положение кнопки
            self.pushButton.setGeometry(550, 0, 200, 50)

    def get_video_frame(self):
        """Обновление кадра с камеры в QLabel"""
        try:
            if not self.server.video_flag and self.server.video_translation_flag:
                rgb_image = cv2.cvtColor(self.server.frame_row, cv2.COLOR_BGR2RGB)
                QImg = QImage(rgb_image.data, rgb_image.shape[1], rgb_image.shape[0], QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(QImg)
                pixmap = pixmap.scaled(800, 480)
                self.Video_label.setPixmap(pixmap)
                self.server.video_flag = True
        except Exception as e:
            print(e)

    def interface_frame(self):
        try:
            if not self.server.video_flag and self.server.service.face_action.face_detection_trig:
                t1 = time.perf_counter()
                t2 = time.perf_counter()
                print(f" {t2 - t1:0.4f} sec")
        except Exception as e:
            print(e)

    def keyPressEvent(self, event):
        key = event.key()
        if key == 81:
            self.server.service.test_rps.start_trig = True

    def update_gif(self):
        """Обновление анимации по текущему индексу"""
        if self.movie:
            self.movie.stop()
            gif_path = self.gifs[self.current_index]
            if 0 <= self.current_index < 9:
                self.movie.setFileName(self.parent_dir + "/all_animation/" + gif_path)
            if 9 <= self.current_index < 19:
                self.movie.setFileName(self.parent_dir + "/PRS/" + gif_path)
            self.outside_layout.setMovie(self.movie)
            self.movie.setSpeed(100)
            self.movie.start()

    def check_animation_finished(self, frame_number):
        """Прерывание анимации при выполнении одного из условий"""
        # Убрал условия с речью
        if self.server.service.music_player.player_active_trig and (self.current_index != 3 or self.current_index != 4):
            self.animationFinished.emit()

        if self.server.service.test_rps.anim_trig and self.current_index != 9:
            self.animationFinished.emit()

        if not self.server.service.test_rps.anim_trig and self.current_index == 9:
            self.animationFinished.emit()

        if frame_number == self.movie.frameCount() - 1:
            self.animationFinished.emit()

    def next_gif(self):
        """Выбор следующей анимации в зависимости от условия"""
        if self.server.animation_active_trig:
            if self.server.service.music_player.player_active_trig:
                if self.server.service.music_player.end_track_trig:
                    if self.server.service.music_player.player_stat == 0:
                        self.current_index = random.randint(3, 4)
                    else:
                        self.current_index = 2
                    self.server.service.music_player.end_track_trig = False
                    self.update_gif()

            elif self.server.service.test_rps.play_game_trig:
                if self.server.service.test_rps.anim_trig:
                    self.current_index = 9
                else:
                    if self.server.service.test_rps.selected == 0:
                        if self.server.service.test_rps.won == 1:
                            self.current_index = 14
                        elif self.server.service.test_rps.won == -1:
                            self.current_index = 15
                        else:
                            self.current_index = 13
                    elif self.server.service.test_rps.selected == 1:
                        if self.server.service.test_rps.won == 1:
                            self.current_index = 11
                        elif self.server.service.test_rps.won == -1:
                            self.current_index = 12
                        else:
                            self.current_index = 10
                    elif self.server.service.test_rps.selected == 2:
                        if self.server.service.test_rps.won == 1:
                            self.current_index = 17
                        elif self.server.service.test_rps.won == -1:
                            self.current_index = 18
                        else:
                            self.current_index = 16
                self.update_gif()

            else:
                if random.randint(0, 100) > 101:
                    self.current_index = 0
                else:
                    self.current_index = 2
                self.update_gif()


if __name__ == '__main__':
    try:
        myshow = MainWindow()
        myshow.show()
        myshow.animationFinished.connect(myshow.next_gif)
        sys.exit(myshow.app.exec_())
    except KeyboardInterrupt:
        myshow.close()