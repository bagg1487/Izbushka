import os.path

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QIcon, QFont
from PyQt5.QtCore import QDir, Qt, QUrl, QSize, QTimer, QPropertyAnimation
from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtMultimediaWidgets import QVideoWidget
from PyQt5.QtWidgets import (QApplication, QFileDialog, QHBoxLayout, QLabel,
                             QPushButton, QSizePolicy, QSlider, QStyle, QVBoxLayout, QWidget, QStatusBar,
                             QDesktopWidget, QGraphicsOpacityEffect)


class VideoPlayer(QtWidgets.QFrame):
    """Класс видеоплейера """
    def __init__(self, parent=None):
        # Наследуемся от QtWidgets.QFrame
        super(VideoPlayer, self).__init__(parent)
        # Задаем параметры фрема
        self.setEnabled(True)
        CSS = """
        QLabel {
            font-family: Ubuntu-Regular;
            font-size: 12px;
            qproperty-alignment: AlignCenter;
            color: yellow;
            background: #565a5e;
            border: 1px solid #565a5e;
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
        QPushButton{
            background: black;
            border: 1px solid #FF0000;
            height: 24px;
        }
        SP_MediaPlay{
            background: red;
        }
        """
        self.setStyleSheet(CSS)
        # Создание QMediaPlayer
        self.mediaPlayer = QMediaPlayer(None, QMediaPlayer.VideoSurface)
        # Задаем размеры кнопки
        btnSize = QSize(16, 16)
        # Создаем QVideoWidget для проигрывания видео
        self.videoWidget = QVideoWidget(self)
        #Создание и настройка фремя меню
        self.menu_frame = QtWidgets.QFrame(self)
        self.menu_frame.setGeometry(0,480-55,800,40)
        self.menu_frame.setStyleSheet("background: rgba(0, 0, 0,255);")
        # Создание и настройка кнопки закрытия видео
        self.close_button = QPushButton('')
        self.close_button.setParent(self)
        self.close_button.setGeometry(760,0,40,40)
        self.close_button.clicked.connect(self.close_event)
        self.close_button.setIcon(QIcon('interface/close_icon.png'))
        self.close_button.setStyleSheet("background: rgba(0, 0, 0,255);")

        # Создание и настрока кнопки по открытию видео
        openButton = QPushButton('')
        openButton.setParent(self.menu_frame)
        openButton.setGeometry(5,0,60,24)
        openButton.setStyleSheet("background: rgba(0, 0, 0,255);color:white;")
        openButton.setToolTip("Open Video File")
        openButton.setStatusTip("Open Video File")
        openButton.setIconSize(btnSize)
        openButton.setIcon(QIcon.fromTheme("document-open", QIcon("interface/folder.png")))
        openButton.clicked.connect(self.abrir)

        # Создание и настрока кнопки по проигрыванию видео(пауза/плей)
        self.playButton = QPushButton(self.menu_frame)
        self.playButton.setGeometry(67,0,40,40)
        self.playButton.setEnabled(False)
        self.playButton.setFixedHeight(24)
        self.playButton.setIconSize(btnSize)
        self.playButton.setIcon(QIcon('interface/playPause.png'))
        self.playButton.clicked.connect(self.play)

        # Создание и настрока слайдбара прогресса видео
        self.positionSlider = QSlider(Qt.Horizontal)
        self.positionSlider.setGeometry(110,-10,685,40)
        self.positionSlider.setParent(self.menu_frame)
        self.positionSlider.setRange(0, 0)
        self.positionSlider.sliderMoved.connect(self.setPosition)

        self.statusBar = QStatusBar()
        self.statusBar.setFont(QFont("Noto Sans", 7))
        self.statusBar.setFixedHeight(14)

        # Настройка ивентов видеоплеера
        self.mediaPlayer.setVideoOutput(self.videoWidget)
        #self.mediaPlayer.stateChanged.connect(self.mediaStateChanged)
        self.mediaPlayer.positionChanged.connect(self.positionChanged)
        self.mediaPlayer.durationChanged.connect(self.durationChanged)
        self.mediaPlayer.error.connect(self.handleError)
        self.statusBar.showMessage("Ready")

        self.timer_update = QTimer(self)
        self.timer_update.timeout.connect(self.update_event)
        self.timer_update.start()

        self.video_player_active_trig = False

        # Создаем таймер и подключаем его к методу reset_timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.reset_timer)

        # Запускаем таймер
        self.timer.start(5000)

        # Устанавливаем фильтр событий для отслеживания события перемещения мыши
        self.installEventFilter(self)
        self.animation_trig = False

    def abrir(self):
        """Открытие диалогового окна для поиска файла для воспроизведения"""
        # Открытие диалогового окна для выбора видеофайла
        fileName, _ = QFileDialog.getOpenFileName(self, "Выберите медиафайл",
                                                  ".", "Видеофайлы (*.mp4 *.flv *.ts *.mts *.avi)")

        # Если был выбран файл
        if fileName != '':
            # Установка медиафайла для проигрывания
            self.mediaPlayer.setMedia(
                QMediaContent(QUrl.fromLocalFile(fileName)))
            # Включение кнопки воспроизведения
            self.playButton.setEnabled(True)
            # Отображение имени выбранного файла в строке состояния
            self.statusBar.showMessage(fileName)
            # Воспроизведение файла
            self.play()

    def play_first(self,file_path='abba.MP4'):
        """Проигрывается один раз при запуске видеоплейера"""
        # Получение пути к директории текущего файла
        parent_dir = os.path.dirname(os.path.abspath(__file__))
        # Установка медиафайла для проигрывания, используя путь к директории текущего файла и указанный путь к файлу
        self.mediaPlayer.setMedia(
            QMediaContent(QUrl.fromLocalFile(os.path.join(parent_dir, file_path))))
        # Включение кнопки воспроизведения
        self.playButton.setEnabled(True)
        # Отображение имени выбранного файла в строке состояния
        self.statusBar.showMessage(file_path)
        # Воспроизведение файла
        self.play()

    def play(self):
        """Переключение состояния между проигрыванием и паузой"""
        if self.video_player_active_trig:
            if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
                self.mediaPlayer.pause()
            else:
                self.mediaPlayer.play()
        else:
            self.mediaPlayer.pause()
            self.mediaPlayer.stop()


    def mediaStateChanged(self, state):
        """Переключение иконки между проигрыванием и паузой"""
        if self.mediaPlayer.state() == QMediaPlayer.PlayingState:
            self.playButton.setIcon(
                self.style().standardIcon(QStyle.SP_MediaPause))
        else:
            self.playButton.setIcon(
                self.style().standardIcon(QStyle.SP_MediaPlay))

    def positionChanged(self, position):
        self.positionSlider.setValue(position)

    def durationChanged(self, duration):
        self.positionSlider.setRange(0, duration)

    def setPosition(self, position):
        self.mediaPlayer.setPosition(position)

    def handleError(self):
        self.playButton.setEnabled(False)
        self.statusBar.showMessage("Error: " + self.mediaPlayer.errorString())

    def update_event(self):
        if self.video_player_active_trig:
            self.setVisible(True)
        else:
            self.setVisible(False)

    def close_event(self):
        self.video_player_active_trig=False
        self.play()

    def event_filter(self):
        # Перезапускаем таймер при перемещении мыши
        self.timer.start(5000)
        #print('Mouse moved, timer reset')
        self.open_animation()

    def reset_timer(self):
            # Сбрасываем таймер и выводим сообщение
            self.timer.stop()
            #print('Timer reset due to inactivity')
            self.close_animation()

    def open_animation(self):
        """Анимация появления при действии"""
        # Если триггер анимации активен
        if self.animation_trig:
            # Создание анимации для меню
            self.animation_menu = QPropertyAnimation(self.menu_frame, b'geometry')
            self.animation_menu.setDuration(500)
            self.animation_menu.setStartValue(QtCore.QRect(0, 480, 800, 40))
            self.animation_menu.setEndValue(QtCore.QRect(0, 480 - 55, 800, 40))

            # Создание анимации для кнопки закрытия
            self.animation_close_button = QPropertyAnimation(self.close_button, b'geometry')
            self.animation_close_button.setDuration(500)
            self.animation_close_button.setStartValue(QtCore.QRect(800, 0, 40, 40))
            self.animation_close_button.setEndValue(QtCore.QRect(760, 0, 40, 40))

            # Запуск анимаций
            self.animation_menu.start()
            self.animation_close_button.start()

            # Отключение триггера анимации
            self.animation_trig = False

    def close_animation(self):
        """Анимация исчезновения при отсутвии действия в течении 3 секунд"""
        # Если триггер анимации неактивен
        if not self.animation_trig:
            # Создание анимации для меню
            self.animation_menu = QPropertyAnimation(self.menu_frame, b'geometry')
            self.animation_menu.setDuration(500)
            self.animation_menu.setStartValue(QtCore.QRect(0, 480 - 55, 800, 40))
            self.animation_menu.setEndValue(QtCore.QRect(0, 480, 800, 40))

            # Создание анимации для кнопки закрытия
            self.animation_close_button = QPropertyAnimation(self.close_button, b'geometry')
            self.animation_close_button.setDuration(500)
            self.animation_close_button.setStartValue(QtCore.QRect(760, 0, 40, 40))
            self.animation_close_button.setEndValue(QtCore.QRect(800, 0, 40, 40))

            # Запуск анимаций
            self.animation_menu.start()
            self.animation_close_button.start()

            # Включение триггера анимации
            self.animation_trig = True


if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)
    player = VideoPlayer()
    player.setWindowTitle("Player")
    player.resize(600, 400)
    player.show()
    sys.exit(app.exec_())
