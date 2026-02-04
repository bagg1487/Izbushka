# import sys
# from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
# from PyQt5.QtCore import QUrl
# from PyQt5.QtGui import QPixmap, QMovie
#
# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()
#
#         self.setWindowTitle("Display Animated GIF")
#
#         # Создаем виджет QLabel
#         self.gif_label = QLabel()
#
#         # Создаем объект QMovie и загружаем анимацию
#         movie = QMovie("PRS/idle.gif")  # Путь к вашему анимированному GIF
#         self.gif_label.setMovie(movie)
#
#         # Запускаем анимацию
#         movie.start()
#
#         # Размещаем виджет на форме
#         central_widget = QWidget(self)
#         layout = QVBoxLayout(central_widget)
#         layout.addWidget(self.gif_label)
#         self.setCentralWidget(central_widget)
#
# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = MainWindow()
#     window.show()
#     sys.exit(app.exec_())
# import sys
# import time
# from PyQt5.QtCore import QThread, QObject, pyqtSignal, Qt
# from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
# from PyQt5.QtGui import QPixmap, QImage
# from PIL import Image, ImageSequence
#
# class Worker(QObject):
#     finished = pyqtSignal(QImage)
#
#     def __init__(self):
#         super().__init__()
#
#     def run(self):
#         # Загрузка анимированного GIF-изображения
#         gif_image = Image.open("PRS/idle.gif")  # Путь к вашему анимированному GIF
#
#         # Создание списка кадров изображения
#         frames = [frame.copy() for frame in ImageSequence.Iterator(gif_image)]
#
#         # Преобразование каждого кадра в QImage
#         for frame in frames:
#             img = frame.convert("RGBA")
#             data = img.tobytes("raw", "RGBA")
#             q_image = QImage(data, img.size[0], img.size[1], QImage.Format_ARGB32)
#             self.finished.emit(q_image)
#             time.sleep(0.1)  # Задержка для эмуляции анимации
#
# class MainWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()
#
#         self.setWindowTitle("Display Animated GIF")
#         self.resize(400, 300)
#
#         # Создаем виджет QLabel для отображения анимации
#         self.gif_label = QLabel()
#         self.gif_label.setAlignment(Qt.AlignCenter)
#
#         # Создаем объект Worker и поток
#         self.worker = Worker()
#         self.thread = QThread()
#
#         # Подключаем сигналы и слоты
#         self.worker.moveToThread(self.thread)
#         self.worker.finished.connect(self.show_animation)
#         self.thread.started.connect(self.worker.run)
#
#         # Запускаем поток
#         self.thread.start()
#
#     def show_animation(self, q_image):
#         # Устанавливаем QImage в QLabel
#         pixmap = QPixmap.fromImage(q_image)
#         self.gif_label.setPixmap(pixmap)
#
#         # Размещаем QLabel на форме
#         central_widget = QWidget(self)
#         layout = QVBoxLayout(central_widget)
#         layout.addWidget(self.gif_label)
#         self.setCentralWidget(central_widget)
#
#     def closeEvent(self, event):
#         self.thread.quit()
#         self.thread.wait()
#         event.accept()
#
# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = MainWindow()
#     window.show()
#     sys.exit(app.exec_())

import sys

from PyQt5.QtGui import QMovie
from PyQt5.QtWidgets import QApplication, QLabel
from PyQt5.QtCore import QTimer, Qt, pyqtSignal


class MainWindow(QLabel):
    animationFinished = pyqtSignal()
    def __init__(self, gifs):
        super().__init__()
        self.gifs = gifs
        self.current_index = 0
        self.movie = QMovie(self.gifs[self.current_index])
        self.setMovie(self.movie)
        self.movie.setSpeed(300)
        self.movie.start()
        self.setFixedSize(800, 480)
        self.setAlignment(Qt.AlignCenter)
        self.update_gif()

        self.movie.frameChanged.connect(self.check_animation_finished)

        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.next_gif)
        # self.timer.start(2000)  # Интервал переключения в миллисекундах



    def update_gif(self):
            if self.movie:
                self.movie.stop()
                gif_path = self.gifs[self.current_index]
                self.movie.setFileName(gif_path)
                self.setMovie(self.movie)
                self.movie.setSpeed(300)
                self.movie.start()

    def check_animation_finished(self, frame_number):
        if frame_number == self.movie.frameCount() - 1:
            self.animationFinished.emit()
    def next_gif(self):
        self.current_index = (self.current_index + 1) % len(self.gifs)
        self.update_gif()


if __name__ == '__main__':
    app = QApplication(sys.argv)

    # Список GIF-файлов
    gif_files = ["PRS/idle.gif", "PRS/papper_lose.gif", "PRS/rock_lose.gif"]

    window = MainWindow(gif_files)
    window.show()
    window.animationFinished.connect(window.next_gif)

    sys.exit(app.exec_())
