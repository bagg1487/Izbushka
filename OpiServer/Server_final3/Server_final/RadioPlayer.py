import os
import platform

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QPropertyAnimation, QTimer, QDir
from PyQt5.QtGui import QPixmap

# Установите переменные окружения для VLC
os.environ['VLC_PLUGIN_PATH'] = '/usr/lib/vlc/plugins'
os.environ['PULSE_SERVER'] = 'none'  # Отключаем PulseAudio

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
        
        # Создаем VLC инстанс с параметрами ALSA
        try:
            # Параметры для использования ALSA напрямую
            vlc_args = [
                '--aout=alsa',
                '--alsa-audio-device=hw:CARD=rockchipes8388,DEV=0',  
                '--no-pulse',
                '--network-caching=3000',
                '--quiet'
            ]
            
            # Создаем VLC инстанс с параметрами
            vlc_instance = vlc.Instance(' '.join(vlc_args))
            self.player = vlc_instance.media_player_new()
            print("VLC инициализирован с параметрами ALSA")
            
        except Exception as e:
            print(f"Ошибка инициализации VLC с ALSA: {e}")
            try:
                # Резервный вариант - попробуем с другими параметрами
                vlc_instance = vlc.Instance('--aout=alsa', '--alsa-audio-device=default')
                self.player = vlc_instance.media_player_new()
                print("VLC инициализирован с ALSA default")
            except Exception as e2:
                print(f"Ошибка резервной инициализации VLC: {e2}")
                # Последний вариант - обычный плеер
                self.player = vlc.MediaPlayer()
                print("Используется обычный VLC MediaPlayer")
        
        # здесь будет храниться список всех радиостанций
        self.elements = []
        # Открытие текстового документа
        parent_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Сначала попробуем тестовые радиостанции
        test_file = parent_dir + '/radio_test.txt'
        main_file = parent_dir + '/radio.txt'
        
        # Используем тестовый файл, если он существует
        if os.path.exists(test_file):
            print("Использую тестовый файл радиостанций")
            f = open(test_file, 'r+', encoding='UTF-8')
        elif os.path.exists(main_file):
            print("Использую основной файл радиостанций")
            f = open(main_file, 'r+', encoding='UTF-8')
        else:
            print("Файл со списком радиостанций не найден")
            # Используем тестовые радиостанции по умолчанию
            self.elements = [
                "http://icecast.vgtrk.cdnvideo.ru/mayakfm_mp3_192kbps",
                "http://nashe1.hostingradio.ru/nashe-128.mp3",
                "http://air.radiorecord.ru:805/rr_320"
            ]
            self.current = 0
            self.player_stat = 1
            self.animation_trig = True
            return
            
        # Читение всех строк из файла
        all_lines = f.readlines()
        f.close()
        
        # Обрабатываем все строки, пропуская комментарии
        for line in all_lines:
            line = line.strip()
            if line and not line.startswith('#'):  # Пропускаем пустые строки и комментарии
                self.elements.append(line)
                
        if not self.elements:
            print("В файле радиостанций нет URL")
            self.elements = [
                "http://icecast.vgtrk.cdnvideo.ru/mayakfm_mp3_192kbps",
                "http://nashe1.hostingradio.ru/nashe-128.mp3"
            ]
        
        # Текущий в начале всегда 0
        self.current = 0
        self.player_stat = 1

        # триггер проигрывания анимации
        self.animation_trig = True
        
        print(f"Загружено {len(self.elements)} радиостанций")

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
        else:
            # Если радио не активно, включаем его
            self.radio_active_trig = True
            self.play()
            self.player_stat = 0
            self.play_pause_button.setPixmap(QPixmap(QDir.currentPath() + "/player_button/pause.png"))
            
        # Установка флага окончания трека
        self.end_track_trig = True

    def play(self):  # включение радио
        """Функция запуска радио"""
        if not self.elements:
            print("Нет радиостанций для воспроизведения")
            return
            
        print(f"Пытаюсь воспроизвести радиостанцию: {self.elements[self.current]}")
        
        try:
            # выбираем радиостанцию
            self.media = vlc.Media(self.elements[self.current])
            # передаем плееру радиостанцию
            self.player.set_media(self.media)
            
            # Устанавливаем громкость
            self.player.audio_set_volume(30)
            
            # Воспроизводим радиостанцию
            result = self.player.play()
            if result == -1:
                print("Ошибка воспроизведения радио")
            else:
                print(f"Радио запущено успешно: {self.elements[self.current]}")
                self.radio_active_trig = True
                
        except Exception as e:
            print(f"Ошибка при запуске радио: {e}")
            # Попробуем следующую радиостанцию
            self.current = (self.current + 1) % len(self.elements)
            print(f"Пробую следующую радиостанцию...")

    def pause(self):
        """Функция постановки радио на паузу"""
        if self.radio_active_trig:
            self.player_stat = 1
            try:
                self.player.stop()
            except Exception as e:
                print(f"Ошибка при остановке радио: {e}")
            self.radio_active_trig = False
            print("Радио остановлено")

    def next_rad(self, event=None):
        """Функция переключения радио на следующую волну"""
        if not self.elements:
            return
            
        # останавливаем текущее радио
        try:
            self.player.stop()
        except:
            pass
            
        if self.radio_active_trig:
            # Увеличиваем индекс текущего трека
            self.current = (self.current + 1) % len(self.elements)
            print(f"Переключаю на радиостанцию {self.current + 1}/{len(self.elements)}")
            
            # Запускаем новую радиостанцию
            self.play()

    def previous_rad(self, event=None):  # прошлая волна
        """Функция переключения радио на прошлую волну"""
        if not self.elements:
            return
            
        # останавливаем радио
        try:
            self.player.stop()
        except:
            pass
            
        if self.radio_active_trig:
            # Уменьшаем индекс текущего трека
            self.current = (self.current - 1) % len(self.elements)
            print(f"Переключаю на радиостанцию {self.current + 1}/{len(self.elements)}")
            
            # Запускаем новую радиостанцию
            self.play()

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