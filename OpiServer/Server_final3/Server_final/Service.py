import pickle
import platform
import random
import os
from datetime import datetime
import segno
from PyQt5.QtCore import QPoint, Qt
from PyQt5.QtGui import QImage, QPainter, QColor, QPixmap
from wikipedia import wikipedia
from DatabaseManager import *
from HandAndFingerDetectionModule import *
from MusicPlayer import *
from yandexGPT_API import *
from RadioPlayer import *
from FaceAction import *
from PlayVideoViaBrowser import *
from VideoPlayer import *
from KeyPhraseAnalizer import *
from Recognizer import *
from RockPapperScrissor import *


class Service:
    """Класс для подключения основных модулей, содержащий функции основной логики работы и взаимодействия между ними"""
    def __init__(self):
        # Инициализация объектов и модулей:
        self.face_action = FaceAction()  # Модуль для работы с действиями лица
        self.hand = HandAndFingerDetectionModule()  # Модуль для обнаружения рук и пальцев
        self.music_player = MusicPlayer()  # Модуль для проигрывания музыки
        self.video_player = VideoPlayer()  # Модуль для проигрывания видео
        self.radio_player = RadioPlayer()  # Модуль для проигрывания радио
        
        try:
            self.recognizer = Recognizer()  # Объект для распознавания
        except Exception as e:
            print(f"Ошибка инициализации Recognizer: {e}")
            self.recognizer = None
            
        self.db_manager = DatabaseManager('main_database.db')  # Менеджер базы данных
        self.db_manager.create_user_table()  # Создание таблицы пользователей
        self.db_manager.create_audio_table()  # Создание таблицы аудиофайлов
        self.play_video = PlayVideoViaBrowser()  # Воспроизведение видео через браузер
        self.test_rps = RockPapperScrissor()  # Тест игры "Камень, ножницы, бумага"
        self.key_phrase_analizer = KeyPhraseAnalizer()  # Анализатор ключевых фраз
        self.os_name = platform.system()

    def get_face_frame(self):
        """Рисование рамки вокруг распознанного лица"""
        if self.face_action.face_detection_trig:
            try:
                # Получаем данные всех пользователей из базы данных
                all_users_BLOB = self.db_manager.get_users()
                know_names = []
                metrix = []
                # Для каждого пользователя извлекаем его кодировки лиц
                for user in all_users_BLOB:
                    encodings = pickle.loads(user[2])
                    for encoding in encodings:
                        metrix.append(encoding)
                        know_names.append(user[1])

                # Создаем словарь с данными для распознавания лиц
                data = {"encodings": metrix, "names": know_names}

                # Обнаружение лиц на изображении
                boxes, names = self.face_action.face_detection(data)

                # Создаем изображение для отображения результатов
                image = QImage(800, 480, QImage.Format_ARGB32_Premultiplied)

                # Устанавливаем все пиксели картинки в прозрачные
                image.fill(Qt.transparent)
                # Если были обнаружены лица
                if names:
                    for ((top, right, bottom, left), name) in zip(boxes, names):
                        y = top - 15 if top - 15 > 15 else top + 15

                        # Создаем красный квадрат на картинке
                        painter = QPainter(image)
                        painter.setBrush(QColor(255, 0, 0, 128))  # Красный цвет с прозрачностью 50%
                        painter.drawRect(left, top, right - left, bottom - top)  # Рисуем квадрат
                        painter.drawText(QPoint(left, y), name)
                        painter.end()

                return image  # Возвращаем изображение с отображенными лицами и их именами
            except Exception as e:
                print(f"Ошибка в get_face_frame: {e}")
                return QImage(800, 480, QImage.Format_ARGB32_Premultiplied)
        return None

    def game_mode(self, command):
        """Вход в режим игры"""
        # Если команда не содержит ключевую фразу "кнб"
        if 'кнб' not in command:
            print("Давайте! скажите камень-ножницы-бумага!")

            # Прослушивать фразы пользователя до тех пор, пока не будет выбрана игра
            while True:
                if self.recognizer:
                    self.recognizer.listening()  # Прослушивание фраз
                    full_phrase = self.recognizer.get_full_phrase()  # Получение полной фразы, если она есть

                    # Если получена полная фраза
                    if full_phrase:
                        key_phrase = self.key_phrase_analizer.analyzer(
                            full_phrase)  # Анализируем фразу на наличие ключевых слов
                        if 'кнб' in key_phrase:  # Если ключевое слово "кнб" найдено
                            self.test_rps.play_game_trig = True  # Устанавливаем флаг начала игры
                            self.test_rps.start_game_RPS(self.recognizer)  # Запускаем игру
                            break  # Выходим из цикла
                else:
                    break

        # Если команда содержит ключевую фразу "кнб"
        else:
            key_phrase = command  # Присваиваем ключевую фразу переменной
            if 'кнб' in key_phrase:  # Если ключевое слово "кнб" найдено
                self.test_rps.play_game_trig = True  # Устанавливаем флаг начала игры
                if self.recognizer:
                    self.test_rps.start_game_RPS(self.recognizer)  # Запускаем игру

    def time_now(self):
        """Озвучивание текущего времени"""
        dt = datetime.now().strftime("%H:%M")  # получаем текущее время
        print(f'Сейчас {dt}')  # выводим в консоль
        return dt

    def calculate(self, str1):
        """Функция Посчитай (мини-калькулятор)"""
        # Разделение строки на слова
        words = str1.split()
        try:
            # объявляем переменные цифр
            number1 = None
            number2 = None
            for word in words:
                try:
                    number = int(word)
                    # Если элемент массива - цифра и первая переменная еще не заполнена
                    if number1 is None:
                        # Заполнить первую переменную
                        number1 = number
                    # Если элемент массива - цифра и вторая переменная еще не заполнена
                    elif number2 is None:
                        # Заполнить вторую переменную
                        number2 = number
                        # Найдены обе цифры, выйти из цикла
                        break
                except ValueError:
                    pass

            # Создание пустой переменной для знака
            operator = None
            # Проход по всем элементам массива
            for item in words:
                # Если элемент массива - один из знаков
                if item in ['х', '/', '+', '-', '*']:
                    # Заполнить переменную знаком и выйти из цикла
                    operator = item
                    break
            
            # выполнение операции
            if operator == '+':
                result = number1 + number2
            elif operator == '-':
                result = number1 - number2
            elif operator == 'х' or operator == '*':
                result = number1 * number2
            elif operator == '/':
                if number2 != 0:
                    result = number1 / number2
                else:
                    result = None
            else:
                result = None
            
            # если что-то пошло не так, просим повторить, иначе выводим ответ
            if result is None:
                return 'Я не поняла, повторите'
            else:
                return f'Результат {result}'
        except Exception:
            return 'Я не поняла, повторите'

    def wikisearch(self, order):
        """Функция поиска в википедии"""
        words = order.split()
        # Найти индекс слова в массиве
        try:
            index = words.index("википедии")
        except ValueError:
            return []
        # Оставить элементы после слова
        input_list = words[index + 1:]
        # Преобразовать массив в строку
        sentence = " ".join(map(str, input_list))
        print(f'Ищу "{sentence}"...')
        try:
            results = wikipedia.summary(sentence, sentences=1)
            print("В википедии написано:")
            print(results)
            return results
        except Exception as e:
            print(f"Ошибка поиска в Википедии: {e}")
            return None

    def makeQR(self, qrtext, OL):
        """создание QR кодов"""
        try:
            # создаем QR-код с нужным текстом и сохраняем в png
            qrcode = segno.make_qr(qrtext)
            qrcode.save("qr.png")
            # Указываем путь к файлу картинки PNG
            current_dir = os.path.dirname(__file__)
            path_to_png = os.path.join(current_dir, "qr.png")
            # Нужно создать экземпляр картинки QImage, для работы с PyQt5
            image = QImage(path_to_png)
            pixmap = QPixmap.fromImage(image)
            pixmap = pixmap.scaled(400, 400)
            # OL(Object Layout) - слой для вывода на экран
            OL.setAlignment(Qt.AlignCenter)
            OL.setPixmap(pixmap)
            print("QR-код создан!")
        except Exception as e:
            print(f"Ошибка создания QR-кода: {e}")

    def joke(self):
        """Воспроизведение шутки/анекдота"""
        try:
            # список шуток
            elements = []
            # Открытие текстового документа
            with open('jokes.txt', 'r', encoding='UTF-8') as f:
                # Чтение всех строк из файла
                elements = f.readlines()
            # считаем сколько у нас шуток
            lenjokes = len(elements)
            if lenjokes > 0:
                # выбираем случайную шутку
                x = random.randint(0, lenjokes - 1)
                # выводим шутку в консоль
                joke_text = elements[x].strip()
                print(joke_text)
                return joke_text
            else:
                return "Шутки закончились!"
        except FileNotFoundError:
            return "Файл с шутками не найден"
        except Exception as e:
            return f"Ошибка чтения шуток: {e}"

    def lets_talk(self, data):
        """Вход в режим разговора"""
        try:
            answer_text = YandexAPI().request(data)
            return answer_text
        except Exception as e:
            print(f"Ошибка Yandex API: {e}")
            return "Извините, произошла ошибка при обработке запроса"