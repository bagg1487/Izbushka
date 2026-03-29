from datetime import datetime
import segno
from PyQt5.QtCore import QPoint
from PyQt5.QtGui import QImage, QPainter, QColor
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
# from RockPapperScrissor import *


class Service:
    """Класс для подключения основных модулей, содержащий функции основной логики работы и взаимодействия между ними """
    def __init__(self):

        # Инициализация объектов и модулей:
        self.speach_rec = Speaker() # Важно: инициализируем голос здесь
        self.face_action = FaceAction()  # Модуль для работы с действиями лица
        self.hand = HandAndFingerDetectionModule()  # Модуль для обнаружения рук и пальцев
        self.music_player = MusicPlayer()  # Модуль для проигрывания музыки
        self.video_player = VideoPlayer()  # Модуль для проигрывания видео
        self.radio_player = RadioPlayer()  # Модуль для проигрывания радио
        self.recognizer = Recognizer()  # Объект для распознавания
        self.db_manager = DatabaseManager('main_database.db')  # Менеджер базы данных
        self.db_manager.create_user_table()  # Создание таблицы пользователей
        self.db_manager.create_audio_table() # Создание таблицы аудиофайлов
        self.play_video = PlayVideoViaBrowser()  # Воспроизведение видео через браузер
        # self.test_rps = RockPapperScrissor()  # Тест игры "Камень, ножницы, бумага"
        self.key_phrase_analizer = KeyPhraseAnalizer()  # Анализатор ключевых фраз
        self.os_name = platform.system()
        
        # Подключаем Gemini
        self.ai_assistant = GeminiAssistant() 
        
    def weather_check(self, query):
        """Новая функция для проверки погоды"""
        text = self.ai_assistant.get_weather(query)
        print(text)
        self.speach_rec.speak(text) 

    def lets_talk(self, data):
        """Обновленный режим разговора через Gemini"""
        # Если спрашивают про погоду внутри диалога
        if "погод" in data.lower():
            return self.ai_assistant.get_weather(data)
            
        # Иначе просто болтаем
        answer_text = self.ai_assistant.ask(data)
        return answer_text

    def get_face_frame(self):
        """Рисование рамки вокруг распознанного лица"""
        if self.face_action.face_detection_trig:
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

                for ((top, right, bottom, left), name) in zip(boxes,
                                                              names):  # рисует рамку и имя для каждого определенного лица
                    y = top - 15 if top - 15 > 15 else top + 15

                    # Создаем красный квадрат на картинке
                    painter = QPainter(image)
                    painter.setBrush(QColor(255, 0, 0, 0))  # Устанавливаем красный цвет с прозрачностью 50%
                    painter.drawRect(left, top, right - left, bottom - top)  # Рисуем квадрат
                    painter.drawText(QPoint(left, y), name)
                    painter.end()

            return image # Возвращаем изображение с отображенными лицами и их именами

    def game_mode(self, command):
        """Вход в режим игры"""
        # Если команда не содержит ключевую фразу "кнб"
        if not 'кнб' in command:
            print("Давайте! скажите камень-ножницы-бумага!")

            # Прослушивать фразы пользователя до тех пор, пока не будет выбрана игра
            while True:
                self.recognizer.listening()  # Прослушивание фраз
                full_phrase = self.recognizer.get_full_phrase()  # Получение полной фразы, если она есть

                # Если получена полная фраза
                if full_phrase:
                    key_phrase = self.key_phrase_analizer.analyzer(
                        full_phrase)  # Анализируем фразу на наличие ключевых слов
                    if 'кнб' in key_phrase:  # Если ключевое слово "кнб" найдено
                        self.test_rps.play_game_trig = True  # Устанавливаем флаг начала игры в камень-ножницы-бумага
                        self.test_rps.start_game_RPS(self.recognizer)  # Запускаем игру
                        break  # Выходим из цикла

                    print(full_phrase)  # Выводим полученную фразу для отладки

        # Если команда содержит ключевую фразу "кнб"
        else:
            key_phrase = command  # Присваиваем ключевую фразу переменной
            if 'кнб' in key_phrase:  # Если ключевое слово "кнб" найдено
                self.test_rps.play_game_trig = True  # Устанавливаем флаг начала игры в камень-ножницы-бумага
                self.test_rps.start_game_RPS(self.recognizer)  # Запускаем игру

    def time_now(self):
        """Озвучивание текущего времени"""
        dt = None
        if self.os_name == "Windows":
            dt = datetime.datetime.now().strftime("%H %M")  # получаем текущее время
        elif self.os_name == "Linux":
            dt = datetime.datetime.now().strftime("%H %M")  # получаем текущее время
        print('Сейчас' + dt)  # выводим в консоль

    def calculate(self, str1):
        """Функция Посчитай (мини-калькулятор)"""
        # Разделение строки на слова
        words = str1.split()
        print(words)
        try:
            # объявляем переменные цифр
            number1 = None
            number2 = None
            for number in words:
                try:
                    number = int(number)
                    # Если элемент массива - цифра и первая переменная еще не заполнена
                    if number1 is None:
                        # Заполнить первую переменную
                        number1 = int(number)
                    # Если элемент массива - цифра и вторая переменная еще не заполнена
                    elif number2 is None:
                        # Заполнить вторую переменную
                        number2 = int(number)
                        # Найдены обе цифры, выйти из цикла
                        break
                except:
                    pass

            # Создание пустой переменной для знака
            operator = None
            # Проход по всем элементам массива
            for item in words:
                # Если элемент массива - один из знаков
                if item in ['х', '/', '+', '-']:
                    # Заполнить переменную знаком и выйти из цикла
                    operator = item
            # выполнение операции
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
            # если что-то пошло не так, просим повторить, иначе выводим ответ
            if result is None:
                print('Я не поняла, повторите')  # выводим в консоль
            else:
                res = 'Результат ' + str(result)
                print(res)  # выводим в консоль
        except:
            print('Я не поняла, повторите')

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
        print(sentence)
        print('Ищу...')
        results = wikipedia.summary([sentence], sentences=1)
        print("В википедии написано")
        print(results)

    def makeQR(self, qrtext, OL):
        """создание QR кодов"""
        # создаем QR-код с нужным текстом и сохраняем в png
        qrcode = segno.make_qr(qrtext)
        qrcode.save("qr.png")
        # Указываем путь к файлу картинки PNG
        current_dir = os.path.dirname(__file__)
        path_to_pngxxx = os.path.join(current_dir, "qr.png")
        # Нужно создать экземпляр картинки QImage, для работы с PyQt5
        image = QImage(path_to_pngxxx)
        pixmap = QPixmap.fromImage(image)
        pixmap = pixmap.scaled(400, 400)
        # OL(Object Layout) - слой для вывода на экран
        OL.setAlignment(Qt.AlignCenter)
        OL.setPixmap(pixmap)
        print("Сделано!")

    def joke(self):
        """Воспроизведение шутки/анекдота"""
        # список шуток
        elements = []
        # Открытие текстового документа
        f = open('jokes.txt', 'r+', encoding='UTF-8')
        # Чтение всех строк из файла
        elements = f.readlines()
        # считаем сколько у нас шуток
        lenjokes = len(elements)
        # выбираем случайную шутку
        x = random.randint(0, lenjokes - 1)
        # выводим шутку в консоль
        print(elements[x])
