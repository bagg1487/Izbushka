import socket
import struct

import cv2
import numpy as np
from PIL import Image
import io
class Client:
    def __init__(self):
        """Создаем экземпляр клиента с определенными флагами и переменными"""
        self.tcp_flag = False  # Флаг для TCP-соединения
        self.video_flag = True  # Флаг для видео
        self.stop_video_thread = False  # Флаг для остановки потока видео
        self.stop_instruction_thread = False  # Флаг для остановки потока инструкций
        self.image = ''  # Переменная для хранения изображения
    def turn_on_client(self, ip):
        """Метод для включения клиента с заданным IP-адресом"""
        self.client_socket1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(ip)  # Выводим IP-адрес

    def turn_off_client(self):
        """Метод для выключения клиента"""
        try:
            self.client_socket.shutdown(2)  # Отключаем сокет TCP
            self.client_socket1.shutdown(2)  # Отключаем второй сокет TCP
            self.client_socket.close()  # Закрываем соединение сокета
            self.client_socket1.close()  # Закрываем соединение второго сокета
        except Exception as e:
            print(e)  # Выводим ошибку, если она возникает

    def receiving_video(self, ip):
        """Метод для получения видео с сервера"""
        stream_bytes = b''  # Инициализируем пустой байтовый поток
        try:
            self.client_socket.connect((ip, 8001))  # Подключаемся к серверу по указанному IP и порту 8001
            self.connection = self.client_socket.makefile('rb')  # Создаем файлоподобный объект для чтения данных
        except:
            print("command port connect failed")  # Выводим сообщение об ошибке при неудачном подключении
            pass
        while True:
            try:
                stream_bytes = self.connection.read(4)  # Читаем 4 байта из потока
                leng = struct.unpack('<L', stream_bytes[:4])  # Распаковываем длину изображения из первых 4 байт
                jpg = self.connection.read(leng[0])  # Читаем изображение заданной длины
                if self.is_valid_image_4_bytes(jpg):  # Проверяем, является ли изображение допустимым
                    if self.video_flag:  # Если флаг видео активен
                        # Декодируем изображение и сохраняем его в переменную image
                        self.image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        self.video_flag = False  # Отключаем флаг видео
            except BaseException as e:
                print(e)  # Выводим ошибку, если она возникает
                break

    def send_data(self, data):
        """Метод для отправки данных через TCP"""
        if self.tcp_flag:  # Если флаг TCP активен
            try:
                self.client_socket1.send(data.encode('utf-8'))  # Отправляем данные, закодированные в utf-8
            except Exception as e:
                print(e)  # Выводим ошибку, если она возникает

    def receive_data(self):
        """Метод для приема данных через TCP"""
        data = ""  # Инициализируем переменную для данных
        data = self.client_socket1.recv(1024).decode('utf-8')  # Получаем данные, декодируя их из utf-8
        return data  # Возвращаем полученные данные

    def is_valid_image_4_bytes(self, buf):
        """Метод для проверки допустимости изображения"""
        bValid = True  # Инициализируем переменную, сигнализирующую о допустимости изображения
        if buf[6:10] in (b'JFIF', b'Exif'):  # Если изображение в формате JFIF или Exif
            if not buf.rstrip(b'\0\r\n').endswith(b'\xff\xd9'):  # Если последние байты не соответствуют маркеру конца изображения
                bValid = False  # Помечаем изображение как недопустимое
        else:
            try:
                Image.open(io.BytesIO(buf)).verify()  # Пытаемся открыть и проверить изображение
            except:
                bValid = False  # Если возникает ошибка, помечаем изображение как недопустимое
        return bValid  # Возвращаем результат проверки допустимости изображения