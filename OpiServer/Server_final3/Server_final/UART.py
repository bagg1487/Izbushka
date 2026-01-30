import serial
from time import sleep
from Command import COMMAND as cmd
from ServoControl import *
from serial.tools import list_ports
class UART:

    def __init__(self, serial_port="/dev/ttyUSB0"):
        self.ser = serial.Serial(serial_port, 115200)  # Open port with baud rate
        self.servoControl = None
        self.uart_connection_flag = False
        self.uart_transmit_flag = True
        self.connect()


    def connect(self):
        """Соединение по доступному UART"""
        try:
            # Поиск и открытие доступного порта9600
            ports = list_ports.comports()
            port = next((p.device for p in ports), None)

            # Проверка наличия доступного порта
            if port is None:
                raise ValueError("No COM port found.")

            # Подключение к серийному порту
            self.ser = serial.Serial(port, baudrate=115200)
            self.ser.write((cmd.CMD_PING + "\r").encode())
            self.uart_transmit_flag = False
            sleep(0.5)

            # Проверка наличия данных в буфере приема
            if self.ser.inWaiting():
                received_data = self.serial_receive()

                # Проверка успешного подключения к UART
                if received_data[0] == cmd.CMD_PING:
                    print("UART работает")
                    if not self.uart_connection_flag:
                        self.uart_connection_flag = True
                        self.servoControl = ServoControl()
                        self.serial_send(cmd.CMD_CALIBRATION_ALL + self.servoControl.getCalibrationCommand() + "\n")
                    return 1
                else:
                    self.uart_connection_flag = False
                    print("UART не подключен")
            else:
                print("UART не подключен")
        except ValueError as ve:
            print("Error:", str(ve))
        except serial.SerialException as se:
            print("Serial port error:", str(se))
        except Exception as e:
            print("An error occurred:", str(e))
        return 0

    def serial_send(self, data):
        """Отправка данных по UART
        Аргументы: Строка или команда для отправки
        """
        try:
            # Попытка передачи данных через UART, если флаг передачи установлен
            if self.uart_transmit_flag:
                print(data)
                self.ser.write((data + "\r").encode())  # transmit data serially
        except serial.SerialException as se:
            print("Serial port error:", str(se))

    def serial_receive(self):
        """Прием информации по UART
        Возвращает: принятую строку
        """
        try:
            # Проверка на оставшиеся байты данных
            data_left = self.ser.inWaiting()
            # Чтение полученных данных и разделение их по символу новой строки
            received_data = self.ser.read(data_left).decode('utf-8').split('\n')
            # Отправка возвратного символа перевода строки
            self.ser.write("\r".encode())
            # Если получены данные, установка флага передачи UART в True
            if received_data:
                self.uart_transmit_flag = True
            return received_data
        except serial.SerialException as se:
            print("Serial port error:", str(se))

        print (received_data)                   #print received data

    def getDistance(self):
        """Запрос по UART расстояния с Ultrasonic датчика на STM32"""
        # Отправка команды SONIC по UART
        self.serial_send(cmd.CMD_SONIC)
        # Пауза для ожидания ответа
        sleep(0.1)
        # Проверка на оставшиеся байты данных
        data_left = self.ser.inWaiting()
        # Чтение полученных данных и разделение их по символу новой строки
        received_data = self.ser.read(data_left).decode('utf-8').split('\n')
        # Вывод первого элемента полученных данных
        print(received_data[0])
        # Возврат первого элемента полученных данных
        return received_data[0]
