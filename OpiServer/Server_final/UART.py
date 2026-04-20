import serial
import time
from time import sleep
from Command import COMMAND as cmd
from ServoControl import *
from serial.tools import list_ports

class UART:
    def __init__(self, serial_port='/dev/ttyUSB0', max_retries=10, retry_delay=1):
        self.ser = None
        self.servoControl = None
        self.uart_connection_flag = False
        self.uart_transmit_flag = True
        
        for attempt in range(max_retries):
            try:
                self.ser = serial.Serial(serial_port, 115200, timeout=1)
                print(f"UART подключен на {serial_port} (попытка {attempt + 1})")
                self.connect()
                break
            except serial.SerialException as e:
                if attempt < max_retries - 1:
                    print(f"Попытка {attempt + 1}/{max_retries} не удалась: {e}")
                    print(f"Жду {retry_delay} секунд...")
                    time.sleep(retry_delay)
                else:
                    print(f"Не удалось подключиться к {serial_port} после {max_retries} попыток")
                    # Пробуем найти другой порт
                    self.connect()
                    break

    def connect(self):
        """Соединение по доступному UART"""
        try:
            # Поиск всех доступных портов
            ports = list_ports.comports()
            
            if not ports:
                print("Нет доступных COM портов")
                self.uart_connection_flag = False
                return 0
            
            print(f"Найдены порты: {[p.device for p in ports]}")
            
            # Попробовать все найденные USB порты
            for port_info in ports:
                port = port_info.device
                if 'USB' in port or 'ACM' in port:  # USB или ACM устройства
                    try:
                        print(f"Пробую подключиться к {port}...")
                        
                        # Закрыть существующее соединение, если есть
                        if self.ser and self.ser.is_open:
                            self.ser.close()
                        
                        # Попытка подключения
                        self.ser = serial.Serial(port, baudrate=115200, timeout=1)
                        self.ser.write((cmd.CMD_PING + "\r").encode())
                        sleep(0.5)
                        
                        if self.ser.inWaiting():
                            received_data = self.serial_receive()
                            if received_data and len(received_data) > 0 and received_data[0] == cmd.CMD_PING:
                                print(f"UART работает на {port}")
                                if not self.uart_connection_flag:
                                    self.uart_connection_flag = True
                                    self.servoControl = ServoControl()
                                    self.serial_send(cmd.CMD_CALIBRATION_ALL + 
                                                   self.servoControl.getCalibrationCommand() + "\n")
                                return 1
                        self.ser.close()
                    except Exception as e:
                        print(f"Ошибка при подключении к {port}: {e}")
                        continue
            
            print("Не удалось подключиться ни к одному из портов")
            self.uart_connection_flag = False
            
        except Exception as e:
            print(f"Ошибка подключения: {e}")
            self.uart_connection_flag = False
        
        return 0

    def serial_send(self, data):
        """Отправка данных по UART"""
        try:
            if self.uart_transmit_flag and self.ser and self.ser.is_open:
                self.ser.write((data + "\r").encode())
        except serial.SerialException as se:
            print(f"Ошибка отправки данных: {str(se)}")
        except AttributeError:
            print("UART порт не инициализирован")

    def serial_receive(self):
        """Прием информации по UART"""
        try:
            if self.ser and self.ser.is_open:
                data_left = self.ser.inWaiting()
                if data_left > 0:
                    received_data = self.ser.read(data_left).decode('utf-8', errors='ignore').split('\n')
                    self.ser.write("\r".encode())
                    
                    if received_data:
                        self.uart_transmit_flag = True
                        print(f"Получено: {received_data}")
                    return received_data
            return []
        except serial.SerialException as se:
            print(f"Ошибка приема данных: {str(se)}")
            return []
        except AttributeError:
            print("UART порт не инициализирован")
            return []

    def getDistance(self):
        """Запрос расстояния с Ultrasonic датчика"""
        try:
            if self.ser and self.ser.is_open:
                self.serial_send(cmd.CMD_SONIC)
                sleep(0.1)
                                                                                
                data_left = self.ser.inWaiting()
                if data_left > 0:
                    received_data = self.ser.read(data_left).decode('utf-8', errors='ignore').split('\n')
                    if received_data and len(received_data) > 0:
                        print(f"Расстояние: {received_data[0]}")
                        return received_data[0]
                return "0"
            return "0"
        except Exception as e:
            print(f"Ошибка получения расстояния: {e}")
            return "0"