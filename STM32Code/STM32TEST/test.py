import serial
import serial.tools.list_ports
import time
import sys
import keyboard

BAUNDRATE = 115200


def find_uart():
    ports = serial.tools.list_ports.comports()

    uart_keywords = [
        "STMicroelectronics", "STM", "STLink",
        "Silicon Labs", "CP210x", "CP2102",
        "USB Serial Port", "UART Bridge",
        "CH340", "CH341",
        "FTDI", "FT232"
    ]

    for port in ports:
        print(f"Найден: {port.device} - {port.description}")

        for keyword in uart_keywords:
            if keyword.lower() in port.description.lower():
                return port.device

    if ports:
        print("\nНе найден порт для uart. \nДоступные порты:")
        for i, port in enumerate(ports):
            print(f"   {i + 1}. {port.device} - {port.description}")
        print("\nВыберите порт:")
        try:
            choice = int(input("Введите номер: ")) - 1
            if 0 <= choice < len(ports):
                return ports[choice].device
        except:
            pass

    return None


def test_servo(port, baudrate=BAUNDRATE):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(1)

        print("\nУправление сервоприводом:")
        print("W - поднять (LOOK_UP)")
        print("S - опустить (LOOK_DOWN)")
        print("SPACE - вернутся в начальное положение (LOOK_DEFINE)")
        print("Q - выход из теста")

        while True:
            if keyboard.is_pressed('w'):
                ser.write(b'CMD_LOOK_UP\r')
                print("Отправлено: CMD_LOOK_UP")
                time.sleep(0.3)
            elif keyboard.is_pressed('s'):
                ser.write(b'CMD_LOOK_DOWN\r')
                print("Отправлено: CMD_LOOK_DOWN")
                time.sleep(0.3)
            elif keyboard.is_pressed('space'):
                ser.write(b'CMD_LOOK_DEFINE\r')
                print("Отправлено: CMD_LOOK_DEFINE")
                time.sleep(0.3)
            elif keyboard.is_pressed('q'):
                print("\nВыход из теста серво")
                break

    except Exception as e:
        print(f"Ошибка: {e}")
    finally:
        ser.close()

def test_PING(port, baudrate=BAUNDRATE):
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        print(f"Подключено к {port} со скоростью {baudrate}")
        time.sleep(2)

        for _ in range(2):
            ser.write(b'CMD_PING\r')
            time.sleep(0.5)

        if ser.in_waiting:
            response = ser.readline().decode().strip()
            print(f"Проверка связи\nОТВЕТ: {response}")
            return True
        else:
            print("Нет ответа")
            return False
    except Exception as e:
        print(f"Ошибка: {e}")
        return False
    finally:
        ser.close()


def test_rgb_led(port, baudrate=BAUNDRATE):
    print("\nПорты для подключения:\nGPIO 13 - красный\nGPIO 14 - зеленый\nGPIO 15 - синий")
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(1)
    except Exception as e:
        print(f"Ошибка: {e}")
        return

    colors = [
        (255, 0, 0, "КРАСНЫЙ"),
        (0, 255, 0, "ЗЕЛЕНЫЙ"),
        (0, 0, 255, "СИНИЙ"),
        (255, 255, 0, "ЖЕЛТЫЙ"),
        (255, 0, 255, "МАГЕНТА"),
        (0, 255, 255, "ЦИАН"),
        (255, 255, 255, "БЕЛЫЙ"),
        (0, 0, 0, "ВЫКЛЮЧЕНО"),
    ]

    try:
        for r, g, b, name in colors:
            cmd = f"CMD_LED#{r}#{g}#{b}\r"
            print(f"\n{name:15} -> {cmd.strip()}", end=" ", flush=True)

            ser.write(cmd.encode())
            ser.flush()
            time.sleep(1.5)

    except KeyboardInterrupt:
        print("\n\nТест прерван")
    finally:
        ser.close()
        for _ in range(10):
            keyboard.read_event(suppress=False)


def manual_port():
    ports = serial.tools.list_ports.comports()

    if not ports:
        print("Нет доступных COM портов")
        return None

    for i, port in enumerate(ports):
        print(f"   {i + 1}. {port.device} - {port.description}")

    try:
        choice = int(input("\nВыберите номер порта: ")) - 1
        if 0 <= choice < len(ports):
            return ports[choice].device
    except:
        pass

    return None


def set_baudrate():
    try:
        baudrate = input("\nВыберите скорость (по умолчанию 115200): ")
        if baudrate.strip():
            return int(baudrate)
    except ValueError:
        print("Введите число")
    return 115200


if __name__ == "__main__":
    port = find_uart()

    if not port:
        port = manual_port()

    if not port:
        sys.exit(1)

    print(f"\nИспользуется порт: {port}")
    print(f"Текущая скорость: {BAUNDRATE}")

    while True:
        try:
            print("1 - Изменить скорость (baudrate)")
            print("2 - Тест PING")
            print("3 - Тест RGB LED")
            print("4 - Тест поднятия туловища")
            print("5 - Выход")

            choice = input("\nВыбирите (1/2/3/4/5): ").strip()

            if choice == '1':
                BAUNDRATE = set_baudrate()
                print(f"Скорость изменена на: {BAUNDRATE}")

            elif choice == '2':
                test_PING(port, BAUNDRATE)

            elif choice == '3':
                test_rgb_led(port, BAUNDRATE)

            elif choice == '4':
                test_servo(port, BAUNDRATE)

            elif choice == '5':
                print("Выход из программы")
                sys.exit(0)

            else:
                print("Неверный выбор. Выберите (1/2/3/4/5)")
        except KeyboardInterrupt:
            sys.exit(0)
