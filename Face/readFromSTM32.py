# import serial
# import time
# import json
# import requests
# from datetime import datetime
#
# PORT = "/dev/ttyUSB0"
# BAUD = 115200
#
# latest_sensor_data = {
#     "timestamp": "",
#     "temperature": 0,
#     "humidity": 0,
#     "pressure": 0,
#     "gas": 0,
#     "light": 0
# }
#
#
# def read_sensor_data():
#     global latest_sensor_data
#     try:
#         ser = serial.Serial(PORT, BAUD, timeout=1)
#         print("Reading sensor data...\n")
#
#         while True:
#             try:
#                 line = ser.readline().decode("utf-8", errors="ignore").strip()
#                 if line:
#                     print(">>>", line)
#                     try:
#                         data = json.loads(line)
#                         if all(key in data for key in ['temperature', 'humidity', 'pressure', 'gas', 'light']):
#                             latest_sensor_data.update(data)
#                             latest_sensor_data['timestamp'] = datetime.now().isoformat()
#                     except:
#                         pass
#             except KeyboardInterrupt:
#                 print("Stopped.")
#                 break
#             except Exception as e:
#                 print(f"Error: {e}")
#                 time.sleep(1)
#
#     except Exception as e:
#         print(f"Serial error: {e}")
#
#
# def get_latest_sensor_data():
#     return latest_sensor_data.copy()
#
#
# if __name__ == "__main__":
#     read_sensor_data()
import time
import json
import random
from datetime import datetime

latest_sensor_data = {
    "timestamp": "",
    "temperature": 0,
    "humidity": 0,
    "pressure": 0,
    "gas": 0,
    "light": 0
}


def read_sensor_data():
    global latest_sensor_data
    print("Generating mock sensor data...\n")

    base_temp = 22.0
    base_humidity = 45.0
    base_pressure = 1013.0
    base_gas = 150
    base_light = 300

    while True:
        try:
            mock_data = {
                "temperature": round(base_temp + random.uniform(-2, 2), 1),
                "humidity": round(base_humidity + random.uniform(-5, 5), 1),
                "pressure": round(base_pressure + random.uniform(-10, 10), 1),
                "gas": int(base_gas + random.uniform(-20, 20)),
                "light": int(base_light + random.uniform(-50, 50))
            }

            latest_sensor_data.update(mock_data)
            latest_sensor_data['timestamp'] = datetime.now().isoformat()

            print(">>>", json.dumps(mock_data))
            time.sleep(2)

        except KeyboardInterrupt:
            print("Stopped.")
            break
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(1)


def get_latest_sensor_data():
    return latest_sensor_data.copy()


if __name__ == "__main__":
    read_sensor_data()