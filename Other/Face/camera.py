import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import urllib.request
import os
import numpy as np
import collections
import threading
import time
import requests
import queue

# Глобальные переменные для управления рисованием
drawing_active = False
drawing_thread = None
stop_drawing_flag = False

# Переменные для рисования
drawing_canvas = None
last_point = None
draw_color = (0, 255, 0)  # Зеленый по умолчанию
brush_thickness = 8
color_index = 0
colors = [(0, 255, 0), (0, 0, 255), (255, 0, 0), (255, 255, 255)]  # Зеленый, Красный, Синий, Белый
color_names = ["ЗЕЛЕНЫЙ", "КРАСНЫЙ", "СИНИЙ", "БЕЛЫЙ"]

# Очереди для сглаживания
finger_positions = collections.deque(maxlen=5)
gesture_history = collections.deque(maxlen=10)

# URL для отправки изменений выражения лица
EXPRESSION_API_URL = "http://127.0.0.1:8000/change-expression"

# Глобальные ссылки на функции из main
update_frame_callback = None
frame_queue_ref = None


def send_expression(expression):
    """Отправка выражения лица на сервер"""
    try:
        requests.post(
            EXPRESSION_API_URL,
            json={"expression": expression},
            timeout=1
        )
        print(f"Выражение лица изменено: {expression}")
    except Exception as e:
        print(f"Ошибка отправки выражения: {e}")


def download_model():
    """Скачивание модели распознавания жестов"""
    model_url = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
    model_path = "hand_landmarker.task"

    if not os.path.exists(model_path):
        try:
            print("Скачивание модели распознавания жестов...")
            urllib.request.urlretrieve(model_url, model_path)
            print("Модель загружена успешно!")
        except Exception as e:
            print(f"Ошибка загрузки модели: {e}")
            return None
    return model_path


def init_canvas(frame_shape):
    """Создает прозрачный холст для рисования"""
    return np.zeros((frame_shape[0], frame_shape[1], 3), dtype=np.uint8)


def detect_gesture(hand_landmarks, frame_shape):
    """Определение жеста по ключевым точкам руки"""
    h, w = frame_shape[:2]

    # Получаем ключевые точки пальцев
    thumb_tip = hand_landmarks[4]
    index_tip = hand_landmarks[8]
    index_pip = hand_landmarks[6]
    index_mcp = hand_landmarks[5]
    middle_tip = hand_landmarks[12]
    middle_pip = hand_landmarks[10]
    middle_mcp = hand_landmarks[9]
    ring_tip = hand_landmarks[16]
    ring_pip = hand_landmarks[14]
    ring_mcp = hand_landmarks[13]
    pinky_tip = hand_landmarks[20]
    pinky_pip = hand_landmarks[18]
    pinky_mcp = hand_landmarks[17]

    def is_finger_up_simple(tip, pip, mcp):
        """Проверка, поднят ли палец (упрощенная версия)"""
        tip_to_pip = pip.y - tip.y
        pip_to_mcp = mcp.y - pip.y
        return tip_to_pip > 0.05 and pip_to_mcp > 0.03

    # Проверяем какие пальцы подняты
    index_up = is_finger_up_simple(index_tip, index_pip, index_mcp)
    middle_up = is_finger_up_simple(middle_tip, middle_pip, middle_mcp)
    ring_up = is_finger_up_simple(ring_tip, ring_pip, ring_mcp)
    pinky_up = is_finger_up_simple(pinky_tip, pinky_pip, pinky_mcp)

    def is_thumb_up():
        """Проверка, поднят ли большой палец"""
        thumb_to_index = index_mcp.x - thumb_tip.x
        return thumb_to_index > 0.1

    thumb_up = is_thumb_up()

    # Подсчет поднятых пальцев
    fingers_up = [thumb_up, index_up, middle_up, ring_up, pinky_up]
    count_up = sum(fingers_up)

    # Координаты кончиков пальцев для рисования
    index_pos = (int(index_tip.x * w), int(index_tip.y * h))
    middle_pos = (int(middle_tip.x * w), int(middle_tip.y * h))

    # Распознавание жестов
    if count_up == 1 and index_up:
        send_expression("happy")
        return "DRAW", index_pos
    elif count_up == 5:
        send_expression("surprised")
        return "CLEAR", None
    elif count_up == 0:
        send_expression("neutral")
        return "FIST", None
    elif count_up == 2 and index_up and middle_up and not ring_up and not pinky_up:
        send_expression("winking")
        return "CHANGE_COLOR", middle_pos
    elif count_up == 3 and index_up and middle_up and ring_up and not pinky_up:
        return "INCREASE_THICKNESS", middle_pos
    elif count_up == 4 and index_up and middle_up and ring_up and not pinky_up:
        return "DECREASE_THICKNESS", middle_pos
    elif count_up == 2 and thumb_up and index_up:
        send_expression("laughing")
        return "HEART", index_pos

    return "NONE", None


def process_drawing(frame_queue, update_frame_callback):
    """Основная функция для обработки рисования жестами"""
    global drawing_canvas, last_point, finger_positions, brush_thickness
    global draw_color, color_index, gesture_history, stop_drawing_flag
    global drawing_active

    # Загружаем модель распознавания жестов
    model_path = download_model()
    if model_path is None:
        print("Не удалось загрузить модель распознавания жестов")
        drawing_active = False
        return

    # Настройка детектора жестов
    base_options = python.BaseOptions(model_asset_path=model_path)
    options = vision.HandLandmarkerOptions(
        base_options=base_options,
        num_hands=1,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5
    )

    try:
        detector = vision.HandLandmarker.create_from_options(options)
    except Exception as e:
        print(f"Ошибка создания детектора: {e}")
        drawing_active = False
        return

    # Инициализация переменных
    drawing_canvas = None
    last_point = None
    finger_positions.clear()
    gesture_history.clear()

    print("\n" + "=" * 50)
    print("РЕЖИМ РИСОВАНИЯ АКТИВИРОВАН!")
    print("=" * 50)
    print("\nИНСТРУКЦИЯ:")
    print("1. 1 палец (указательный) - РИСОВАТЬ")
    print("2. 2 пальца (V-знак) - СМЕНИТЬ ЦВЕТ")
    print("3. 3 пальца - УВЕЛИЧИТЬ ТОЛЩИНУ")
    print("4. 4 пальца - УМЕНЬШИТЬ ТОЛЩИНУ")
    print("5. 5 пальцев - ОЧИСТИТЬ ХОЛСТ")
    print("6. Кулак - ПАУЗА")
    print("7. Большой + указательный - ЖЕСТ 'СЕРДЦЕ'")
    print("\nНажмите Q для выхода")
    print("=" * 50)

    send_expression("happy")

    while drawing_active and not stop_drawing_flag:
        try:
            # Получаем кадр из очереди
            frame = None
            try:
                frame = frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if frame is None:
                continue

            if drawing_canvas is None:
                drawing_canvas = init_canvas(frame.shape)

            if drawing_canvas.shape[:2] != frame.shape[:2]:
                drawing_canvas = init_canvas(frame.shape)

            display_frame = frame.copy()
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            try:
                results = detector.detect(mp_image)
            except Exception as e:
                print(f"Ошибка распознавания жеста: {e}")
                continue

            current_gesture = "NONE"
            finger_pos = None

            if results.hand_landmarks:
                hand_landmarks = results.hand_landmarks[0]
                h, w = frame.shape[:2]

                current_gesture, finger_pos = detect_gesture(hand_landmarks, frame.shape)
                gesture_history.append(current_gesture)

                # Отрисовка ключевых точек руки
                for idx, landmark in enumerate(hand_landmarks):
                    x, y = int(landmark.x * w), int(landmark.y * h)
                    cv2.circle(display_frame, (x, y), 2, (0, 255, 255), -1)

                finger_tip_indices = [4, 8, 12, 16, 20]
                for tip_idx in finger_tip_indices:
                    x, y = int(hand_landmarks[tip_idx].x * w), int(hand_landmarks[tip_idx].y * h)
                    cv2.circle(display_frame, (x, y), 8, (0, 255, 255), -1)
                    cv2.circle(display_frame, (x, y), 10, (0, 255, 0), 2)

                if current_gesture == "DRAW" and finger_pos:
                    finger_positions.append(finger_pos)
                    if len(finger_positions) > 1:
                        avg_x = int(np.mean([p[0] for p in finger_positions]))
                        avg_y = int(np.mean([p[1] for p in finger_positions]))
                        smoothed_pos = (avg_x, avg_y)

                        if last_point is not None:
                            cv2.line(drawing_canvas, last_point, smoothed_pos, draw_color, brush_thickness)

                        cv2.circle(drawing_canvas, smoothed_pos, brush_thickness // 2, draw_color, -1)
                        last_point = smoothed_pos

                        cv2.circle(display_frame, smoothed_pos, brush_thickness + 5, draw_color, 2)
                        cv2.circle(display_frame, smoothed_pos, brush_thickness + 2, (255, 255, 255), 2)

                elif current_gesture == "CLEAR":
                    if list(gesture_history).count("CLEAR") > 5:
                        drawing_canvas = init_canvas(frame.shape)
                        last_point = None
                        finger_positions.clear()
                        gesture_history.clear()
                        print("Холст очищен!")

                elif current_gesture == "FIST":
                    last_point = None
                    finger_positions.clear()

                elif current_gesture == "CHANGE_COLOR":
                    if list(gesture_history).count("CHANGE_COLOR") > 5:
                        color_index = (color_index + 1) % len(colors)
                        draw_color = colors[color_index]
                        gesture_history.clear()
                        print(f"Цвет изменен на: {color_names[color_index]}")

                elif current_gesture == "INCREASE_THICKNESS":
                    if list(gesture_history).count("INCREASE_THICKNESS") > 5:
                        brush_thickness = min(20, brush_thickness + 2)
                        gesture_history.clear()
                        print(f"Толщина кисти увеличена: {brush_thickness}")

                elif current_gesture == "DECREASE_THICKNESS":
                    if list(gesture_history).count("DECREASE_THICKNESS") > 5:
                        brush_thickness = max(2, brush_thickness - 2)
                        gesture_history.clear()
                        print(f"Толщина кисти уменьшена: {brush_thickness}")

                elif current_gesture == "HEART":
                    if list(gesture_history).count("HEART") > 5 and finger_pos:
                        x, y = finger_pos
                        radius = brush_thickness * 2
                        cv2.circle(drawing_canvas, (x - radius // 2, y - radius // 2), radius, draw_color, -1)
                        cv2.circle(drawing_canvas, (x + radius // 2, y - radius // 2), radius, draw_color, -1)
                        pts = np.array([[x - radius, y - radius // 3],
                                        [x + radius, y - radius // 3],
                                        [x, y + radius]], np.int32)
                        cv2.fillPoly(drawing_canvas, [pts], draw_color)
                        gesture_history.clear()
                        print("Нарисовано сердце!")

            else:
                last_point = None
                finger_positions.clear()

            display_frame = cv2.addWeighted(display_frame, 1.0, drawing_canvas, 1.0, 0)

            # Отображение информации
            info_y = 30
            cv2.putText(display_frame, "РЕЖИМ РИСОВАНИЯ ИЗБУШКИ",
                        (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            info_y += 30
            cv2.putText(display_frame, f"ЦВЕТ: {color_names[color_index]}",
                        (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)

            info_y += 25
            cv2.putText(display_frame, f"ТОЛЩИНА: {brush_thickness}",
                        (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            info_y += 25
            cv2.putText(display_frame, f"ЖЕСТ: {current_gesture}",
                        (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

            # Обновляем кадр для трансляции
            if update_frame_callback:
                update_frame_callback(display_frame.copy())

            cv2.imshow('Избушка - Рисование жестами', display_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                print("Выход по клавише Q")
                break

        except Exception as e:
            print(f"Ошибка в обработке рисования: {e}")
            time.sleep(0.1)

    # Очистка ресурсов
    print("Завершение режима рисования...")
    try:
        detector.close()
    except:
        pass

    try:
        cv2.destroyAllWindows()
        cv2.waitKey(1)
    except:
        pass

    drawing_active = False
    stop_drawing_flag = False
    send_expression("neutral")
    print("Режим рисования завершен.")


def start_drawing_mode(frame_queue_param=None, update_callback=None):
    """Запуск режима рисования в отдельном потоке"""
    global drawing_thread, drawing_active, stop_drawing_flag
    global frame_queue_ref, update_frame_callback

    if drawing_active:
        print("Режим рисования уже активен!")
        return False

    stop_drawing_flag = False
    drawing_active = True

    # Сохраняем ссылки на очередь и callback
    frame_queue_ref = frame_queue_param
    update_frame_callback = update_callback

    drawing_thread = threading.Thread(
        target=process_drawing,
        args=(frame_queue_ref, update_frame_callback),
        daemon=True
    )
    drawing_thread.start()

    print("Запуск потока рисования...")
    return True


def stop_drawing_mode():
    """Остановка режима рисования"""
    global drawing_active, stop_drawing_flag

    if not drawing_active:
        print("Режим рисования не был активен")
        return False

    print("Остановка режима рисования...")
    stop_drawing_flag = True
    drawing_active = False

    if drawing_thread and drawing_thread.is_alive():
        drawing_thread.join(timeout=3.0)
        if drawing_thread.is_alive():
            print("Предупреждение: поток рисования не завершился вовремя")

    print("Режим рисования остановлен")
    return True


def is_drawing_active():
    """Проверка активности режима рисования"""
    global drawing_active
    return drawing_active


def get_drawing_status():
    """Получение статуса рисования в виде словаря"""
    global drawing_active, color_index, brush_thickness, draw_color
    return {
        "active": drawing_active,
        "color": color_names[color_index] if color_index < len(color_names) else "ЗЕЛЕНЫЙ",
        "color_rgb": list(draw_color) if draw_color else [0, 255, 0],
        "thickness": brush_thickness
    }


if __name__ == "__main__":
    print("Тестирование модуля рисования жестами")
    print("1. Запуск теста камеры")
    print("2. Запуск режима рисования (тест без основного сервера)")
    print("3. Выход")

    choice = input("Выберите опцию (1-3): ")

    if choice == "1":
        for i in range(3):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"Камера {i} доступна")
                ret, frame = cap.read()
                if ret:
                    cv2.imshow(f"Test Camera {i}", frame)
                    cv2.waitKey(1000)
                cap.release()
            else:
                print(f"Камера {i} недоступна")
        cv2.destroyAllWindows()
    elif choice == "2":
        print("Для работы с рисованием требуется запущенный основной сервер.")
        print("Запустите main.py, а затем используйте веб-интерфейс.")
    else:
        print("Выход")