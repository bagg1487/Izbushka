import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import urllib.request
import os
import numpy as np
import collections

drawing_canvas = None
last_point = None
draw_color = (0, 255, 0)  # BGR, не RGB
brush_thickness = 8

finger_positions = collections.deque(maxlen=5)

def download_model():
    model_url = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
    model_path = "hand_landmarker.task"

    if not os.path.exists(model_path):
        print("Скачивание модели...")
        try:
            urllib.request.urlretrieve(model_url, model_path)
            print("Модель скачана!")
        except Exception as e:
            print(f"Ошибка при скачивании модели: {e}")
            return None
    return model_path


def init_canvas(frame_shape):
    """Создает прозрачный холст для рисования"""
    return np.zeros((frame_shape[0], frame_shape[1], 3), dtype=np.uint8)


def detect_gesture(hand_landmarks, frame_shape):
    h, w = frame_shape[:2]

    thumb_tip = hand_landmarks[4]
    thumb_ip = hand_landmarks[3]
    thumb_mcp = hand_landmarks[2]

    index_tip = hand_landmarks[8]
    index_dip = hand_landmarks[7]
    index_pip = hand_landmarks[6]
    index_mcp = hand_landmarks[5]

    middle_tip = hand_landmarks[12]
    middle_dip = hand_landmarks[11]
    middle_pip = hand_landmarks[10]
    middle_mcp = hand_landmarks[9]

    ring_tip = hand_landmarks[16]
    ring_dip = hand_landmarks[15]
    ring_pip = hand_landmarks[14]
    ring_mcp = hand_landmarks[13]

    pinky_tip = hand_landmarks[20]
    pinky_dip = hand_landmarks[19]
    pinky_pip = hand_landmarks[18]
    pinky_mcp = hand_landmarks[17]

    def is_finger_up_simple(tip, pip, mcp):
        tip_to_pip = pip.y - tip.y
        pip_to_mcp = mcp.y - pip.y
        return tip_to_pip > 0.05 and pip_to_mcp > 0.03

    index_up = is_finger_up_simple(index_tip, index_pip, index_mcp)
    middle_up = is_finger_up_simple(middle_tip, middle_pip, middle_mcp)
    ring_up = is_finger_up_simple(ring_tip, ring_pip, ring_mcp)
    pinky_up = is_finger_up_simple(pinky_tip, pinky_pip, pinky_mcp)

    def is_thumb_up():
        thumb_to_index = index_mcp.x - thumb_tip.x
        return thumb_to_index > 0.1

    thumb_up = is_thumb_up()
    fingers_up = [thumb_up, index_up, middle_up, ring_up, pinky_up]
    count_up = sum(fingers_up)

    index_pos = (int(index_tip.x * w), int(index_tip.y * h))

    # Жест рисования - только указательный палец
    if count_up == 1 and index_up and not middle_up and not ring_up and not pinky_up:
        if (index_tip.y < middle_tip.y - 0.1 and
                index_tip.y < ring_tip.y - 0.1 and
                index_tip.y < pinky_tip.y - 0.1):
            return "DRAW", index_pos

    # Жест очистки - все 5 пальцев
    elif count_up >= 4:
        if index_up and middle_up and ring_up and pinky_up:
            return "CLEAR", None

    elif count_up == 0:
        all_fingers_down = not index_up and not middle_up and not ring_up and not pinky_up
        if all_fingers_down:
            return "FIST", None

    return "NONE", None


def process_orange_pi_camera():
    global drawing_canvas, last_point, finger_positions, brush_thickness, draw_color

    # Загрузка модели
    model_path = download_model()
    if model_path is None:
        print("Не удалось загрузить модель. Проверьте подключение к интернету.")
        return

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
        print(f"Ошибка при создании детектора: {e}")
        return

    # Настройки камеры для Orange Pi
    camera_index = 0


    # Пробуем разные индексы камеры
    for i in range(4):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Камера найдена на индексе {i}")
            break
        cap.release()

    # cap = cv2.VideoCapture(3) # 3 индекс для камеры

    if not cap.isOpened():
        print("Не удалось открыть камеру. Проверьте подключение USB-камеры.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Не удалось получить кадр с камеры")
            import time
            time.sleep(0.1)
            continue

        frame = cv2.flip(frame, 1)

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
            print(f"Ошибка при детекции: {e}")
            continue

        current_gesture = "NONE"
        finger_pos = None

        if results.hand_landmarks:
            hand_landmarks = results.hand_landmarks[0]
            h, w = frame.shape[:2]

            current_gesture, finger_pos = detect_gesture(hand_landmarks, frame.shape)

            # Отображаем только кончик указательного пальца
            index_x, index_y = int(hand_landmarks[8].x * w), int(hand_landmarks[8].y * h)
            cv2.circle(display_frame, (index_x, index_y), 8, (0, 255, 0), -1)

            # Обработка жестов
            if current_gesture == "DRAW" and finger_pos:
                finger_positions.append(finger_pos)
                if len(finger_positions) > 1:
                    avg_x = int(np.mean([p[0] for p in finger_positions]))
                    avg_y = int(np.mean([p[1] for p in finger_positions]))
                    smoothed_pos = (avg_x, avg_y)

                    if last_point is not None:
                        cv2.line(drawing_canvas, last_point, smoothed_pos, draw_color, brush_thickness)
                    last_point = smoothed_pos

                else:
                    last_point = finger_pos

            elif current_gesture == "CLEAR":
                drawing_canvas = init_canvas(frame.shape)
                last_point = None
                finger_positions.clear()
                # Показываем надпись только при очистке
                cv2.putText(display_frame, "CLEAR", (w // 2 - 60, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

            elif current_gesture == "FIST":
                last_point = None
                finger_positions.clear()

        else:
            last_point = None
            finger_positions.clear()

        # Наложение холста на кадр
        display_frame = cv2.addWeighted(display_frame, 1.0, drawing_canvas, 1.0, 0)

        # Показываем минимальную информацию
        if current_gesture == "DRAW":
            cv2.putText(display_frame, "DRAW", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Показываем только одну строку инструкций внизу
        cv2.putText(display_frame, "finger - DRAW | palm - CLEAR",
                   (10, display_frame.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow('Управление жестами', display_frame)


        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            drawing_canvas = init_canvas(frame.shape)
            last_point = None
            finger_positions.clear()
        elif key == ord('s'):
            if drawing_canvas is not None and np.any(drawing_canvas != 0):
                cv2.imwrite("drawing_saved.png", drawing_canvas)
                print("Рисунок сохранен как 'drawing_saved.png'")
        elif key == ord('+'):
            brush_thickness = min(20, brush_thickness + 2)
        elif key == ord('-'):
            brush_thickness = max(2, brush_thickness - 2)
        elif key == ord('r'):
            draw_color = (0, 0, 255)
        elif key == ord('g'):
            draw_color = (0, 255, 0)
        elif key == ord('b'):
            draw_color = (255, 0, 0)
        elif key == ord('w'):
            draw_color = (255, 255, 255)

    cap.release()
    cv2.destroyAllWindows()
    detector.close()


if __name__ == "__main__":
    process_orange_pi_camera()