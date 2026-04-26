
import cv2
import mediapipe as freedomtech
#from gtts import gTTS


class HandAndFingerDetectionModule:
    def __init__(self):
        # Импорт модулей для рисования и распознавания рук
        self.drawingModule = freedomtech.solutions.drawing_utils
        self.handsModule = freedomtech.solutions.hands

        # Создание экземпляра класса Hands
        self.mod = self.handsModule.Hands()

    def findpostion(self, frame1):
        """Поиск ключевых точек руки на кадре
        Возвращает: список ключевых точек руки(номер, координата x, координата y)
        """
        # Создание пустого списка
        list = []

        # Получение высоты и ширины кадра
        height, width = frame1.shape[:2]

        # Обработка кадра и получение результатов
        results = self.mod.process(cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB))

        # Проверка наличия обнаруженных рук
        if results.multi_hand_landmarks != None:
            # Проход по всем обнаруженным рукам
            for handLandmarks in results.multi_hand_landmarks:
                # Рисование landmarks для каждой руки на кадре
                self.drawingModule.draw_landmarks(frame1, handLandmarks, self.handsModule.HAND_CONNECTIONS)

                # Создание временного списка для landmarks каждой руки
                hand_list = []

                # Проход по landmarks каждой руки и добавление их координат в список
                for id, pt in enumerate(handLandmarks.landmark):
                    x = int(pt.x * width)
                    y = int(pt.y * height)
                    hand_list.append([id, x, y])

                # Добавление временного списка landmarks в общий список
                list.append(hand_list)

        # Возврат списка landmarks
        return list

    def findnameoflandmark(self,frame1):
        """Поиск пальцев на кадре.
        Возвращает: список имен распознанных пальцев
        """
        # Создание пустого списка
        list = []

        # Обработка кадра и получение результатов
        results = self.mod.process(cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB))

        # Проверка наличия обнаруженных рук
        if results.multi_hand_landmarks != None:
            # Проход по всем обнаруженным рукам
            for handLandmarks in results.multi_hand_landmarks:
                # Проход по каждой точке руки и добавление ее названия в список
                for point in self.handsModule.HandLandmark:
                    list.append(
                        str(point).replace("< ", "").replace("HandLandmark.", "").replace("_", " ").replace("[]", ""))

        # Возврат списка с названиями точек рук
        return list
