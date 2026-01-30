
import cv2
import mediapipe
import time
import random

from Speaker import *


class RockPapperScrissor:
    def __init__(self):
        self.frame = None
        self.start_trig = False
        self.anim_trig = False
        self.drawingModule = mediapipe.solutions.drawing_utils
        self.hands_module = mediapipe.solutions.hands
        self.speach_rec = Speaker()
        self.dialog_trig = True
        self.first_dialog_trig = True
        self.exit_trig = False
        self.play_game_trig = False
        self.selected = None
        self.won = None

    def calculate_game_state(self, move):
        """Расчет результата"""
        # Определение возможных ходов и победителей
        moves = ["Rock", "Paper", "Scissors"]
        wins = {"Rock": "Scissors", "Paper": "Rock", "Scissors": "Paper"}

        # Выбор компьютера случайным образом
        self.selected = random.randint(0, 2)
        print("Computer played " + moves[self.selected])

        # Проверка результатов
        if moves[self.selected] == move:
            return 0, moves[self.selected]

        if wins[move] == moves[self.selected]:
            return 1, moves[self.selected]

        return -1, moves[self.selected]

    def get_finger_status(self, hands_module, hand_landmarks, finger_name):
        """Расчет положения пальцев, кроме Большого"""
        # Сопоставление идентификаторов пальцев с их значениями
        finger_id_map = {'INDEX': 8, 'MIDDLE': 12, 'RING': 16, 'PINKY': 20}

        # Получение координат вершин пальцев
        finger_tip_y = hand_landmarks.landmark[finger_id_map[finger_name]].y
        finger_dip_y = hand_landmarks.landmark[finger_id_map[finger_name] - 1].y
        finger_mcp_y = hand_landmarks.landmark[finger_id_map[finger_name] - 2].y

        # Возврат результата сравнения координат вершин пальцев
        return finger_tip_y < finger_mcp_y

    def get_thumb_status(self, hands_module, hand_landmarks):
        """Расчет положения Большого пальца"""
        # Получение координат вершин большого пальца
        thumb_tip_x = hand_landmarks.landmark[hands_module.HandLandmark.THUMB_TIP].x
        thumb_mcp_x = hand_landmarks.landmark[hands_module.HandLandmark.THUMB_TIP - 2].x
        thumb_ip_x = hand_landmarks.landmark[hands_module.HandLandmark.THUMB_TIP - 1].x

        # Возврат результата сравнения координат вершин большого пальца
        return thumb_tip_x > thumb_ip_x > thumb_mcp_x

    def start_game_RPS(self, rec):
        # Инициализация переменных
        start_time = 0.0
        timer_started = True
        time_left_now = 3
        hold_for_play = False
        draw_timer = 0.0
        game_over_text = ""
        computer_played = ""

        # Обработка изображения рук
        with self.hands_module.Hands(static_image_mode=False, min_detection_confidence=0.7,
                                     min_tracking_confidence=0.4, max_num_hands=2) as hands:
            while True:
                # Обновление времени таймера, если он запущен
                if timer_started:
                    now_time = time.time()
                    time_elapsed = now_time - start_time
                    if time_elapsed >= 1:
                        time_left_now -= 1
                        start_time = now_time
                        if time_left_now <= 0:
                            hold_for_play = True
                            timer_started = False

                # Обработка результатов обнаружения рук
                results = hands.process(cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB))

                # Определение движения руки
                move = "UNKNOWN"
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        # Отрисовка маркеров, если игра еще не началась или прошло менее 2 секунд после последнего хода
                        if hold_for_play or time.time() - draw_timer <= 2:
                            self.drawingModule.draw_landmarks(self.frame, hand_landmarks,
                                                              self.hands_module.HAND_CONNECTIONS)

                        # Получение состояния пальцев
                        current_state = ""
                        thumb_status = self.get_thumb_status(self.hands_module, hand_landmarks)
                        current_state += "1" if thumb_status else "0"

                        index_status = self.get_finger_status(self.hands_module, hand_landmarks, 'INDEX')
                        current_state += "1" if index_status else "0"

                        middle_status = self.get_finger_status(self.hands_module, hand_landmarks, 'MIDDLE')
                        current_state += "1" if middle_status else "0"

                        ring_status = self.get_finger_status(self.hands_module, hand_landmarks, 'RING')
                        current_state += "1" if ring_status else "0"

                        pinky_status = self.get_finger_status(self.hands_module, hand_landmarks, 'PINKY')
                        current_state += "1" if pinky_status else "0"
                        print(current_state)
                        move_speak = "UNKNOWN"
                        # Определение движения руки на основе состояния пальцев
                        if current_state == "00000":
                            move = "Rock"
                            move_speak = "камень"
                        elif current_state == "11111":
                            move = "Paper"
                            move_speak = "бумагу"
                        elif current_state == "01100":
                            move = "Scissors"
                            move_speak = "ножницы"
                        else:
                            move = "UNKNOWN"

                    # Обработка ходов игрока и вывод результатов игры
                    if hold_for_play and move != "UNKNOWN":
                        # Отключение анимации после хода игрока
                        self.anim_trig = False
                        draw_timer = time.time()
                        print("Player played " + move)

                        # Вычисление результата игры
                        self.won, cmp_move = self.calculate_game_state(move)
                        computer_played = "You: " + move + " | Computer: " + cmp_move

                        # Воспроизведение речевых сообщений в зависимости от результата игры
                        if self.won == 1:
                            game_over_text = "You've won!"
                            self.speach_rec.speak("Вы победили")
                        elif self.won == -1:
                            game_over_text = "You've lost!"
                            self.speach_rec.speak("Я выйграла")
                        else:
                            game_over_text = "It's a draw!"
                            self.speach_rec.speak("У нас ничья")

                        self.dialog_trig = True
                        hold_for_play = False

                # Обработка диалога с пользователем
                if self.dialog_trig:
                    if self.first_dialog_trig:
                        self.speach_rec.speak(
                            "По окончанию отсчета покажите жест. Если вы готовы скажите Да")
                        self.first_dialog_trig = False
                    else:
                        self.speach_rec.speak("сыграем еще раз?")

                    # Обработка ответа пользователя
                    while True:
                        rec.listening()
                        query = rec.get_full_phrase()
                        query = query.lower()
                        if 'да' in query:
                            self.start_trig = True
                            self.dialog_trig = False

                            break
                        elif 'нет' in query:
                            self.dialog_trig = False
                            self.exit_trig = True
                            self.play_game_trig = False
                            break

                # Обновление текста на экране в зависимости от состояния игры
                label_text = "PRESS SPACE TO START!"
                if hold_for_play:
                    label_text = "PLAY NOW!"
                elif timer_started:
                    label_text = "PLAY STARTS IN " + str(time_left_now)
                    self.speach_rec.speak(str(time_left_now))

                # Обработка начала игры после нажатия пробела
                if self.start_trig:
                    print("pressed space")
                    start_time = time.time()
                    timer_started = True
                    self.anim_trig = True
                    time_left_now = 3
                    self.start_trig = False

                # Обработка выхода из игры
                if self.exit_trig:
                    self.exit_trig = False
                    self.first_dialog_trig = True
                    self.dialog_trig = True
                    self.start_trig = False
                    break


if __name__ == "__main__":
    pass
