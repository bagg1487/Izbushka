import os

import numpy as np
from sklearn.calibration import CalibratedClassifierCV
from sklearn.svm import LinearSVC
from sklearn.pipeline import make_pipeline
from sklearn.feature_extraction.text import TfidfVectorizer


class KeyPhraseAnalizer:
    """Класс анализа и поиска ключевых фраз"""
    def __init__(self):
        self.key_phrase = []
        self.classifier = make_pipeline(TfidfVectorizer(), CalibratedClassifierCV(LinearSVC(dual=True)))
        # Открываем файл на чтение
        parent_dir = os.path.dirname(os.path.abspath(__file__))
        with open(parent_dir + '/key_phrase.txt', 'r', encoding='utf-8') as file:
            # Читаем содержимое файла и разделяем его на строки
            lines = file.read().splitlines()

        # Преобразуем строки в кортежи с помощью eval()
        for s in lines:
            self.key_phrase.append(tuple(item for item in s.split(',')))
        train_texts, train_labels = zip(*self.key_phrase)
        self.classifier.fit(train_texts, train_labels)

    def analyzer(self, phrase):
        """Анализирует текст на совпадение с ключевыми фразами
        Аргументы: phrase- входящий текст
        Возвращает: ключевую фразу с большим совпадением"""
        list_phrase = [phrase]
        # Предсказание вероятностей принадлежности классам
        predicted_probs = self.classifier.predict_proba(list_phrase)
        # Получение названий классов из классификатора
        class_names = self.classifier.classes_
        # Получаем индексы классов с самыми большими вероятностями
        top_classes = np.argsort(predicted_probs, axis=1)[:, -2:]
        # Выводим результаты
        for i, data in enumerate(list_phrase):
            class1, class2 = top_classes[i]
            # Функция для вывода двух списков поэлементно с использованием лямбда-функции
            print_list = lambda x, y: print("\n".join("{} {}".format(a, b) for a, b in zip(x, y)))

            # Вывод списков
            print_list(predicted_probs[i], class_names)
            print(
                f"Data: {data}, Top Classes: {class_names[class1]}, {class_names[class2]}")
            # Если вероятности обоих классов выше порога, возвращаем пустой набор
            if predicted_probs[0, class1] < 0.1 and predicted_probs[0, class2] < 0.1:
                return {"none", "none"}
            elif predicted_probs[0, class1] > 0.1 and predicted_probs[0, class2] < 0.1:
                return {class_names[class1].strip(), "none"}
            elif predicted_probs[0, class1] > 0.1 and predicted_probs[0, class2] > 0.1:
                return {class_names[class1].strip(), class_names[class2].strip()}
            elif predicted_probs[0, class1] < 0.1 and predicted_probs[0, class2] > 0.1:
                return {"none", class_names[class2].strip()}

    def cut_stop_word(self, phrase):
        """"Вырезает стоп слова из фразы"""
        # Токенизируем фразу
        tokens = phrase.split()
        # Открываем файл на чтение
        parent_dir = os.path.dirname(os.path.abspath(__file__))
        with open(parent_dir + '/russian', 'r', encoding='utf-8') as file:
            # Читаем содержимое файла и разделяем его на слова
            stop_words = file.read().split()

        tokens = [word for word in tokens if word.lower() not in stop_words]

        # Склеиваем слова обратно в фразу
        phrase_nom = ' '.join(tokens).lower()
        return phrase_nom
