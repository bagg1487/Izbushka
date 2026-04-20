import numpy as np
from sklearn.calibration import CalibratedClassifierCV
from sklearn.svm import LinearSVC
from sklearn.pipeline import make_pipeline
from sklearn.feature_extraction.text import TfidfVectorizer

# Создаем модель LinearSVC
classifier = make_pipeline(TfidfVectorizer(), CalibratedClassifierCV(LinearSVC()))

# Тренируем модель на тренировочных данных
training_data = [
    ("открой гугл", "гугл"),
    ("открой яндекс", "яндекс"),
    ("открой youtube", "youtube"),
    ("включи youtube", "youtube"),
    ("запусти видео в youtube", "youtube"),
    ("видео из youtube", "youtube"),
    ("найди информацию о Python", "гугл"),
    ("покажи мне новости", "яндекс"),
    ("проиграй песню", "найди видео"),
    ("найди рецепт картошки", "гугл"),
    ("покажи мне картинки", "яндекс"),
    ("проиграй видео", "найди видео"),
    ("найди статью о Python", "гугл"),
    ("покажи мне книги", "яндекс"),
    ("проиграй фильм", "найди видео"),
    ("включи видео", "найди видео"),
    ("запусти видео", "найди видео"),
    ("включи видео с ", "найди видео"),
    ("включи например видео", "найди видео"),
    ("включи видео с ", "найди видео"),
]
train_texts, train_labels = zip(*training_data)
classifier.fit(train_texts, train_labels)

# Тестируем модель на новых данных
test_data = [
    "найди классное видео в гугл"
]
# predicted_probs = classifier.decision_function(test_data)
#
# # Получаем прогнозы классов для всех данных
predicted_classes = classifier.predict(test_data)
print(predicted_classes)
predicted_probs = classifier.predict_proba(test_data)

# Получаем индексы классов с самыми большими вероятностями
top_classes = np.argsort(predicted_probs, axis=1)[:, -2:]

# Получаем названия меток классов
class_names = classifier.classes_

# Выводим результаты
for i, data in enumerate(test_data):
    class1, class2 = top_classes[i]
    print(f"Data: {data}, Predicted Probs: {predicted_probs[i]}, Top Classes: {class_names[class1]}, {class_names[class2]}")
# # Получаем тестовые слова
# test_words = classifier.named_steps['tfidfvectorizer'].vocabulary_
# test_words2 = classifier.named_steps['tfidfvectorizer'].get_feature_names_out()
#
# print(test_words)
# print(test_words2)
# predicted_labels = classifier.predict(test_data)
# print(predicted_labels)
#
# predicted_probs = classifier.decision_function(test_data)
#
# # Выбираем второй приоритет
# second_priority_labels = [np.argsort(probs)[-2] for probs in predicted_probs]
#
# # Выводим метки по индексам
# labels = list(set(train_labels))
# for index in second_priority_labels:
#     print("Метка:", labels[index])

# from pymorphy2 import MorphAnalyzer
# from nltk.corpus import stopwords
# import nltk
# nltk.download('stopwords')
#
# # Инициализируем морфологический анализатор
# morph = MorphAnalyzer()
#
# # Пример фразы в родительном падеже
# phrase = 'Избушка найди видео с котиками на ютуб '
#
# # Токенизируем фразу
# tokens = phrase.split()
# # Открываем файл на чтение
#
# with open('russian', 'r', encoding='utf-8') as file:
#     # Читаем содержимое файла и разделяем его на слова
#     stop_words = file.read().split()
#
# tokens = [word for word in tokens if word.lower() not in stop_words]
#
# # Приводим каждое слово к именительному падежу
# #tokens_nom = [morph.parse(word)[0].inflect({'nomn'}).word for word in tokens]
#
# # Склеиваем слова обратно в фразу
# phrase_nom = ' '.join(tokens).lower()
#
# search_query=phrase_nom.replace('найди', '').replace('избушка', '').replace('видео', '')
# print("Фраза для поиска:", search_query)
