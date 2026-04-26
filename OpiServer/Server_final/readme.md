# Избушка - проект по созданию и проектированию избушки

## Что это за проект

"Избушка" — это десктопное приложение на PyQt5 для управления роботом-ассистентом. 

Программа обеспечивает:

- **Голосовое управление** (распознавание и синтез речи)
- **Управление роботом** через UART (движение, сервоприводы, LED-подсветка)
- **Воспроизведение музыки** (MP3, WAV)
- **Интернет-радио**
- **Проигрывание видео** (MP4, AVI)
- **Распознавание лиц** (OpenCV + LBPH)
- **Распознавание жестов рук** (MediaPipe)
- **ИИ-ассистент** (Gemini AI)
- **Прогноз погоды**
- **База данных пользователей** (SQLite)
- **Игра "Камень-ножницы-бумага"** (голосом и жестами)

## Структура проекта

```
Server_final/
├── main.py # Точка входа, GUI на PyQt5
├── Server.py # TCP-сервер для связи с клиентами
├── Service.py # Основные сервисы (музыка, радио, видео)
├── Speaker.py # Синтез речи (gTTS + pygame)
├── Recognizer.py # Распознавание речи (Vosk)
├── GeminiAssistant.py # ИИ-ассистент Gemini
├── FaceAction.py # Распознавание лиц
├── Face.py # Обучение модели лиц
├── HandAndFingerDetectionModule.py # Распознавание жестов
├── RockPapperScrissor.py # Игра «Камень-ножницы-бумага»
├── ServoControl.py # Калибровка сервоприводов
├── UART.py # Связь с роботом по Serial
├── MusicPlayer.py # Музыкальный плеер
├── VideoPlayer.py # Видеоплеер
├── RadioPlayer.py # Интернет-радио (VLC)
├── PlayVideoViaBrowser.py # Открытие видео в браузере
├── KeyPhraseAnalizer.py # Анализ ключевых фраз (sklearn)
├── DatabaseManager.py # Работа с SQLite
├── Command.py # Список команд для робота
├── server_interface.py # GUI-интерфейс (сгенерирован)
├── server_interface.ui # Исходник интерфейса Qt Designer
├── requirements.txt # Зависимости Python
├── .env # API-ключи (GEMINI_API_KEY)
├── voice1/ # Временные аудиофайлы (создаётся автоматически)
├── all_animation/ # GIF-анимации для лица
├── PRS/ # Дополнительные анимации
├── player_button/ # Иконки для плееров
├── audio/ # Аудиофайлы (MP3, WAV)
└── dataset/ # Фото для обучения распознавания лиц
```

## Технологии

| Компонент | Технология |
|-----------|-------------|
| Фреймворк | PyQt5 |
| Голосовой ввод | Vosk (офлайн) / SpeechRecognition (Google) |
| Голосовой вывод | gTTS + pygame |
| ИИ-ассистент | Google Gemini API |
| Распознавание лиц | OpenCV + LBPH |
| Распознавание жестов | MediaPipe |
| Погода | Open-Meteo API |
| База данных | SQLite |
| Связь с роботом | UART (Serial) |
| Музыка/Видео | QMediaPlayer, VLC |

## Установка и запуск


```
git clone https://github.com/bagg1487/Izbushka/tree/final-izba
cd OpiServer/Server_final
python3 -m venv venv
source venv/bin/activate  # Linux/Mac
# или
venv\Scripts\activate   # Windows
pip install -r requirements.txt
./install_all.sh
```

## Настройка API ключей

Создайте файл .env вставьте туда свой ключ

GEMINI_API_KEY=

