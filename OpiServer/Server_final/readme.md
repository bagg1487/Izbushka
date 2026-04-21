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


```bash
git clone https://github.com/bagg1487/Izbushka/tree/final-izba
cd OpiServer/Server_final
python3 -m venv venv
source venv/bin/activate  # Linux/Mac
# или
venv\Scripts\activate   # Windows
pip install -r requirements.txt
chmod +x install_all.sh
./install_all.sh
```

## Настройка API ключей

Создайте файл .env вставьте туда свой ключ

GEMINI_API_KEY=

