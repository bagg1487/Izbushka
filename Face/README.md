# Избушка — голосовой ассистент с управлением жестами и эмоциями

## Проект который умеет:
- **Распознавать голосовые команды** и отвечать голосом
- **Менять выражение лица** (радость, злость, удивление и др.)
- **Запускать игровой автомат** (казино)
- **Рисовать жестами рук** через веб-камеру
- **Сообщать погоду** в Новосибирске
- **Отвечать на вопросы** через Wikipedia и GPT
- **Принимать данные с датчиков** (STM32)

## Установка и запуск

```bash
sudo apt update
sudo apt install python3 python3-pip python3-venv -y
python3 -m venv izba-env
source izba-env/bin/activate
pip install -r requirements.txt
```

## Endpoint

| Endpoint | Метод | Описание |
|----------|-------|----------|
| `/` | GET | Анимированный персонаж с эмоциями |
| `/control` | GET | Панель управления кнопками |
| `/casino` | GET | Игровой автомат |
| `/drawing` | GET | Рисование жестами через камеру |
| `/diagnostics` | GET | Графики датчиков (STM32) |
| `/video_feed` | GET | Потоковое видео с камеры (MJPEG) |

## Фото

<p align="center">
  <img src="https://github.com/user-attachments/assets/3c482afa-85b6-43f3-8f78-a5f4e65c4abd" alt="Image" width="400">
  <br>
  <em>/</em>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/58699126-a8af-47ec-938c-af038839a456" alt="Image" width="400">
  <br>
  <em>/casino</em>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/0fe944cc-da7a-4362-84da-367afd68fbe9" alt="Image" width="400">
  <br>
  <em>/drawing</em>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/0fcaf8ec-c260-4179-a484-647cf1e97d47" alt="Image" width="400">
  <br>
  <em>/diagnostics</em>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/c1cb1459-bbc0-4ae2-aa8f-689f1f6b8fe7" alt="Image" width="400">
  <br>
  <em>/video_feed</em>
</p>
