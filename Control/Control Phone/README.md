# Приложение для управления избушкой (Android Studio)
---

*SDK* должен быть >21
Середина 29
А лучше 34 (Android 14)

## Полезные ссылки: 
- [github.io](https://telecomdep.github.io/notes/Android/sockets/06_sockets_zmq.html)(Работа с сокетами/сайт нашего преподавателя)
- [KursHub.ru](https://kurshub.ru/journal/blog/chto-takoe-soket/)
- [Metanit](https://metanit.com/python/network/1.2.php)

---

Перед созданием frontend необходимо ознакомиться с [приложением](https://github.com/bagg1487/Izbushka/tree/main/Control/Control%20Phone/Organizer)

Перед созданием backend необходимо ознакомиться с [сервером](https://github.com/bagg1487/Izbushka/blob/main/OpiServer/Server_final3/Server_final/Server.py), его функционалом

---
Пример интерфейса

| | | |
|:---:|:---:|:---:|
| ![Фото 1](https://github.com/Darkness1853/Pictures/blob/main/p1/photo_2026-03-02_20-47-48.jpg) | ![Фото 2](https://github.com/Darkness1853/Pictures/blob/main/p1/photo_2026-03-02_20-47-47.jpg) | ![Фото 3](https://github.com/Darkness1853/Pictures/blob/main/p1/photo_2026-03-02_20-47-46.jpg) |
| *MainActivity* | *FunctionsActivity* | *InfoActivity* |

| | |
|:---:|:---:|
| ![Фото 1](https://github.com/Darkness1853/Pictures/blob/main/p1/photo_2026-03-06_15-03-28.jpg) | ![Фото 2](https://github.com/Darkness1853/Pictures/blob/main/p1/photo_2026-03-06_15-03-23.jpg) |
| *CalibrationActivity* | *LivestreamActivity* |

Синтаксис [Kotlin](https://telecomdep.github.io/notes/Android/Kotlin/kotlin.html)

Работа с Android Studio

Создание первого [проекта](https://telecomdep.github.io/notes/Android/02_first_app.html)

Разработка [интерфейса](https://telecomdep.github.io/notes/Android/app_activity.html)

---

# Backend

Для перемещение между Activity используйте Intent
```kotlin
findViewById<Button>(R.id.GoCalculator).setOnClickListener({
            val calc_Intent = Intent(this, CalculatorActivity::class.java)
            startActivity(calc_Intent)
        });    
```
---

# В планах

Необходимо в приложении открыть порта: 

Видеопоток (порт 8001)
Для получения кадров в формате jpeg и отображения его на экране



Поток команд (5001)
Для отправки команд на сервер и приема ответов от сервера

Необходимо создать класс команд

```kotlin
object Commands {
    // Движение
    const val MOVE_FORWARD = "CMD_MOVE_FORWARD"
    const val MOVE_BACKWARD = "CMD_MOVE_BACKWARD"
    const val MOVE_LEFT = "CMD_MOVE_LEFT"
    const val MOVE_RIGHT = "CMD_MOVE_RIGHT"
    const val MOVE_STOP = "CMD_MOVE_STOP"
    
    // Управление руками
    const val HANDS_UP = "CMD_HANDS_UP"
    const val CLENCH_LEFT = "CMD_CLENCH_LEFT"
    const val CLENCH_RIGHT = "CMD_CLENCH_RIGHT"
    
    // Музыка
    const val MUSIC_PLAY = "CMD_MUSIC_PLAY"
    const val MUSIC_PAUSE = "CMD_MUSIC_PAUSE"
    
    // общение
    const val MENU = "CMD_MENU"
    const val FUNC_ABOUT = "CMD_FUNC_ABOUT"
}
```
