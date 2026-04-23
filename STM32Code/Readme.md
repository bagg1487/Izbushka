# Для работы с STM32

---

# Необходимо установить:

- [Keil](https://ios.ru/files/Keil.rar)
- [STM32CubeMX](https://ios.ru/files/stm32cubemx.zip)
- Также [firmware](https://ios.ru/files/stm32cube_fw_v1270.zip) для STM32CubeMX\

---

## Инструкция:

### Установка **Keil**

- Скачать архив с Keil
- Установить Keil
- Запустить по именем администратора и перейти в File -> License Management
- . В открывшемся окне скопировать CID

![alt text](image/image1.png)

- Разархивировать архив, перейти в папку и запустите файл. В открывшемся окне в поле CID вставить ранее скопированный текст, остальные поля настроить как на картинке и нажать Generate
  
![alt text](image/image2.png)

- Скопировать сгенерированный код, вернуться в License Management и в поле New License ID Code вставить скопированный текст и нажать Add LIC

### Установка **STM32CubeMX**

- Запускаем **STM32CubeMX**, во вкладке Project Manager проверяем, что в Toolchain / IDE выбрано MDK-ARV 4 версии

![alt text](image/image3.png)

- Нажимаем File -> Load Project и выбрать нужный проект с расширением ioc на гитхабе

![alt text](image/image4.png)

- После загрузки проекта нажимаем GENERATE CODE

![alt text](image/image5.png)

- Автоматически открывается программа Keil с исходным кодом программы

![alt text](image/image6.png)

- После изменения кода нажать Build и после Load. Файл автоматически загрузится в STM

![alt text](image/image7.png)

При загрузке кода на STM32 необходимо подключить программатор.

![alt text](image/image8.png)