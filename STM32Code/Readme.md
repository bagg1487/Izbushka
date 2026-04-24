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

<p align="center">
  <img src="https://github.com/user-attachments/assets/0af85be7-3b2f-4274-9cc3-78446ddd34e8" alt="Image" width="400">
  <br>
  <em>Фото 1</em>
</p>

- Разархивировать архив, перейти в папку и запустите файл. В открывшемся окне в поле CID вставить ранее скопированный текст, остальные поля настроить как на картинке и нажать Generate
  
<p align="center">
  <img src="https://github.com/user-attachments/assets/324a506f-db1d-4b22-940c-64293a15acb1" alt="Image" width="319">
  <br>
  <em>Фото 2</em>
</p>

- Скопировать сгенерированный код, вернуться в License Management и в поле New License ID Code вставить скопированный текст и нажать Add LIC

### Установка **STM32CubeMX**

- Запускаем **STM32CubeMX**, во вкладке Project Manager проверяем, что в Toolchain / IDE выбрано MDK-ARV 4 версии

<p align="center">
  <img src="https://github.com/user-attachments/assets/c3aaedf1-7420-48c9-b063-51a561d76c12" alt="Image" width="400">
  <br>
  <em>Фото 3</em>
</p>

- Нажимаем File -> Load Project и выбрать нужный проект с расширением ioc на гитхабе

<p align="center">
  <img src="https://github.com/user-attachments/assets/8667b7b8-18ab-407b-9b55-8c8a460853f7" alt="Image" width="400">
  <br>
  <em>Фото 4</em>
</p>

- После загрузки проекта нажимаем GENERATE CODE

<p align="center">
  <img src="https://github.com/user-attachments/assets/2f308fef-95c4-4d44-84f9-e4816946df3e" alt="Image" width="400">
  <br>
  <em>Фото 5</em>
</p>

- Автоматически открывается программа Keil с исходным кодом программы

<p align="center">
  <img src="https://github.com/user-attachments/assets/143f979f-021c-4e6e-aa77-cefd18238e44" alt="Image" width="400">
  <br>
  <em>Фото 6</em>
</p>

- После изменения кода нажать Build и после Load. Файл автоматически загрузится в STM

<p align="center">
  <img src="https://github.com/user-attachments/assets/47fb0378-4976-4917-b792-132637b27a0b" alt="Image" width="400">
  <br>
  <em>Фото 7</em>
</p>

При загрузке кода на STM32 необходимо подключить программатор.

<p align="center">
  <img src="https://github.com/user-attachments/assets/6d757376-ca02-4520-b4ea-93028d90d2b6" alt="Image" width="225">
  <br>
  <em>Фото 8</em>
</p>