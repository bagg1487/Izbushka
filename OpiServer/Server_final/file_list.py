import os

# Указываем путь к директории
directory = os.path.dirname(__file__)

# Получаем список файлов
files = os.listdir(directory)

# Выводим список файлов
print(files)