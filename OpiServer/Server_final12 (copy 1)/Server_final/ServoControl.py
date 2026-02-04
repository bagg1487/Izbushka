import os


class ServoControl:
    """Класс для работы с калибровкой сервоприводов"""
    def __init__(self):
        self.parent_dir = os.path.dirname(os.path.abspath(__file__))
        self.calibration_point = self.readFromFile('angles')

    def saveToFile(self, list, filename):
        """Записывает состояние углов всех сервоприводов в файл\n
        Аргументы: list - список углов, filename - имя файла для записи
        """
        file2 = open(self.parent_dir+"/"+filename + '.txt', 'w')
        for i in range(len(list)):
            for j in range(len(list[i])):
                if str(list[i][j]) != "":
                    file2.write(str(list[i][j]))
                    file2.write('#')
            file2.write('\n')
        file2.close()

    def readFromFile(self, filename):
        """Чтение состояния углов всех сервоприводов в файл\n
        Аргументы: filename - имя файла для чтения
        Возвращает: список углов
        """
        try:
            # Пытаемся открыть файл для чтения
            file1 = open(self.parent_dir + "/" + filename + ".txt", "r")
            # Читаем строки из файла
            list_row = file1.readlines()
            list_source = []
            # Обрабатываем строки файла
            for i in range(len(list_row)):
                column_list = list_row[i].strip().split("#")
                list_source.append(column_list)
            # Преобразуем строки в числа
            for i in range(len(list_source)):
                for j in range(len(list_source[i]) - 1):
                    list_source[i][j] = int(list_source[i][j])
            file1.close()
        # Если возникает ошибка ввода-вывода (файл не найден или нет разрешения на чтение)
        except IOError:
            # Создаем список значений по умолчанию
            list_source = [[90, 90, 90, 90, 90], [90, 90, 90, 90, 90], [90, 90, 90, 90, 90], [90, 90, 90, 90, 90]]
            # Сохраняем список значений по умолчанию в файл
            self.saveToFile(list_source, "angles")

        # Возвращаем полученный список значений
        return list_source

    def getCalibrationCommand(self):
        """Извлекает из файла текущее значение калибровочных углов и возвращает в виде параметров для команды"""
        # Создаем строку-команду с начальным символом "#"
        command = "#"
        # Открываем файл для чтения
        file1 = open(self.parent_dir + "/" + "angles.txt", "r")
        # Читаем строки из файла
        list_row = file1.readlines()
        # Обрабатываем строки файла
        for i in range(len(list_row)):
            # Добавляем значения из каждой строки к команде
            command += list_row[i].split('\n')[0]
        file1.close()
        # Возвращаем сформированную команду
        return command

    def setAngle(self, data):
        """Записывает значения калибровочных углов пришедших с клиента в список для дальнейшей записи в файл"""
        if data[1] == "L":
            if data[2] == "A":
                self.calibration_point[0][0] = int(data[3])
            elif data[2] == "B":
                self.calibration_point[0][1] = int(data[3])
            elif data[2] == "C":
                self.calibration_point[0][2] = int(data[3])
            elif data[2] == "D":
                self.calibration_point[0][3] = int(data[3])
            elif data[2] == "E":
                self.calibration_point[0][4] = int(data[3])
        if data[1] == "R":
            if data[2] == "A":
                self.calibration_point[1][0] = int(data[3])
            elif data[2] == "B":
                self.calibration_point[1][1] = int(data[3])
            elif data[2] == "C":
                self.calibration_point[1][2] = int(data[3])
            elif data[2] == "D":
                self.calibration_point[1][3] = int(data[3])
            elif data[2] == "E":
                self.calibration_point[1][4] = int(data[3])
        if data[1] == "H":
            if data[2] == "A":
                self.calibration_point[2][0] = int(data[3])
            elif data[2] == "B":
                self.calibration_point[2][1] = int(data[3])
            elif data[2] == "C":
                self.calibration_point[2][2] = int(data[3])
            elif data[2] == "D":
                self.calibration_point[2][3] = int(data[3])
            elif data[2] == "E":
                self.calibration_point[2][4] = int(data[3])
        if data[1] == "M":
            if data[2] == "A":
                self.calibration_point[3][0] = int(data[3])
            elif data[2] == "B":
                self.calibration_point[3][1] = int(data[3])
            elif data[2] == "C":
                self.calibration_point[3][2] = int(data[3])
            elif data[2] == "D":
                self.calibration_point[3][3] = int(data[3])
            elif data[2] == "E":
                self.calibration_point[3][4] = int(data[3])