import os
#import face_recognition
import cv2
import time

import numpy as np
from transliterate import translit
from imutils import paths
import sys

sys.path.append('')


class FaceAction:

    def __init__(self):
        self.frame = None
        self.name = ''
        self.face_detection_trig = False
        self.recognizer = cv2.face.LBPHFaceRecognizer_create()
        self.current_dir = os.path.dirname(__file__)
        self.recognizer.read(self.current_dir+'/Face/face.yml')
        self.detector = cv2.CascadeClassifier(self.current_dir+"/Face/haarcascade_frontalface_default.xml")
        self.name = self.Read_from_txt(self.current_dir+'/Face/name')

    def Read_from_txt(self, filename):
        file1 = open(filename + ".txt", "r")
        list_row = file1.readlines()
        list_source = []
        for i in range(len(list_row)):
            column_list = list_row[i].strip().split("\t")
            list_source.append(column_list)
        for i in range(len(list_source)):
            for j in range(len(list_source[i])):
                list_source[i][j] = str(list_source[i][j])
        file1.close()
        return list_source

    def Save_to_txt(self, list_name,list_label):
        file2 = open('Face/name.txt', 'w')
        for i in range(len(list_name)):
            file2.write(str(list_name[i]))
            file2.write('\t')
            file2.write(str(list_label[i]))
            file2.write('\n')
        file2.close()
    def getFaceInfo(self,path='dataset'):
        imagePaths = list(
            paths.list_images(self.current_dir+"/"+path)) # загружаем в список все изображения из папки dataset и вложенных в нее

        #imagePaths = [os.path.join(path,f) for f in os.listdir(path)]
        faceSamples=[]
        names = []
        for imagePath in imagePaths:
            if os.path.split(imagePath)[-1].split(".")[1]=="jpg":
                name = imagePath.split(os.sep)[-2]
                img = cv2.imread(imagePath)
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                faces = self.detector.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5)
                for (x,y,w,h) in faces:
                    faceSamples.append(gray[y:y+h,x:x+w])
                    names.append(name)
        labels = [item for item in range(0, len(names))]
        return faceSamples,labels, names
    def trainImage(self):
        faces, labels, names = self.getFaceInfo()
        self.recognizer.train(faces, np.array(labels))
        self.recognizer.write(self.current_dir+'/Face/face.yml')
        self.recognizer.read(self.current_dir+'/Face/face.yml')
        self.Save_to_txt(labels,names)
        print("\n  {0} faces trained.".format(len(np.unique(labels))))
    def face_detect(self,img):
        if self.face_detection_trig:
            try:
                self.name = self.Read_from_txt(self.current_dir+'/Face/name')
                #if sys.platform.startswith('win') or sys.platform.startswith('darwin'):
                gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                faces = self.detector.detectMultiScale(gray,1.2,5)
                if len(faces)>0 :
                    for (x,y,w,h) in faces:
                        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        id, confidence = self.recognizer.predict(gray[y:y + h, x:x + w])
                        print("confidence",id,confidence)
                        if confidence > 100:
                            cv2.putText(img, str("unknow"), (x + 5, y + h + 30), cv2.FONT_HERSHEY_DUPLEX, 1,
                                        (0, 255, 0), 2)
                        else:
                            cv2.putText(img, self.name[int(id)][1], (x + 5, y + h + 30), cv2.FONT_HERSHEY_DUPLEX, 1,
                                        (0, 255, 0), 2)
            except Exception as e:
                print(e)
    def add_user(self, rec):
        """Добавление нового пользователя.
        На основе диалога узнает информацию о пользователе и добавляет его в существующую модель распознавания
        Аргументы: rec - модуль распознавания голоса
        """
        print('Назовите имя пользователя')
        trig_name = True  # Установить флаг trig_name в True
        name = ""  # Инициализировать переменную name

        # Бесконечный цикл для ожидания ввода имени пользователя и подтверждения
        while True:
            rec.listening()  # Прослушивать фразы
            if trig_name and rec.get_full_phrase():  # Если фраза получена и флаг trig_name установлен
                name = rec.get_full_phrase()  # Получить имя пользователя
                name = name.lower()  # Привести имя к нижнему регистру
                print('Ваше имя: ' + name + 'все верно?')
                trig_name = False  # Установить флаг trig_name в False
            if not trig_name:  # Если флаг trig_name установлен в False
                query = rec.get_full_phrase()  # Получить полную фразу
                query = query.lower()  # Привести фразу к нижнему регистру
                if 'да' in query:  # Если в ответе есть слово "да"
                    print('В течении 10 секунд изменяйте положение головы, принимая различные позы.')
                    eng_name = translit(name, language_code='ru',
                                        reversed=True)  # Транслитерировать имя на английский язык
                    if not os.path.isdir(
                            "dataset" + os.path.sep + eng_name):  # Если папка с именем пользователя не существует
                        os.mkdir("dataset" + os.path.sep + eng_name)  # Создать папку с именем пользователя
                    for i in range(1, 10):  # Цикл по номерам изображений от 1 до 9
                        time.sleep(1)  # Подождать 1 секунду
                        save_pass = "dataset" + os.path.sep + eng_name + os.path.sep + "image_{}.jpg".format(
                            i)  # Путь сохранения изображения
                        if self.frame is not None:  # Если есть изображение
                            cv2.imwrite(save_pass, self.frame)  # Сохранить изображение
                            print("{} written!".format(save_pass))  # Вывести сообщение о сохранении изображения
                            cv2.putText(self.frame, str(i), (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 0),
                                        2)  # Нанести номер изображения на кадр
                    return self.trainImage()  # Выполнить обучение модели
                elif 'нет' in query:  # Если в ответе есть слово "нет"
                    trig_name = True  # Установить флаг trig_name в True (ожидать новое имя пользователя)