import os
import sys
import cv2
import numpy as np
from imutils import paths


class  Face:
    def __init__(self):
        self.recognizer = cv2.face.LBPHFaceRecognizer_create()
        self.recognizer.read('Face/face.yml')
        self.detector = cv2.CascadeClassifier("Face/haarcascade_frontalface_default.xml")
        self.name = self.Read_from_txt('Face/name')
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
            paths.list_images(path))  # загружаем в список все изображения из папки dataset и вложенных в нее

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
        self.recognizer.write('Face/face.yml')
        self.recognizer.read('Face/face.yml')
        self.Save_to_txt(labels,names)
        print("\n  {0} faces trained.".format(len(np.unique(labels))))
    def face_detect(self,img):
        try:
            self.name = self.Read_from_txt('Face/name')
            if sys.platform.startswith('win') or sys.platform.startswith('darwin'):
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

if __name__ == '__main__':
    f=Face()
    f.trainImage()


