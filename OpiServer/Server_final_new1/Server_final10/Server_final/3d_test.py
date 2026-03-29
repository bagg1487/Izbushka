import sys

from PyQt5 import Qt3DCore, Qt3DExtras
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout

from PyQt5.QtGui import QVector3D
from PyQt5.QtCore import Qt
from PyQt5.uic.properties import QtGui


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("3D Model Rotation Example")
        self.setGeometry(100, 100, 800, 600)

        # Создание сцены и вида 3D
        self.scene = Qt3DCore.QEntity()
        self.view = Qt3DExtras.Qt3DWindow()
        container = QWidget.createWindowContainer(self.view)
        self.setCentralWidget(container)

        # Создание сферы (3D модели)
        self.sphere = Qt3DExtras.QSphereMesh()
        self.sphereMesh = Qt3DCore.QEntity(self.scene)
        self.sphereMesh.addComponent(self.sphere)

        # Создание материала для сферы
        self.material = Qt3DExtras.QPhongMaterial(self.scene)

        # Установка материала сферы
        self.sphereMesh.addComponent(self.material)

        # Установка положения сферы
        transform = Qt3DCore.QTransform()
        transform.setScale(1)
        self.sphereMesh.addComponent(transform)

        # Добавление сцены в вид 3D
        self.view.setRootEntity(self.scene)

        # Установка обработчиков событий мыши
        container.mousePressEvent = self.mousePressEvent
        container.mouseMoveEvent = self.mouseMoveEvent
        container.mouseReleaseEvent = self.mouseReleaseEvent

        self.last_mouse_position = None
        self.rotating = False

    def mousePressEvent(self, event):
        if event.buttons() == Qt.LeftButton:
            self.last_mouse_position = event.pos()
            self.rotating = True

    def mouseMoveEvent(self, event):
        if self.rotating and self.last_mouse_position:
            delta = event.pos() - self.last_mouse_position
            rotation_axis = QVector3D(delta.y(), delta.x(), 0)
            angle = delta.manhattanLength() / 2
            self.sphereMesh.transform().setRotation(
                self.sphereMesh.transform().rotation() * QtGui.QQuaternion.fromAxisAndAngle(rotation_axis, angle)
            )
            self.last_mouse_position = event.pos()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.rotating = False
            self.last_mouse_position = None

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
