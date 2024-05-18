from PySide6.QtWidgets import QWidget, QLabel, QFrame, QSizePolicy
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import Qt, QSize

class ImageVisualizer(QLabel):
    def __init__(self, parent:QWidget, resolution=(320,240)):
        super().__init__(parent)

        self.setStyleSheet("background-color: #101010; padding: 1px;")
        self.image = QImage()
        self.setMinimumSize(resolution[0], resolution[1])
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.removeImage()

    def removeImage(self):
        self.setText("Stream Not Running")
        self.setFrameStyle(QFrame.Shape.StyledPanel | QFrame.Shadow.Plain)
    
    def setImage(self, color_data, resolution):
        self.image = QImage(color_data, resolution[0], resolution[1], QImage.Format.Format_RGB888)
        self.scaleImage()

    def scaleImage(self):
        image = self.image.scaled(
            self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.FastTransformation
        )
        self.setPixmap(QPixmap.fromImage(image))
