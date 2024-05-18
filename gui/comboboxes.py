from PySide6.QtWidgets import QWidget, QComboBox
from capturer import Capturer

class ComboBoxes(QWidget):
    def __init__(self, parent:QWidget):
        super().__init__(parent)
        
        self.serials = QComboBox(self)
        self.serials.setPlaceholderText("000000000000")
        self.serials.setFrame(False)

        self.resolutions = QComboBox(self)
        self.resolutions.addItems(
            [str(res[0])+" x "+str(res[1]) for res in Capturer.depthResolutions]
        )
        self.resolutions.currentIndexChanged.connect(parent.onDepthResolutionChange)
        self.resolutions.setFrame(False)

        self.depthFrameRates = QComboBox(self)
        self.depthFrameRates.addItems(
            [str(fps) for fps in Capturer.depthFrameRates]
        )
        self.depthFrameRates.currentIndexChanged.connect(parent.onDepthFrameRateChange)
        self.depthFrameRates.setFrame(False)

        self.gyroFrameRates = QComboBox(self)
        self.gyroFrameRates.addItems(
            [str(fps) for fps in Capturer.gyroFrameRates]
        )
        self.gyroFrameRates.currentIndexChanged.connect(parent.onGyroFrameRateChange)
        self.gyroFrameRates.setFrame(False)

        self.accelFrameRates = QComboBox(self)
        self.accelFrameRates.addItems(
            [str(fps) for fps in Capturer.accelFrameRates]
        )
        self.accelFrameRates.currentIndexChanged.connect(parent.onAccelFrameRateChange)
        self.accelFrameRates.setFrame(False)

    def setEnabled(self, arg__1: bool):
        self.serials.setEnabled(arg__1)
        self.resolutions.setEnabled(arg__1)
        self.depthFrameRates.setEnabled(arg__1)
        self.gyroFrameRates.setEnabled(arg__1)
        self.accelFrameRates.setEnabled(arg__1)

    def addDeviceSerial(self, serial:str):
        if self.serials.findText(serial) == -1:
            self.serials.addItem(serial)
            # Auto select first added device
            if self.serials.findText(serial) == 0:
                self.serials.setCurrentIndex(0)
            return True
        return False

    def removeDeviceSerial(self, serial:str):
        index = self.serials.findText(serial)
        if index > -1:
            self.serials.removeItem(index)
            return True
        return False
    
    def setDefaultValues(self):
        self.resolutions.setCurrentIndex(0)
        self.depthFrameRates.setCurrentIndex(0)
        self.gyroFrameRates.setCurrentIndex(0)
        self.accelFrameRates.setCurrentIndex(0)
        self.serials.update()
