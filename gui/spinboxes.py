from PySide6.QtWidgets import QWidget, QSpinBox, QDoubleSpinBox, QAbstractSpinBox

from customdoublespinbox import CustomDoubleSpinBox
from customspinbox import CustomSpinBox

class SpinBoxes(QWidget):
    def __init__(self, parent:QWidget):
        super().__init__(parent)
        
        self.threshold = CustomDoubleSpinBox(self, 2, 0.5, 0.5, 10.0, " m")
        self.threshold.valueChanged.connect(parent.onDistanceThresholdChange)

        self.memoryCellSize = CustomSpinBox(self, 1, 1, 100)
        self.outlierRadius = CustomDoubleSpinBox(self, 3, 0.001, 0.0, 0.01, " m")
        self.outlierNeighs = CustomSpinBox(self, 1, 0, 100)
        self.skipFitness = CustomDoubleSpinBox(self, 3, 0.01, 0.5, 1.0)
        self.skipVoxelSize = CustomDoubleSpinBox(self, 3, 0.001, 0.0, 0.01, " m")
        self.lastLambda = CustomDoubleSpinBox(self, 3, 0.01, 0.5, 1.0)

        self.statisticNeighs = CustomSpinBox(self, 1, 5, 500)
        self.standardRatio = CustomDoubleSpinBox(self, 2, 0.1, 0.1, 10.0)

        self.orientNormalsNeighs = CustomSpinBox(self, 1, 5, 500)
        self.octalTreeDepth = CustomSpinBox(self, 1, 1, 10)

        self.rotationX = CustomDoubleSpinBox(self, 1, 1, -360, 360, " °")
        self.rotationY = CustomDoubleSpinBox(self, 1, 1, -360, 360, " °")
        self.rotationZ = CustomDoubleSpinBox(self, 1, 1, -360, 360, " °")
        self.minBoundX = CustomDoubleSpinBox(self, 3, 0.01, -100, 100, " m")
        self.minBoundY = CustomDoubleSpinBox(self, 3, 0.01, -100, 100, " m")
        self.minBoundZ = CustomDoubleSpinBox(self, 3, 0.01, -100, 100, " m")
        self.maxBoundX = CustomDoubleSpinBox(self, 3, 0.01, -100, 100, " m")
        self.maxBoundY = CustomDoubleSpinBox(self, 3, 0.01, -100, 100, " m")
        self.maxBoundZ = CustomDoubleSpinBox(self, 3, 0.01, -100, 100, " m")

        self.rotationX.setMaximumWidth(60)
        self.rotationY.setMaximumWidth(60)
        self.rotationZ.setMaximumWidth(60)
        self.minBoundX.setMaximumWidth(60)
        self.minBoundY.setMaximumWidth(60)
        self.minBoundZ.setMaximumWidth(60)
        self.maxBoundX.setMaximumWidth(60)
        self.maxBoundY.setMaximumWidth(60)
        self.maxBoundZ.setMaximumWidth(60)
        
    def setDefaultValues(self):
        self.threshold.setValue(1.0)

        self.memoryCellSize.setValue(30)
        self.outlierRadius.setValue(0.005)
        self.outlierNeighs.setValue(70)
        self.skipFitness.setValue(0.96)
        self.skipVoxelSize.setValue(0.003)
        self.lastLambda.setValue(1.0)

        self.statisticNeighs.setValue(50)
        self.standardRatio.setValue(2.5)

        self.orientNormalsNeighs.setValue(100)
        self.octalTreeDepth.setValue(7)

        self.rotationX.setValue(-4.5)
        self.rotationY.setValue(-2.5)
        self.rotationZ.setValue(18)
        self.minBoundX.setValue(-0.11)
        self.minBoundY.setValue(-0.13)
        self.minBoundZ.setValue(0.01)
        self.maxBoundX.setValue(0.09)
        self.maxBoundY.setValue(0.06)
        self.maxBoundZ.setValue(0.15)
