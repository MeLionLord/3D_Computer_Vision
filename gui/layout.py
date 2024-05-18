from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QGridLayout, QFormLayout,
    QLabel, QFrame, QSizePolicy, QStackedLayout, QToolButton
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QPixmap
from capturer import Capturer
from comboboxes import ComboBoxes
from spinboxes import SpinBoxes

class Layout(QWidget):
    modelToolTips = ["Switch to Mesh", "Switch to Point Cloud"]
    layoutToolTips = ["Switch to Model View", "Switch to Stream View"]

    def __init__(self, parent:QWidget):
        super().__init__(parent)
        iconFolder = "icons"

        # ComboBoxes
        parent.comboBoxes = ComboBoxes(parent)

        # SpinBoxes
        parent.spinBoxes = SpinBoxes(parent)

        # Labels & Layouts
        mainLayout = QHBoxLayout()
        mainLayout.setSpacing(2)
        mainLayout.setContentsMargins(0,0,0,0)
        ml1 = QFrame(self)
        ml1.setFrameStyle(QFrame.Shape.Panel | QFrame.Shadow.Plain)
        ml2 = QWidget(self)
        mainLayout.addWidget(ml1, 2)
        mainLayout.addWidget(ml2, 8)

        streamParamsLayout = QVBoxLayout()
        streamParamsLayout.setSpacing(2)
        streamParamsLayout.setContentsMargins(2,2,2,2)
        il1 = QWidget(self)
        depthCamLabel = QLabel(Capturer.deviceName, self)
        streamParamsLayout.addWidget(depthCamLabel)
        streamParamsLayout.addWidget(il1)
        streamParamsLayout.addStretch()
        
        cameraParamsLayout = QFormLayout()
        cameraParamsLayout.setVerticalSpacing(4)
        cameraParamsLayout.setHorizontalSpacing(60)
        cameraParamsLayout.setContentsMargins(6,4,6,12)
        statusLabel = QLabel("Status:", self)
        serialLabel = QLabel("Serial Number:", self)
        resolutionLabel = QLabel("Resolution:", self)
        depthFrameRateLabel = QLabel("Depth Frame Rate:", self)
        gyroFrameRateLabel = QLabel("Gyro Frame Rate:", self)
        accelFrameRateLabel = QLabel("Accel Frame Rate:", self)
        depthFormatLabel = QLabel("Depth Format:", self)
        colorFormatLabel = QLabel("Color Format:", self)
        gyroFormatLabel = QLabel("Gyro Format:", self)
        accelFormatLabel = QLabel("Accel Format:", self)
        depthThresholdLabel = QLabel("Max. Distance:", self)
        parent.camState = QLabel(parent.camStates[0], self)
        depthSetFormatLabel = QLabel("Z16", self)
        colorSetFormatLabel = QLabel("RGB8", self)
        gyroSetFormatLabel = QLabel("MOTION_XYZ32F", self)
        accelSetFormatLabel = QLabel("MOTION_XYZ32F", self)
        cameraParamsLayout.addRow(statusLabel, parent.camState)
        cameraParamsLayout.addRow(serialLabel, parent.comboBoxes.serials)
        cameraParamsLayout.addRow(resolutionLabel, parent.comboBoxes.resolutions)
        cameraParamsLayout.addRow(depthFrameRateLabel, parent.comboBoxes.depthFrameRates)
        cameraParamsLayout.addRow(depthThresholdLabel, parent.spinBoxes.threshold)
        cameraParamsLayout.addRow(colorFormatLabel, colorSetFormatLabel)
        cameraParamsLayout.addRow(depthFormatLabel, depthSetFormatLabel)
        cameraParamsLayout.addRow(accelFrameRateLabel, parent.comboBoxes.accelFrameRates)
        cameraParamsLayout.addRow(accelFormatLabel, accelSetFormatLabel)
        cameraParamsLayout.addRow(gyroFrameRateLabel, parent.comboBoxes.gyroFrameRates)
        cameraParamsLayout.addRow(gyroFormatLabel, gyroSetFormatLabel)

        processingParamsLayout = QVBoxLayout()
        processingParamsLayout.setSpacing(2)
        processingParamsLayout.setContentsMargins(2,2,2,2)
        il2 = QWidget(self)
        il3 = QWidget(self)
        il4 = QWidget(self)
        il5 = QWidget(self)
        icpLabel = QLabel("Iterative Closest Point Algorithm", self)
        filterLabel = QLabel("Filtering by Standard Deviation", self)
        triangLabel = QLabel("Poisson Surface Reconstruction", self)
        cropLabel = QLabel("Crop Parameters", self)
        processingParamsLayout.addWidget(icpLabel)
        processingParamsLayout.addWidget(il2)
        processingParamsLayout.addWidget(filterLabel)
        processingParamsLayout.addWidget(il3)
        processingParamsLayout.addWidget(triangLabel)
        processingParamsLayout.addWidget(il4)
        processingParamsLayout.addWidget(cropLabel)
        processingParamsLayout.addWidget(il5)
        processingParamsLayout.addStretch()

        icpParamsLayout = QFormLayout()
        icpParamsLayout.setVerticalSpacing(4)
        icpParamsLayout.setHorizontalSpacing(30)
        icpParamsLayout.setContentsMargins(6,4,6,12)
        memoryLabel = QLabel("Memory Cell Size:", self)
        voxelFilterLabel = QLabel("Voxel Filter Size:", self)
        outlierRadiusLabel = QLabel("Pre-Filter Radius Size:", self)
        outlierNeighsLabel = QLabel("Pre-Filter Min Neighbours:", self)
        skipFitnessLabel = QLabel("Min Fitness to Skip Frame:", self)
        skipVoxelLabel = QLabel("Voxel Compare Size:", self)
        itersVoxelsLabel = QLabel("ICP Voxel Sizes with Iterations:", self)
        lastLambdaLabel = QLabel("Lambda in Last Iteration:", self)
        voxelFilterSetLabel = QLabel("0,001 m", self)
        itersVoxelsSetLabel = QLabel("{5-1, 3-3, 2-1}", self)
        icpParamsLayout.addRow(memoryLabel, parent.spinBoxes.memoryCellSize)
        icpParamsLayout.addRow(voxelFilterLabel, voxelFilterSetLabel)
        icpParamsLayout.addRow(outlierRadiusLabel, parent.spinBoxes.outlierRadius)
        icpParamsLayout.addRow(outlierNeighsLabel, parent.spinBoxes.outlierNeighs)
        icpParamsLayout.addRow(skipFitnessLabel, parent.spinBoxes.skipFitness)
        icpParamsLayout.addRow(skipVoxelLabel, parent.spinBoxes.skipVoxelSize)
        icpParamsLayout.addRow(itersVoxelsLabel, itersVoxelsSetLabel)
        icpParamsLayout.addRow(lastLambdaLabel, parent.spinBoxes.lastLambda)

        filterParamsLayout = QFormLayout()
        filterParamsLayout.setVerticalSpacing(4)
        filterParamsLayout.setHorizontalSpacing(41)
        filterParamsLayout.setContentsMargins(6,4,6,12)
        statNeighsLabel = QLabel("Statistical Filter Neighbours:", self)
        stdRatioLabel = QLabel("Standard Deviation Ratio:", self)
        filterParamsLayout.addRow(statNeighsLabel, parent.spinBoxes.statisticNeighs)
        filterParamsLayout.addRow(stdRatioLabel, parent.spinBoxes.standardRatio)

        triangParamsLayout = QFormLayout()
        triangParamsLayout.setVerticalSpacing(4)
        triangParamsLayout.setHorizontalSpacing(25)
        triangParamsLayout.setContentsMargins(6,4,6,12)
        orientNeighsLabel = QLabel("Neighbours to Orient Normals:", self)
        poissDepthLabel = QLabel("Octal Tree Depth:", self)
        triangParamsLayout.addRow(orientNeighsLabel, parent.spinBoxes.orientNormalsNeighs)
        triangParamsLayout.addRow(poissDepthLabel, parent.spinBoxes.octalTreeDepth)

        cropParamsLayout = QGridLayout()
        cropParamsLayout.setVerticalSpacing(4)
        cropParamsLayout.setHorizontalSpacing(4)
        cropParamsLayout.setContentsMargins(6,0,6,6)
        rotationLabel = QLabel("Rotation:", self)
        minBoundLabel = QLabel("Min Bound:", self)
        maxBoundLabel = QLabel("Max Bound:", self)
        xLabel = QLabel("x", self)
        yLabel = QLabel("y", self)
        zLabel = QLabel("z", self)
        cropParamsLayout.addWidget(xLabel,0,1,Qt.AlignmentFlag.AlignHCenter)
        cropParamsLayout.addWidget(yLabel,0,2,Qt.AlignmentFlag.AlignHCenter)
        cropParamsLayout.addWidget(zLabel,0,3,Qt.AlignmentFlag.AlignHCenter)
        cropParamsLayout.addWidget(rotationLabel,1,0)
        cropParamsLayout.addWidget(parent.spinBoxes.rotationX,1,1)
        cropParamsLayout.addWidget(parent.spinBoxes.rotationY,1,2)
        cropParamsLayout.addWidget(parent.spinBoxes.rotationZ,1,3)
        cropParamsLayout.addWidget(minBoundLabel,2,0)
        cropParamsLayout.addWidget(parent.spinBoxes.minBoundX,2,1)
        cropParamsLayout.addWidget(parent.spinBoxes.minBoundY,2,2)
        cropParamsLayout.addWidget(parent.spinBoxes.minBoundZ,2,3)
        cropParamsLayout.addWidget(maxBoundLabel,3,0)
        cropParamsLayout.addWidget(parent.spinBoxes.maxBoundX,3,1)
        cropParamsLayout.addWidget(parent.spinBoxes.maxBoundY,3,2)
        cropParamsLayout.addWidget(parent.spinBoxes.maxBoundZ,3,3)
        cropParamsLayout.setColumnStretch(0,7)
        cropParamsLayout.setColumnStretch(1,1)
        cropParamsLayout.setColumnStretch(2,1)
        cropParamsLayout.setColumnStretch(3,1)

        rightLayout = QVBoxLayout()
        rightLayout.setSpacing(2)
        rightLayout.setContentsMargins(0,0,2,0)
        pl1 = QFrame(self)
        pl1.setFrameStyle(QFrame.Shape.Panel | QFrame.Shadow.Plain)
        pl2 = QFrame(self)
        pl2.setFrameStyle(QFrame.Shape.Panel | QFrame.Shadow.Plain)
        rightLayout.addWidget(pl1,1)
        rightLayout.addWidget(pl2,99)

        nameLayout = QHBoxLayout()
        nameLayout.setContentsMargins(2,0,0,0)
        self.modelPixmap = [QPixmap(f"{iconFolder}/cube-dots-fill.png"),
                            QPixmap(f"{iconFolder}/cube-dots.png")]
        self.modelButton = QToolButton(self)
        self.modelButton.setAutoRaise(True)
        self.modelButton.clicked.connect(parent.onSwitchModels)
        self.projNameLabel = QLabel("New_Project*", self)
        self.layoutPixmap = [QPixmap(f"{iconFolder}/layout-square.png"),
                             QPixmap(f"{iconFolder}/layout-border.png")]
        self.layoutButton = QToolButton(self)
        self.layoutButton.setAutoRaise(True)
        self.layoutButton.clicked.connect(self.onSwitchLayouts)
        nameLayout.addWidget(self.projNameLabel)
        nameLayout.addStretch()
        nameLayout.addWidget(self.modelButton)
        nameLayout.addWidget(self.layoutButton)

        visLayout = QGridLayout()
        visLayout.setSpacing(6)
        visLayout.setContentsMargins(6,6,6,6)
        visLayout.addWidget(parent._colorScreen,0,0)
        visLayout.addWidget(parent._depthScreen,0,1)
        visLayout.addWidget(parent._streamCloudScreen.windowContainer,1,0)
        visLayout.addWidget(parent._motionScreen.windowContainer,1,1)
        visLayout.setRowStretch(0,1)
        visLayout.setRowStretch(1,1)
        visLayout.setColumnStretch(0,1)
        visLayout.setColumnStretch(1,1)

        modelLayout = QVBoxLayout()
        modelLayout.addWidget(parent._modelScreen.windowContainer)

        self.stackedVisualizers = QStackedLayout()
        sl1 = QWidget(self)
        sl2 = QWidget(self)
        self.stackedVisualizers.addWidget(sl1)
        self.stackedVisualizers.addWidget(sl2)

        self.stackedParameters = QStackedLayout()
        sl3 = QWidget(self)
        sl4 = QWidget(self)
        self.stackedParameters.addWidget(sl3)
        self.stackedParameters.addWidget(sl4)

        ml1.setLayout(self.stackedParameters)
        ml2.setLayout(rightLayout)
        pl1.setLayout(nameLayout)
        pl2.setLayout(self.stackedVisualizers)
        il1.setLayout(cameraParamsLayout)
        il2.setLayout(icpParamsLayout)
        il3.setLayout(filterParamsLayout)
        il4.setLayout(triangParamsLayout)
        il5.setLayout(cropParamsLayout)
        sl1.setLayout(visLayout)
        sl2.setLayout(modelLayout)
        sl3.setLayout(streamParamsLayout)
        sl4.setLayout(processingParamsLayout)

        ml1.setSizePolicy(QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding))
        pl1.setSizePolicy(QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed))

        self.setLayout(mainLayout)
        self.setDefaultValues()

    def onSwitchLayouts(self):
        index = not self.stackedVisualizers.currentIndex()
        self.stackedVisualizers.setCurrentIndex(index)
        self.stackedParameters.setCurrentIndex(index)
        self.layoutButton.setIcon(self.layoutPixmap[index])
        self.layoutButton.setToolTip(Layout.layoutToolTips[index])
        self.modelButton.setVisible(index)

        p = self.parent()
        if index:
            p._visTimer.timeout.disconnect(p.onStreamTimerTick)
            p._visTimer.timeout.connect(p.onModelTimerTick)
        else:
            p._visTimer.timeout.disconnect(p.onModelTimerTick)
            p._visTimer.timeout.connect(p.onStreamTimerTick)

    def onSwitchModelIcons(self):
        p = self.parent()
        index = not p.modelIndex
        p.modelIndex = index
        self.modelButton.setIcon(
            self.modelPixmap[index]
        )
        self.modelButton.setToolTip(Layout.modelToolTips[index])
        return index

    def setDefaultValues(self):
        p = self.parent()

        p.modelIndex = False
        self.modelButton.setIcon(self.modelPixmap[0])
        self.modelButton.setToolTip(Layout.modelToolTips[0])
        self.modelButton.setVisible(False)

        self.layoutButton.setIcon(self.layoutPixmap[0])
        self.layoutButton.setToolTip(Layout.layoutToolTips[0])

        p.comboBoxes.setDefaultValues()
        p.spinBoxes.setDefaultValues()
