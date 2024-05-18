import sys
sys.coinit_flags = 2
import pythoncom # Needed to resolve COM error 0x80010106

import numpy as np
import pyrealsense2 as rs
import open3d as o3d

from layout import Layout
from toolbar import ToolBar
from imagevisualizer import ImageVisualizer
from open3dvisualizer import Open3DVisualizer
from capturer import Capturer
from projectmanager import ProjectManager
from reconstructor import Reconstructor

from PySide6.QtWidgets import(
    QMainWindow, QApplication, QMessageBox
)
from PySide6.QtCore import(
    Qt, QTimer
)
from PySide6.QtGui import(
    QMouseEvent, QCloseEvent, QResizeEvent
)
# import time

class MainWindow(QMainWindow):
    path_color = np.array([59/255, 130/255, 246/255], dtype=np.float64)
    camStates = ["Disconnected", "Connected", "Streaming"]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("3D Reconstruction")

        # Cameras initial settings
        self._depthRes = Capturer.depthResolutions[0]
        self._depthFPS = Capturer.depthFrameRates[0]
        self._gyroFPS = Capturer.gyroFrameRates[0]
        self._accelFPS = Capturer.accelFrameRates[0]
        self._depthPeriod = Capturer.depthCaptureRates[0]
        self._lastFrameId = -1
        self._visPeriod = 20
        self._resetView = False
        visResolution = (self._depthRes[0]//2, self._depthRes[1]//2)
        
        # Camera controller
        self._capturer = Capturer(1.00, self.onDevicesChanged)
        
        # 3D Geometries
        self._pathGeometry = o3d.geometry.LineSet()
        self._cameraGeometry = o3d.io.read_triangle_mesh("bin/d435_model.stl")
        center = self._cameraGeometry.get_center()
        self._cameraGeometry.scale(0.001, center=center)
        self._cameraGeometry.translate(-center)
        self._cameraGeometry.transform([[ 1, 0, 0, 0],
                                        [ 0, 0, 1, 0],
                                        [ 0,-1, 0, 0],
                                        [ 0, 0, 0, 1]])
        self._streamPointGeometry = o3d.geometry.PointCloud()
        self._modelGeometry = o3d.geometry.PointCloud()
        self.modelIndex = False

        # 3D Visualizers
        self._motionScreen = Open3DVisualizer(self, visResolution)
        self._streamCloudScreen = Open3DVisualizer(self, visResolution)
        self._modelScreen = Open3DVisualizer(self, self._depthRes)
        
        # 2D Visualizers
        self._colorScreen = ImageVisualizer(self, visResolution)
        self._depthScreen = ImageVisualizer(self, visResolution)

        # StatusBar
        self._statusBar = self.statusBar()

        # Labels & Layouts
        self._layout = Layout(self)
        self.setCentralWidget(self._layout)

        # ToolBar
        self._toolBar = ToolBar(self)
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, self._toolBar)
        
        # Timers checking for new data from cameras
        self._captureTimer = QTimer(self)
        self._captureTimer.timeout.connect(self.onCaptureTimerTick)
        self._visTimer = QTimer(self)
        self._visTimer.timeout.connect(self.onStreamTimerTick)

        # Style
        with open("style.qss", 'r') as qss:
            style = qss.read()
            app.setStyleSheet(style)
        self.show()

        # Open project dialog
        self._projManager = ProjectManager(self)
        if not self._projManager.begin("saves"): sys.exit()
        self.setFocus()

        # Reconstruction controller
        self._reconstructor = Reconstructor(self._projManager, self._modelScreen)
        
        # Search for connected devices at start-up
        devices = self._capturer.context.query_devices()
        for device in devices:
            self._addDevice(device)

        self.onShowPointModel()
        self._visTimer.start(self._visPeriod)

    def _resetStreamGeometries(self):
        self._pathGeometry.clear()
        self._pathGeometry.points = Open3DVisualizer.coordPoints
        self._pathGeometry.lines = Open3DVisualizer.coordLines
        self._pathGeometry.colors = Open3DVisualizer.coordColors
        self._motionScreen.removeAllGeometries()
        self._motionScreen.addGeometry(self._pathGeometry)
        self._motionScreen.addGeometry(self._cameraGeometry)

        points = Open3DVisualizer.createPlaceHolderPoints(self.spinBoxes.threshold.value())
        self._streamPointGeometry.clear()
        self._streamPointGeometry.points = points
        self._streamCloudScreen.removeAllGeometries()
        self._streamCloudScreen.addGeometry(self._streamPointGeometry)

    def _cleanStreamVisualizers(self):
        self._motionScreen.removeAllGeometries()
        self._streamCloudScreen.removeAllGeometries()
        self._motionScreen.addGeometry(Open3DVisualizer.emptyGeometry)
        self._streamCloudScreen.addGeometry(Open3DVisualizer.emptyGeometry)
        self._colorScreen.removeImage()
        self._depthScreen.removeImage()

    def _updateImageVisualizers(self):
        color, depth = self._capturer.getColorDepthImage()
        self._colorScreen.setImage(color, self._depthRes)
        self._depthScreen.setImage(depth, self._depthRes)
        return color, depth

    def _updatePointCloudVisualizer(self):
        pcd = self._capturer.updatePointCloud()
        if pcd is None or len(pcd.points) == 0: return None
        self._streamPointGeometry.points = pcd.points
        self._streamPointGeometry.colors = pcd.colors
        
        if self._resetView:
            self._streamCloudScreen.resetView()
            self._resetView = False
        else:
            self._streamCloudScreen.updateGeometries()
        self._streamCloudScreen.updateVis()
        return pcd

    def _updatePoseVisualizer(self):
        rot, trn = self._capturer.getPose()
        if rot is None or trn is None: return

        # end = len(self._pathGeometry.points)
        # self._pathGeometry.points.append(tr)
        # self._pathGeometry.lines.append(np.array([end-1, end], dtype=np.int32))
        # self._pathGeometry.colors.append(MainWindow.path_color)

        self._cameraGeometry.rotate(rot)
        self._motionScreen.updateGeometries()
        self._motionScreen.updateVis()
        self._cameraGeometry.rotate(np.transpose(rot))
        return rot, trn

    def onStreamTimerTick(self):
        self._motionScreen.updateVis()
        self._streamCloudScreen.updateVis()

    def onModelTimerTick(self):
        self._modelScreen.updateVis()

    def onCaptureTimerTick(self):
        if not Capturer.activeDevice:
            self.onStartStopStreaming()
            return
        
        frameId = self._capturer.getFrameId()
        if frameId != self._lastFrameId:
            self._lastFrameId = frameId
            color, depth = self._updateImageVisualizers()
            # if frameId % 2 == 0:
            #     self.updatePointCloudVisualizer()
            pcd = self._updatePointCloudVisualizer()
            rot, trn = self._updatePoseVisualizer()

            if self._toolBar.recordState and pcd is not None:
                if frameId % self._depthFPS == 0:
                    self._projManager.saveStreamDataWithImages(pcd, rot, trn, color, depth)
                else:
                    self._projManager.saveStreamData(pcd, rot, trn)

    def setProjectName(self, name:str):
        if name == "": return False
        self._layout.projNameLabel.setText(name)
        return True

    def onCreateNewProject(self):
        name = self._projManager.createNewProject()
        self.setProjectName(name)

    def onOpenExistingProject(self):
        name = self._projManager.openExistingProject()
        self.setProjectName(name)
        if self.modelIndex:
            self.onShowMeshModel()
        else:
            self.onShowPointModel()

    def _forceViewMode(self, index):
        if self._layout.stackedVisualizers.currentIndex() != index:
            self._layout.onSwitchLayouts()

    def _startStreaming(self):
        depthCams = self.comboBoxes.serials.count()
        if depthCams == 0:
            self._requestConnectDevice()
            return False
        
        started = self._capturer.startStreaming(
            self._depthRes, self._depthFPS, self._gyroFPS, self._accelFPS,
            self.comboBoxes.serials.currentText()
        )
        if not started: return False

        self._layout.layoutButton.setEnabled(False)
        self._forceViewMode(0)

        self._resetStreamGeometries()
        self.camState.setText(self.camStates[2])
        self.onPlayPauseRecording()
        return True

    def _stopStreaming(self):
        stopped = self._capturer.stopStreaming()
        if not stopped: return False
        
        self._layout.layoutButton.setEnabled(True)
        self._cleanStreamVisualizers()
        self.camState.setText(self.camStates[1])
        return True

    def onStartStopStreaming(self):
        streaming = self._toolBar.streamState
        changed = False
        if not streaming:
            changed = self._startStreaming()
        else:
            changed = self._stopStreaming()
        if changed: streaming = self._toolBar.toggleStream()
        else: return

        if streaming != self._toolBar.feedState:
            self.onPlayPauseRecording()
        
        self.comboBoxes.setEnabled(not streaming)
        self._layout.layoutButton.setEnabled(not streaming)
        self._toolBar.recordAc.setEnabled(streaming)
        self._toolBar.feedAc.setEnabled(streaming)
        self._toolBar.icpAc.setEnabled(not streaming)
        self._toolBar.filterAc.setEnabled(not streaming)
        self._toolBar.meshAc.setEnabled(not streaming)

    def onPlayPauseRecording(self):
        feeding = self._toolBar.toggleFeed()
        if feeding:
            self._captureTimer.start(self._depthPeriod)
            self._visTimer.stop()
        else:
            self._captureTimer.stop()
            self._visTimer.start(self._visPeriod)

    def onStartStopRecording(self):
        if not self._toolBar.recordState:
            if not self._projManager.checkDataExistenceBeforeStreaming(): return
            self._projManager.resetCounters()

        recording = self._toolBar.toggleRecord()
        self._toolBar.streamAc.setEnabled(not recording)
        self._toolBar.openProjAc.setEnabled(not recording)
        self._toolBar.newProjAc.setEnabled(not recording)

    def _requestConnectDevice(self):
        QMessageBox.warning(
            self, "Device Not Found",
            f"Camera {Capturer.deviceName} not found.\n"+
            "Please connect the device and repeat the action."
        )

    def _addDevice(self, device:rs.device):
        deviceInfo = (device.get_info(rs.camera_info.name),
                      device.get_info(rs.camera_info.serial_number))
        self._capturer.connectedDevices.append(device)
        
        if deviceInfo[0].lower() == Capturer.deviceName.lower():
            added = self.comboBoxes.addDeviceSerial(deviceInfo[1])
            if added:
                self.camState.setText(self.camStates[1])

    def _removeDevice(self, device:rs.device):
        deviceInfo = device.get_info(rs.camera_info.serial_number)
        self._capturer.connectedDevices.remove(device)

        if self._toolBar.streamState and device == Capturer.activeDevice:
            Capturer.activeDevice = None
        
        removed = self.comboBoxes.removeDeviceSerial(deviceInfo)
        if removed and self.comboBoxes.serials.count() == 0:
            self.camState.setText(self.camStates[0])
        self.comboBoxes.serials.update()

    def onDevicesChanged(self, info:rs.event_information):
        for device in self._capturer.connectedDevices:
            if info.was_removed(device):
                self._removeDevice(device)

        newDevices = info.get_new_devices()
        for device in newDevices:
            self._addDevice(device)

    def onDepthResolutionChange(self, index:int):
        self._depthRes = Capturer.depthResolutions[index]
        if index == 1: # High res -> Low FPS
            self._depthFPS = Capturer.depthFrameRates[1]
            self._depthPeriod = Capturer.depthCaptureRates[1]
            self.comboBoxes.depthFrameRates.setCurrentIndex(1)

    def onDepthFrameRateChange(self, index:int):
        self._depthFPS = Capturer.depthFrameRates[index]
        self._depthPeriod = Capturer.depthCaptureRates[index]
        if index == 0: # High FPS -> Low res
            self._depthRes = Capturer.depthResolutions[0]
            self.comboBoxes.resolutions.setCurrentIndex(0)

    def onGyroFrameRateChange(self, index:int):
        self._gyroFPS = Capturer.gyroFrameRates[index]

    def onAccelFrameRateChange(self, index:int):
        self._accelFPS = Capturer.accelFrameRates[index]

    def onDistanceThresholdChange(self, value:float):
        self._capturer.setThreshold(value)
        self._resetView = True

    def _resetModelGeometry(self, geometry=None):
        if geometry is None:
            self._modelGeometry = Open3DVisualizer.emptyGeometry
        else:
            self._modelGeometry = geometry
        self._modelScreen.removeAllGeometries()
        self._modelScreen.addGeometry(self._modelGeometry)
        self._modelScreen.setViewControl()

    def _forceModelType(self, index):
        if self.modelIndex != index:
            self.onSwitchModels()

    def onSwitchModels(self):
        index = self._layout.onSwitchModelIcons()
        if index:
            self.onShowMeshModel()
        else:
            self.onShowPointModel()
        self.modelIndex = index

    def onShowPointModel(self):
        pointModel = self._projManager.loadPointModel("point_model_outfit")
        if pointModel is None:
            pointModel = self._projManager.loadPointModel("point_model")
        self._resetModelGeometry(pointModel)

    def onShowMeshModel(self):
        meshModel = self._projManager.loadMeshModel("mesh_model")
        if meshModel is not None:
            meshModel.compute_triangle_normals()
        self._resetModelGeometry(meshModel)

    def _setProcessingMode(self, state):
        self._forceViewMode(1)
        self.spinBoxes.setEnabled(not state)
        self._toolBar.streamAc.setEnabled(not state)
        self._toolBar.openProjAc.setEnabled(not state)
        self._toolBar.newProjAc.setEnabled(not state)
        self._toolBar.icpAc.setEnabled(not state)
        self._toolBar.filterAc.setEnabled(not state)
        self._toolBar.meshAc.setEnabled(not state)
        self._layout.layoutButton.setEnabled(not state)
        self._layout.modelButton.setEnabled(not state)

    def onRunICPAlg(self):
        self._forceModelType(0)
        self._setProcessingMode(True)
        points = self._reconstructor.run_icp(
            app,
            self.spinBoxes.memoryCellSize.value(),
            self.spinBoxes.outlierRadius.value(),
            self.spinBoxes.outlierNeighs.value(),
            self.spinBoxes.skipFitness.value(),
            self.spinBoxes.skipVoxelSize.value(),
            self.spinBoxes.lastLambda.value()
        )
        if points is not None:
            self._resetModelGeometry(points)
        self._setProcessingMode(False)

    def onFilterModel(self):
        self._forceModelType(0)
        self._setProcessingMode(True)
        points = self._reconstructor.remove_outliers(
            app,
            self.spinBoxes.standardRatio.value(),
            self.spinBoxes.statisticNeighs.value(),
            np.array([
                self.spinBoxes.rotationX.value(),
                self.spinBoxes.rotationY.value(),
                self.spinBoxes.rotationZ.value()
            ]),
            np.array([
                self.spinBoxes.minBoundX.value(),
                self.spinBoxes.minBoundY.value(),
                self.spinBoxes.minBoundZ.value()
            ]),
            np.array([
                self.spinBoxes.maxBoundX.value(),
                self.spinBoxes.maxBoundY.value(),
                self.spinBoxes.maxBoundZ.value()
            ])
        )
        self._resetModelGeometry(points)
        self._setProcessingMode(False)
        # self.spinBoxes.memoryCellSize.setEnabled(False)

    def onRunMeshAlg(self):
        self._forceModelType(1)
        self._setProcessingMode(True)
        mesh = self._reconstructor.create_mesh(
            app,
            self.spinBoxes.octalTreeDepth.value(),
            self.spinBoxes.orientNormalsNeighs.value(),
            np.array([
                self.spinBoxes.minBoundX.value(),
                self.spinBoxes.minBoundY.value(),
                self.spinBoxes.minBoundZ.value()
            ]),
            np.array([
                self.spinBoxes.maxBoundX.value(),
                self.spinBoxes.maxBoundY.value(),
                self.spinBoxes.maxBoundZ.value()
            ])
        )
        self._resetModelGeometry(mesh)
        self._setProcessingMode(False)

    def resizeEvent(self, event:QResizeEvent):
        if self._toolBar.streamState:
            self._colorScreen.scaleImage()
            self._depthScreen.scaleImage()
        return super().resizeEvent(event)
    
    def mousePressEvent(self, event:QMouseEvent):
        self.setFocus()
        return super().mousePressEvent(event)
    
    def closeEvent(self, event:QCloseEvent):
        self._motionScreen.vis.destroy_window()
        self._streamCloudScreen.vis.destroy_window()
        
        self._captureTimer.stop()
        self._visTimer.stop()
        if self._toolBar.streamState:
            if self._toolBar.recordState:
                self.onStartStopRecording()
            self._capturer.stopStreaming()
        self._projManager.savePreferences()
        return super().closeEvent(event)


if __name__ == '__main__':
    app = QApplication()
    mainWindow = MainWindow()
    sys.exit(app.exec())
