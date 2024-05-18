from PySide6.QtWidgets import QWidget, QToolBar
from PySide6.QtGui import QPixmap

class ToolBar(QToolBar):
    # Color: #3B82F6
    # Transparency: 5C
    def __init__(self, parent:QWidget):
        super().__init__(parent)
        iconFolder = "icons"

        newFileIcon = QPixmap(f"{iconFolder}/new-folder.png")
        self.newProjAc = self.addAction(
            newFileIcon, "&New Project", "Ctrl+N", parent.onCreateNewProject
        )

        openFileIcon = QPixmap(f"{iconFolder}/folder.png")
        self.openProjAc = self.addAction(
            openFileIcon, "&Open Project", "Ctrl+M", parent.onOpenExistingProject
        )

        self.addSeparator()

        self.streamPixmap = [QPixmap(f"{iconFolder}/camera.png"),
                             QPixmap(f"{iconFolder}/no-video.png")]
        self.streamState = False
        self.streamAc = self.addAction(
            self.streamPixmap[0], "Start Camera Stream", "Ctrl+I"
        )
        self.streamAc.triggered.connect(parent.onStartStopStreaming)
        
        self.feedPixmap = [QPixmap(f"{iconFolder}/play.png"),
                           QPixmap(f"{iconFolder}/pause.png")]
        self.feedState = False
        self.feedAc = self.addAction(
            self.feedPixmap[0], "Pause Recording", "P"
        )
        self.feedAc.triggered.connect(parent.onPlayPauseRecording)
        self.feedAc.setEnabled(False)

        self.recordPixmap = [QPixmap(f"{iconFolder}/circle.png"),
                             QPixmap(f"{iconFolder}/square.png")]
        self.recordState = False
        self.recordAc = self.addAction(
            self.recordPixmap[0], "Start Recording", "Ctrl+O"
        )
        self.recordAc.triggered.connect(parent.onStartStopRecording)
        self.recordAc.setEnabled(False)

        self.addSeparator()
        
        icpIcon = QPixmap(f"{iconFolder}/cube-connect.png")
        self.icpAc = self.addAction(
            icpIcon, "Run ICP Algorithm", "Ctrl+E", parent.onRunICPAlg
        )

        filterIcon = QPixmap(f"{iconFolder}/cube-remove.png")
        self.filterAc = self.addAction(
            filterIcon, "Remove Outliers", "Ctrl+R", parent.onFilterModel
        )

        meshIcon = QPixmap(f"{iconFolder}/layers.png")
        self.meshAc = self.addAction(
            meshIcon, "Run Mesh Algorithm", "Ctrl+T", parent.onRunMeshAlg
        )
        
        # cropIcon = QPixmap(f"{iconFolder}/crop.png")
        # self.cropAc = self.addAction(
        #     cropIcon, "Crop Current Model", "Ctrl+U", parent.onCropModel
        # )

    def toggleStream(self):
        self.streamState = not self.streamState
        self.streamAc.setIcon(self.streamPixmap[self.streamState])
        return self.streamState

    def toggleFeed(self):
        self.feedState = not self.feedState
        self.feedAc.setIcon(self.feedPixmap[self.feedState])
        return self.feedState

    def toggleRecord(self):
        self.recordState = not self.recordState
        self.recordAc.setIcon(self.recordPixmap[self.recordState])
        return self.recordState
