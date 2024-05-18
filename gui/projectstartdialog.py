from PySide6.QtWidgets import(
    QDialog, QWidget, QDialogButtonBox, QPushButton, QMessageBox,
    QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QFileDialog
)
from PySide6.QtCore import Qt, QDir

class ProjectStartDialog(QDialog):
    def __init__(self, manager, parent:QWidget):
        super().__init__(parent)
        self.setWindowTitle("Choose a Project")
        self.setMinimumSize(320, 100)
        self.setWindowFlag(Qt.WindowType.WindowMinimizeButtonHint)
        self.manager = manager

        createButton = QPushButton("Create", self)
        createButton.clicked.connect(self.createNewProject)
        openButton = QPushButton("Open", self)
        openButton.clicked.connect(self.openExistingProject)

        self._dialogButtons = QDialogButtonBox(Qt.Orientation.Horizontal, self)
        self._dialogButtons.addButton(createButton, QDialogButtonBox.ButtonRole.ActionRole)
        self._dialogButtons.addButton(openButton, QDialogButtonBox.ButtonRole.ApplyRole)

        layout = QVBoxLayout()
        infoLabel = QLabel("Open an exsiting project or create a new one!", self)

        layout.addWidget(infoLabel)
        layout.addWidget(self._dialogButtons)
        self.setLayout(layout)

    def createNewProject(self):
        name = self.manager.createNewProject()
        if self.parent().setProjectName(name):
            self.accept()

    def openExistingProject(self):
        name = self.manager.openExistingProject()
        if self.parent().setProjectName(name):
            self.accept()
