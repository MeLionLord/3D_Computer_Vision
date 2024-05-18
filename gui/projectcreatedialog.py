from PySide6.QtWidgets import(
    QDialog, QWidget, QDialogButtonBox, QPushButton, QMessageBox,
    QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QFileDialog
)
from PySide6.QtCore import Qt, QDir

class ProjectCreateDialog(QDialog):
    def __init__(self, manager, parent:QWidget, path:str):
        super().__init__(parent)
        self.setWindowTitle("Create New Project")
        self.setMinimumSize(320, 180)
        self.manager = manager
        
        createButton = QPushButton("Create", self)
        browseButton = QPushButton("...", self)
        browseButton.setMaximumSize(30,20)
        browseButton.clicked.connect(self.onOpenFolderBrowser)

        self._dialogButtons = QDialogButtonBox(Qt.Orientation.Horizontal, self)
        self._dialogButtons.addButton(createButton, QDialogButtonBox.ButtonRole.AcceptRole)
        self._dialogButtons.addButton(QDialogButtonBox.StandardButton.Cancel)

        layout = QVBoxLayout()
        browseLayout = QHBoxLayout()

        projNameLabel = QLabel("Project Name", self)
        projLocationLabel = QLabel("Project Location", self)

        freeName = "New_Project"
        counter = 0
        projDir = QDir.current()
        while True:
            if not projDir.exists(f"{path}/{freeName}"): break
            counter += 1
            freeName = f"New_Project{counter}"

        self.projNameEdit = QLineEdit(freeName, self)
        self.projNameEdit.setFrame(False)
        self.projLocationEdit = QLineEdit("", self)
        self.projLocationEdit.setText(path)
        self.projLocationEdit.setFrame(False)

        browseLayout.addWidget(self.projLocationEdit)
        browseLayout.addWidget(browseButton)

        layout.addWidget(projNameLabel)
        layout.addWidget(self.projNameEdit)
        layout.addWidget(projLocationLabel)
        layout.addLayout(browseLayout)
        layout.addStretch()
        layout.addWidget(self._dialogButtons)

        self.setLayout(layout)

        self._dialogButtons.accepted.connect(self.createProject)
        self._dialogButtons.rejected.connect(self.reject)

    def onOpenFolderBrowser(self):
        folderLocation = QFileDialog.getExistingDirectory(
            self, "Choose Project Location", self.projLocationEdit.text(),
            QFileDialog.Option.ShowDirsOnly | QFileDialog.Option.DontResolveSymlinks
        )
        if folderLocation != "":
            self.projLocationEdit.setText(folderLocation)

    def createProject(self):
        path = self.projLocationEdit.text()
        name = self.projNameEdit.text()
        fullPath = f"{path}/{name}"
        projDir = QDir.current()

        if len(self.manager.checkFolderIntegrity(path, self.manager.folders)) > 0:
            QMessageBox.warning(
                self.parent(), "Warning", "Cannot create project inside another project."
            )
            return

        if not projDir.mkpath(fullPath):
            QMessageBox.critical(
                self.parent(), "Error", "Directory cannot be created here."
            )
            return
        
        projDir.setPath(fullPath)
        if not projDir.isEmpty():
            if not self.manager.removeFolder(fullPath): return
        
        self.manager.createProjectFolders(fullPath)
        self.accept()

    def warnOverwriteDirectory(self):
        warn = QMessageBox(
            QMessageBox.Icon.Warning,
            "Warning",
            "Directory with this name already exists!\n"+
            "Would you like to overwrtie the contents of this folder?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            self.parent()
        )
        if warn.exec() == QMessageBox.StandardButton.No: return False
        return True
