import open3d as o3d
import numpy as np
import cv2
from PySide6.QtWidgets import QWidget, QMessageBox, QFileDialog
from PySide6.QtCore import QDir, QObject, QFile, QSaveFile, QIODevice
from PySide6.QtGui import QImageWriter

from projectstartdialog import ProjectStartDialog
from projectcreatedialog import ProjectCreateDialog

class ProjectManager:
    folders = {"data":{"points":{}, "motion":{}, "images":{}}, "model":{}}
    preferences = "bin/preferences.bin"

    def __init__(self, parent:QWidget):
        # Create standard save folder
        self._parent = parent
        self._dataCounter = 1
        self._dir = QDir.current()
        self._imageWriter = QImageWriter()
        self._imageWriter.setFormat(bytes("PNG","utf-8"))
        # print(self._imageWriter.supportedImageFormats())

    def begin(self, path:str):
        if self.openLastProject(): return True
        return self.projectPrompt(path)
    
    def _getLostFolders(self, path, folders):
        projDir = QDir(path)
        lostFolders = []
        for key in folders:
            if not projDir.exists(key):
                lostFolders.append(key)
        return lostFolders
    
    def _getExistingFolders(self, path, folders):
        projDir = QDir(path)
        existingFolders = []
        for key in folders:
            if projDir.exists(key):
                existingFolders.append(key)
        return existingFolders
    
    def _createFolders(self, path, folders):
        if len(folders) == 0: return True

        projDir = QDir(path)
        for folder in folders:
            subFolder = f"{path}/{folder}"
            last = self._createFolders(subFolder, folders[folder])
            if last: 
                projDir.mkpath(subFolder)

        return False

    def createProjectFolders(self, path):
        self._createFolders(path, self.folders)
    
    # Lze vytvorit projekt do jineho projektu
        
    def checkFolderIntegrity(self, path, folders):
        if len(folders) == 0: return []

        existFolders = self._getExistingFolders(path, folders)

        for folder in existFolders:
            existSubFolders = self.checkFolderIntegrity(f"{path}/{folder}", folders[folder])
            if len(existSubFolders) < len(folders[folder]):
                existFolders.remove(folder)
        
        return existFolders
    
    def checkProjectIntegrity(self, path):
        existFolders = self.checkFolderIntegrity(path, self.folders)

        if len(existFolders) == 0:
            QMessageBox.warning(
                self.parent(), "Warning", "The opened project is not a valid project.\n"
                +"Please select a valid project."
            )
            return False
        
        if len(existFolders) < len(self.folders):
            QMessageBox.warning(
                self.parent(), "Warning", "Some data were lost and are no longer inside the project."
            )
            self.createProjectFolders(path)
        
        return True

    def openLastProject(self):
        data = None
        lastFile = QFile(self.preferences, self.parent())
        if lastFile.open(QIODevice.OpenModeFlag.ReadOnly):
            data = lastFile.readAll()
            lastFile.close()
        if data is None: return False

        contents = data.toStdString()
        preferences = contents.splitlines()
        if not self._dir.exists(preferences[0]):
            QMessageBox.warning(
                self.parent(), "Warning", "The last opened project was not found.\n"
                +"Please select project manually."
            )
            return False
        
        if not self.checkProjectIntegrity(preferences[0]): return False

        self._dir.setPath(preferences[0])
        path = preferences[0].rsplit("/",1)
        self.parent().setProjectName(path[1])
        return True

    def projectPrompt(self, path):
        if QDir.isRelativePath(path):
            path = self._dir.absoluteFilePath(path)
        
        if self._dir.mkpath(path):
            freePath = f"{path}/New_Project"
            counter = 0
            while True:
                if not self._dir.exists(freePath): break
                counter += 1
                freePath = f"{path}/New_Project{counter}"

            self._dir.setPath(freePath)
        window = ProjectStartDialog(self, self.parent())
        return window.exec()

    def parent(self) -> QObject:
        return self._parent

    def createNewProject(self):
        path = self._dir.path().rsplit("/",1)
        projWindow = ProjectCreateDialog(self, self.parent(), path[0])
        if projWindow.exec():
            path = projWindow.projLocationEdit.text()
            name = projWindow.projNameEdit.text()
            self._dir.setPath(f"{path}/{name}")
            self.resetCounters()
            return name
        return ""

    def openExistingProject(self):
        path = self._dir.path().rsplit("/",1)
        folderPath = QFileDialog.getExistingDirectory(
            self.parent(), "Open Existing Project", path[0],
            QFileDialog.Option.ShowDirsOnly | QFileDialog.Option.DontResolveSymlinks
        )

        if folderPath == "": return ""
        if not self.checkProjectIntegrity(folderPath): return ""

        self._dir.setPath(folderPath)
        path = folderPath.rsplit("/",1)
        self.resetCounters()
        return path[1]
    
    def removeFolder(self, path):
        projDir = QDir(path)
        if not projDir.removeRecursively():
            QMessageBox.critical(
                self.parent(), "Error", "Some contents in the directory cannot be deleted."
            )
            return False
        return True
    
    def clearFolder(self, path):
        self.removeFolder(path)
        self.createProjectFolders(path)

    def checkDataExistenceBeforeStreaming(self):
        projDir = QDir.current()
        nonEmptyFolders = []
        for folder in self.folders["data"]:
            projDir.setPath(self._dir.path()+f"/data/{folder}")
            if not projDir.isEmpty():
                nonEmptyFolders.append(folder)
        
        if len(nonEmptyFolders) == 0:
            self.resetCounters()
            return True

        warn = QMessageBox(
            QMessageBox.Icon.Warning,
            "Warning",
            "This project already contains recordings.\n"+
            "Would you like to overwrite them?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            self.parent()
        )
        if warn.exec() == QMessageBox.StandardButton.No: return False

        self.resetCounters()
        self.clearFolder(self._dir.path())
        
        return True
    
    def savePointModel(self, pcd, name):
        path = self._dir.path()+f"/model/{name}.ply"
        o3d.io.write_point_cloud(path, pcd)

    def loadPointModel(self, name):
        path = self._dir.path()+f"/model/{name}.ply"
        if not self._dir.exists(path): return None
        return o3d.io.read_point_cloud(path)
    
    def saveMeshModel(self, mesh, name):
        path = self._dir.path()+f"/model/{name}.ply"
        o3d.io.write_triangle_mesh(path, mesh)
        path = self._dir.path()+f"/model/{name}.stl"
        o3d.io.write_triangle_mesh(path, mesh)

    def loadMeshModel(self, name):
        path = self._dir.path()+f"/model/{name}.ply"
        if not self._dir.exists(path): return None
        return o3d.io.read_triangle_mesh(path)
    
    def resetCounters(self):
        self._dataCounter = 1

    def savePointCloud(self, pcd, name):
        path = self._dir.path()+f"/data/points/{name}.ply"
        o3d.io.write_point_cloud(path, pcd)

    def loadPointCloud(self, name):
        path = self._dir.path()+f"/data/points/{name}.ply"
        if not self._dir.exists(path): return None
        return o3d.io.read_point_cloud(path)

    def saveMotionData(self, rot, trn, name):
        path = self._dir.path()+f"/data/motion/{name}.bin"
        data = np.append(rot,trn)
        data.tofile(path)

    def loadMotionData(self, name):
        path = self._dir.path()+f"/data/motion/{name}.bin"
        if not self._dir.exists(path): return None
        data = np.fromfile(path, np.float64)
        rot = data[:9].reshape((3,3))
        trn = data[9:]
        return rot,trn

    def saveImage(self, image, name):
        path = self._dir.path()+f"/data/images/{name}.bmp"
        cv2.imwrite(path, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

    def saveModelPointCloud(self, pcd):
        path = self._dir.path()+f"/model/point_model.ply"
        o3d.io.write_point_cloud(path, pcd)

    def saveStreamDataWithImages(self, pcd, rot, trn, color_image, depth_image):
        index = self._dataCounter
        self.savePointCloud(pcd, f"pcd_{index}")
        self.saveMotionData(rot, trn, f"md_{index}")
        self.saveImage(color_image, f"color_{index}")
        self.saveImage(depth_image, f"depth_{index}")
        self._dataCounter += 1

    def saveStreamData(self, pcd, rot, trn):
        index = self._dataCounter
        self.savePointCloud(pcd, f"pcd_{index}")
        self.saveMotionData(rot, trn, f"md_{index}")
        self._dataCounter += 1

    def savePreferences(self):
        if self._dir.path() == self._dir.currentPath(): return

        saveFile = QSaveFile(self.preferences, self.parent())
        if saveFile.open(QIODevice.OpenModeFlag.WriteOnly):
            saveFile.write(bytes(self._dir.path()+"\n","utf-8"))
            saveFile.commit()
