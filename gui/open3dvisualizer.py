from PySide6.QtWidgets import QWidget, QSizePolicy
from PySide6.QtGui import QWindow
from win32gui import FindWindow
import open3d as o3d

class Open3DVisualizer(QWidget):

    emptyPionts = o3d.utility.Vector3dVector(
        [[-5,.0,.0], [-5,.0,1], [-5,.0,2], [-4,.0,.0], [-4,.0,1], [-4,.0,2],
         [-3.5,.0,.0], [-3.5,.0,2], [-2.5,.0,.0], [-1.5,.0,2], [-1.5,.0,.0],
         [-1,.0,.0], [-1,.0,1], [-1,.0,2], [.0,.0,2], [.0,.0,1],
         [0.5,.0,2], [1.5,.0,2], [2.5,.0,2], [1.5,.0,.0],
         [3,.0,2], [4,.0,1], [5,.0,2], [4,.0,.0]]
    )
    emptyLines = o3d.utility.Vector2iVector(
        [[0,2], [3,0], [4,1], [5,2],
         [6,7], [7,8], [8,9], [9,10],
         [11,13], [13,14], [14,15], [15,12],
         [16,18], [17,19],
         [20,21], [21,22], [21,23]]
    )
    emptyGeometry = o3d.geometry.LineSet(emptyPionts, emptyLines)

    coordPoints = o3d.utility.Vector3dVector([[0.1,0,0],[0,0.1,0],[0,0,0.1],[0,0,0]])
    coordLines = o3d.utility.Vector2iVector([[0,3],[1,3],[2,3]])
    coordColors = o3d.utility.Vector3dVector([[1,0,0],[0,1,0],[0,0,1]])
    coordsGeometry = o3d.geometry.LineSet(coordPoints, coordLines)
    coordsGeometry.colors = coordColors

    def __init__(self, parent:QWidget, resolution=(320,240), geometry=None, window_name="Open3D"):
        super().__init__(parent)

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name, resolution[0], resolution[1])
        if geometry:
            self.activeGeometries = [geometry]
        else:
            self.activeGeometries = [Open3DVisualizer.emptyGeometry]
        self.vis.add_geometry(self.activeGeometries[0])
        self.setViewControl()
        
        hwnd = FindWindow(None, window_name)
        self.window = QWindow.fromWinId(hwnd)
        self.windowContainer = self.createWindowContainer(self.window, self)
        self.windowContainer.setMinimumSize(resolution[0], resolution[1])
        self.windowContainer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def setViewControl(self):
        view_control = self.vis.get_view_control()
        view_control.set_lookat([0,0,0])
        view_control.set_front([0,-1,0])
        view_control.set_up([0,0,1])
        view_control.set_zoom(0.6)
        render_option = self.vis.get_render_option()
        render_option.mesh_show_back_face = True

    def resetView(self):
        for geometry in self.activeGeometries:
            self.vis.remove_geometry(geometry)
        for geometry in self.activeGeometries:
            self.vis.add_geometry(geometry)
        self.setViewControl()

    def updateVis(self):
        self.vis.poll_events()
        self.vis.update_renderer()

    def updateGeometry(self, geometry:o3d.geometry):
        self.vis.update_geometry(geometry)

    def updateGeometries(self):
        for geometry in self.activeGeometries:
            self.vis.update_geometry(geometry)

    def addGeometry(self, geometry:o3d.geometry):
        self.activeGeometries.append(geometry)
        self.vis.add_geometry(geometry)
        self.setViewControl()

    def removeGeometry(self, geometry:o3d.geometry):
        self.vis.remove_geometry(geometry)
        self.activeGeometries.remove(geometry)

    def removeAllGeometries(self):
        for geometry in self.activeGeometries:
            self.vis.remove_geometry(geometry)
        self.activeGeometries.clear()

    def createPlaceHolderPoints(radius:float):
        r = radius
        placeHolderPoints = o3d.utility.Vector3dVector(
            [[r,.0,.0], [-r,.0,.0],
             [.0,r,.0], [.0,-r,.0],
             [.0,.0,r], [.0,.0,-r]]
        )
        return placeHolderPoints
