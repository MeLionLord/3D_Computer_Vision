import numpy as np
import open3d as o3d
from PySide6.QtCore import QDir
import keyboard
from open3dvisualizer import Open3DVisualizer
from projectmanager import ProjectManager

class Reconstructor:
    def __init__(self, projectManager:ProjectManager, visualizer:Open3DVisualizer):
        self.screen = visualizer
        self.projMgr = projectManager

    def changeVisualizer(self, visualizer:Open3DVisualizer):
        self.screen = visualizer
    
    def _colored_icp(self, source, moving, transformation, voxel_radius, max_iters, max_nn=20, fitness=1e-6, rmse=1e-6, lambda_geometric=0.98):
        source_down = source.voxel_down_sample(voxel_radius)
        moving_down = moving.voxel_down_sample(voxel_radius)

        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_radius * 2, max_nn=max_nn))
        moving_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_radius * 2, max_nn=max_nn))

        result_icp = o3d.pipelines.registration.registration_colored_icp(
            moving_down, source_down, voxel_radius, transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(
                lambda_geometric=lambda_geometric
            ),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=fitness, relative_rmse=rmse, max_iteration=max_iters
            )
        )
        return result_icp

    def _copy_to_point_cloud(self, source, target):
        if len(source.points) == 0: return
        target.points = source.points
        target.colors = source.colors
        target.normals = source.normals
        target.covariances = source.covariances

    def _duplicate_point_cloud(self, source):
        pcdCopy = o3d.geometry.PointCloud()
        self._copy_to_point_cloud(source, pcdCopy)
        return pcdCopy

    def _merge_two_point_clouds(self, first, second):
        if len(second.points) == 0: return
        first.points.extend(second.points)
        first.colors.extend(second.colors)
        first.normals.extend(second.normals)
        first.covariances.extend(second.covariances)

    def _merge_point_clouds(self, pcds):
        pcd = self._duplicate_point_cloud(pcds[0])
        for i in range(1,len(pcds)):
            self._merge_two_point_clouds(pcd,pcds[i])
        return pcd

    def _icp_sequence(self, source, moving, transf, outRadius, outNeighs, skipFit, skipSize, lastLambda):
        self._copy_to_point_cloud(moving.voxel_down_sample(0.001), moving)
        if outNeighs > 0 and outRadius > 0:
            pcd, idx = moving.remove_radius_outlier(nb_points=outNeighs, radius=outRadius)
            self._copy_to_point_cloud(pcd, moving)

        if skipSize > 0:
            result = self._colored_icp(source, moving, transf, skipSize, 1)
            if result.fitness > skipFit: return None
            transf = result.transformation

        result = self._colored_icp(source, moving, transf, 0.005, 1, lambda_geometric=1)
        transf = result.transformation
        result = self._colored_icp(source, moving, transf, 0.003, 3, lambda_geometric=1)
        transf = result.transformation
        result = self._colored_icp(source, moving, transf, 0.002, 1, lambda_geometric=lastLambda)
        transf = result.transformation
        
        return transf

    def _integration(self, source, moving, transf):
        moving.transform(transf)
        self._merge_two_point_clouds(source, moving)

    def _registration(self, source, moving, transf, outRadius, outNeighs, skipFit, skipSize, lastLambda):
        transf_new = self._icp_sequence(
            source, moving, transf, outRadius, outNeighs,
            skipFit, skipSize, lastLambda
        )
        if transf_new is None: return None

        return {"points":moving, "transformation":transf_new}

    def run_icp(self, app, memory, outRadius, outNeighs, skipFit, skipSize, lastLambda):
        index = 1
        transf = np.eye(4, dtype=np.float64)
        visul = True
        roll = 0
        
        geometry = self.projMgr.loadPointCloud(f"pcd_{index}")
        if geometry is None: return None
        geometry = geometry.voxel_down_sample(0.001)
        
        past_points = [None]*memory
        past_points[roll] = geometry
        for i in range(1,memory):
            past_points[i] = o3d.geometry.PointCloud()

        self.screen.removeAllGeometries()
        self.screen.addGeometry(geometry)
        self.screen.setViewControl()
        self.screen.updateVis()
        app.processEvents()

        while visul:
            index += 1
            moving = self.projMgr.loadPointCloud(f"pcd_{index}")
            if moving is None: break
            result = self._registration(
                self._merge_point_clouds(past_points), moving, transf,
                outRadius, outNeighs, skipFit, skipSize, lastLambda
            )
            if result is not None:
                transf = result["transformation"]
                self._integration(geometry, result["points"], transf)
                roll = (roll+1) % memory
                past_points[roll] = result["points"]

            self.screen.updateGeometry(geometry)
            self.screen.updateVis()
            visul = not keyboard.is_pressed("esc")
            app.processEvents()

        if visul:
            self._copy_to_point_cloud(geometry.voxel_down_sample(0.0005), geometry)
            geometry.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=30))
            self.projMgr.savePointModel(geometry, "point_model")
            return geometry
        
        return None
    
    def crop_points(self, geometry, angles, minBound, maxBound):
        geometry.translate(-geometry.get_center())
        angles = angles/180*np.pi
        R = geometry.get_rotation_matrix_from_xyz(angles)
        geometry.rotate(R, center=(0, 0, 0))

        bbox = o3d.geometry.AxisAlignedBoundingBox()
        bbox.min_bound = minBound
        bbox.max_bound = maxBound
        return geometry.crop(bbox)

    def remove_outliers(self, app, ratio, neighs, angles, minBound, maxBound):
        geometry = self.projMgr.loadPointModel("point_model")
        if geometry is None: return None

        geometry = self.crop_points(geometry, angles, minBound, maxBound)
        app.processEvents()

        geometry, ind = geometry.remove_statistical_outlier(nb_neighbors=neighs, std_ratio=ratio)
        geometry.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=30))

        self.projMgr.savePointModel(geometry, "point_model_outfit")
        return geometry

    def crop_mesh(self, mesh, minBound, maxBound):
        bbox = o3d.geometry.AxisAlignedBoundingBox()
        bbox.min_bound = minBound
        bbox.max_bound = maxBound
        return mesh.crop(bbox)

    def create_mesh(self, app, depth, neighs, minBound, maxBound):
        geometry = self.projMgr.loadPointModel("point_model_outfit")
        if geometry is None: return None
        app.processEvents()

        geometry.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=30))
        geometry.orient_normals_consistent_tangent_plane(neighs)
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            geometry, depth=depth)
        # vertices_to_remove = densities < np.quantile(densities, 0.01)
        # mesh.remove_vertices_by_mask(vertices_to_remove)

        mesh = self.crop_mesh(mesh, minBound, maxBound)
    
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        mesh.remove_non_manifold_edges()
        mesh.remove_unreferenced_vertices()
        mesh.compute_vertex_normals()

        self.projMgr.saveMeshModel(mesh, "mesh_model")
        return mesh
