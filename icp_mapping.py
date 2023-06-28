import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import copy

def rgbd_to_pcd(color_frame, depth_frame):
    color_image = np.array(color_frame.get_data())
    color = o3d.geometry.Image(color_image)    
    
    depth_image = np.array(depth_frame.get_data())
    depth =  o3d.geometry.Image(depth_image)
    
    pcd_tmp = o3d.geometry.PointCloud.create_from_rgbd_image(
        o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity=False),
        o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

    pcd_tmp.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    return pcd_tmp

def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])
    

bag_filename = 'mandarinka_record.bag'
pipeline = rs.pipeline()
config = rs.config()
align = rs.align(rs.stream.color)

rs.config.enable_device_from_file(config, bag_filename, repeat_playback=False)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

pipeline_profile = pipeline.start(config)

frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)
color_frame = aligned_frames.first(rs.stream.color)
depth_frame = aligned_frames.get_depth_frame()
source = rgbd_to_pcd(color_frame, depth_frame)

for _ in range(100):
    frames = pipeline.wait_for_frames()

aligned_frames = align.process(frames)
color_frame = aligned_frames.first(rs.stream.color)
depth_frame = aligned_frames.get_depth_frame()
target = rgbd_to_pcd(color_frame, depth_frame)

current_transformation = np.identity(4)
draw_registration_result_original_color(source, target, current_transformation)

# colored pointcloud registration
voxel_radius = [0.04, 0.02, 0.01]
max_iter = [50, 30, 14]

for scale in range(3):
    iter = max_iter[scale]
    radius = voxel_radius[scale]
    # print([iter, radius, scale])

    # print("3-1. Downsample with a voxel size %.2f" % radius)
    source_down = source.voxel_down_sample(radius)
    target_down = target.voxel_down_sample(radius)

    # print("3-2. Estimate normal.")
    source_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
    target_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

    # print("3-3. Applying colored point cloud registration")
    result_icp = o3d.pipelines.registration.registration_colored_icp(
        source_down, target_down, radius, current_transformation,
        o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                          relative_rmse=1e-6,
                                                          max_iteration=iter))
    current_transformation = result_icp.transformation
    print(result_icp)

draw_registration_result_original_color(source, target,
                                        result_icp.transformation)
