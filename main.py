import sys
import open3d as o3d
import numpy as np
import pyrealsense2 as rs


def frame_to_pcd(frame):
    aligned_frames = align.process(frame)
    # depth_frame = aligned_frames.first(rs.stream.depth)
    color_frame = aligned_frames.get_color_frame()
    # color_frame = aligned_frames.first(rs.stream.color)
    depth_frame = aligned_frames.get_depth_frame()

    color_data = np.array(color_frame.get_data())
    color = o3d.geometry.Image(color_data)    
    
    depth_data = np.array(depth_frame.get_data())
    depth =  o3d.geometry.Image(depth_data)
    
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity=False),
        o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    return pcd

def pose_handler(pose, time, prev_time):
    pose_frame = pose.get_pose_frame()
    pose_data = pose_frame.get_pose_data()
    tracker_confidence = pose_data.tracker_confidence # failed:0 - high:3
    if tracker_confidence < 3: return

    # translation = pose_data.translation
    # rotation = pose_data.rotation # xyzw

    velocity = pose_data.velocity
    angular_velocity = pose_data.angular_velocity

    # print('Translation:',translation)
    # print('Rotation',rotation)
    delta_time = time-prev_time
    translation = np.array([velocity.x,velocity.y,velocity.z])/delta_time
    rotation = np.array([angular_velocity.x,angular_velocity.y,angular_velocity.z])/delta_time
    # print('Appending translation','x:',translation[0],'y:',translation[1],'z:',translation[2])
    # print('Appending rotation','x:',rotation[0],'y:',rotation[1],'z:',rotation[2])
    return [translation,rotation]

def transform(transformation,translation,rotation):
    transformation *= np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[translation[0],translation[1],translation[2],1]])
    transformation *= np.array([[np.cos(rotation[2]),-np.sin(rotation[2]),0,0],[np.sin(rotation[2]),np.cos(rotation[2]),0,0],[0,0,1,0],[0,0,0,1]])
    transformation *= np.array([[np.cos(rotation[1]),0,np.sin(rotation[1]),0],[0,1,0,0],[-np.sin(rotation[1]),0,np.cos(rotation[1]),0],[0,0,0,1]])
    transformation *= np.array([[1,0,0,0],[0,np.cos(rotation[0]),-np.sin(rotation[0]),0],[0,np.sin(rotation[0]),np.cos(rotation[0]),0],[0,0,0,1]])
    return transformation

def colored_icp(source, target, transformation):
    # Colored pointcloud registration
    voxel_radius = [0.01, 0.005]
    max_iters = [50, 30]
    for scale in range(len(voxel_radius)):
        iters = max_iters[scale]
        radius = voxel_radius[scale]

        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, transformation, o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=iters))
        
    if result_icp.inlier_rmse < 0.005: return np.identity(4)
    return result_icp.transformation

    
# Constants
RGBD_BAGFILE = 'mandarinka_record.bag'
TRACK_BAGFILE = 'tracking_record.bag'
FROM_BAG = True
FRAME_GAP = 10
FRAME_WAIT = 1000


# Pipelines
pipeline_rgbd = rs.pipeline()
config_rgbd = rs.config()
pipeline_track = rs.pipeline()
config_track = rs.config()
align = rs.align(rs.stream.color)

if FROM_BAG:
    rs.config.enable_device_from_file(config_rgbd, RGBD_BAGFILE, repeat_playback=False)
    rs.config.enable_device_from_file(config_track, TRACK_BAGFILE, repeat_playback=False)

config_rgbd.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config_rgbd.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
config_track.enable_stream(rs.stream.pose, 848, 800, rs.format.six_dof, 200)

pipeline_profile_rgbd = pipeline_rgbd.start(config_rgbd)
pipeline_profile_track = pipeline_track.start(config_track)

if FROM_BAG:
    playback = pipeline_profile_rgbd.get_device().as_playback()
    playback.set_real_time(False)
    playback = pipeline_profile_track.get_device().as_playback()
    playback.set_real_time(False)


# First frames
scene = pipeline_rgbd.wait_for_frames()
source = frame_to_pcd(scene)
source.points.extend(source.points)
source.colors.extend(source.colors)

pose = pipeline_track.wait_for_frames()
pose_frame = pose.get_pose_frame()
if pose_frame: pose_data = pose_frame.get_pose_data()


# Initialize Visualizer
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='Stream', width=1280, height=720)
vis.add_geometry(source)
vis.update_geometry(source)
visiualize = vis.poll_events()
vis.update_renderer()

frame_counter = 0
pose_time = pose.get_timestamp()
scene_time = scene.get_timestamp()
transformation = np.identity(4)
translation = np.zeros(3)
rotation = np.zeros(3)
found_best_frame = False
prev = {'pose_time':pose_time,'frames_time_diff':sys.float_info.max,
        'translation':np.zeros(3),'rotation':np.zeros(3)}

while visiualize:
# for _ in range(20):
    got_scene, scene = pipeline_rgbd.try_wait_for_frames(FRAME_WAIT)
    if not got_scene: continue
    scene_time = scene.get_timestamp()

    while not found_best_frame:
        got_pose, pose = pipeline_track.try_wait_for_frames(FRAME_WAIT)
        if not got_pose: continue
        pose_time = pose.get_timestamp()

        frames_time_diff = abs(scene_time-pose_time)
        if prev['frames_time_diff'] < frames_time_diff:
            found_best_frame = True
            prev['translation'] = translation
            prev['rotation'] = rotation

        [translation,rotation] = pose_handler(pose, pose_time, prev['pose_time'])
        prev['pose_time'] = pose_time
        prev['frames_time_diff'] = frames_time_diff

    prev['frames_time_diff'] = sys.float_info.max
    frame_counter += 1
    if frame_counter <= FRAME_GAP: continue
    frame_counter = 0

    transformation = transform(transformation,prev['translation'],prev['rotation'])
    target = frame_to_pcd(scene)
    transformation = colored_icp(source, target, transformation)
    source.transform(transformation)

    transformation = np.identity(4)
    prev['translation'] = np.zeros(3)
    prev['rotation'] = np.zeros(3)

    source.points.extend(target.points)
    source.colors.extend(target.colors)

    # source = source.voxel_down_sample(0.0001) # No render update !!!
    # print(len(source.points))

    vis.update_geometry(source)
    visiualize = vis.poll_events()
    vis.update_renderer()

print("END")
# Hold visualizer on last frame
while visiualize:
    visiualize = vis.poll_events()

vis.destroy_window()
pipeline_rgbd.stop()
pipeline_track.stop()
