import open3d as o3d
import numpy as np
import keyboard
import pyrealsense2 as rs

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

def update_frame(frames):
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.first(rs.stream.depth)
    color_frame = aligned_frames.get_color_frame()
    
    pcd_new = rgbd_to_pcd(color_frame, depth_frame)
    pcd.points = pcd_new.points
    pcd.colors = pcd_new.colors

    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()


bag_filename = 'mandarinka_record.bag'
FROM_BAG = True

pipeline = rs.pipeline()
config = rs.config()
align = rs.align(rs.stream.color)

if FROM_BAG:
    rs.config.enable_device_from_file(config, bag_filename, repeat_playback=False)

config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

pipeline_profile = pipeline.start(config)

if FROM_BAG:
    playback = pipeline_profile.get_device().as_playback()
    playback.set_real_time(False)

# First frame to build visualizer
frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)
color_frame = aligned_frames.first(rs.stream.color)
depth_frame = aligned_frames.get_depth_frame()
pcd = rgbd_to_pcd(color_frame, depth_frame)

# Initialize Visualizer
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='Stream', width=1280, height=720)
vis.add_geometry(pcd)

got_frame = False


while vis.poll_events():
    got_frame, frames = pipeline.try_wait_for_frames(1000)
    if not got_frame: break

    update_frame(frames)
    
vis.destroy_window()
pipeline.stop()
