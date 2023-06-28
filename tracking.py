import open3d as o3d
import numpy as np
import keyboard
import pyrealsense2 as rs

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
pipe.start(cfg)

while not keyboard.is_pressed('Esc'):
    got_frame, frames = pipe.try_wait_for_frames(1000)
    if not got_frame: break

    pose = frames.get_pose_frame()
    if pose:
        data = pose.get_pose_data()  
        translation = data.translation
        rotation = data.rotation # xyzw

        velocity = data.velocity
        angular_velocity = data.angular_velocity

        acceleration = data.acceleration
        angular_acceleration = data.angular_acceleration

        tracker_confidence = data.tracker_confidence # failed:0 - high:3

        print(dir(data))
        break


