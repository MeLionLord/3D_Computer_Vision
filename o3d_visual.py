import open3d as o3d
import numpy as np
import keyboard

def rgbd_to_pcd(rgbd_image):
    color_image = np.array(rgbd_image.color)
    color = o3d.geometry.Image(color_image)    
    
    depth_image = np.array(rgbd_image.depth)
    depth =  o3d.geometry.Image(depth_image)
    
    pcd_tmp = o3d.geometry.PointCloud.create_from_rgbd_image(
        o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity=False),
        o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

    pcd_tmp.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    return pcd_tmp

# Open record from bag file
bag_filename = 'mandarinka_record.bag'
bag_reader = o3d.t.io.RSBagReader.create(bag_filename)

# Create first frame
im_rgbd = bag_reader.next_frame()
pcd = rgbd_to_pcd(im_rgbd)

# Initialize Visualizer and start animation callback
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='Stream', left=0, top=35)
vis.add_geometry(pcd)

try:
    while not bag_reader.is_eof():
        im_rgbd = bag_reader.next_frame()
        if not im_rgbd: continue

        pcd_new = rgbd_to_pcd(im_rgbd)
        pcd.points = pcd_new.points
        pcd.colors = pcd_new.colors

        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

        if keyboard.is_pressed('Esc'):
            break

except Exception as e:
    print(e)

print('Window closed')
vis.destroy_window()
bag_reader.close()
