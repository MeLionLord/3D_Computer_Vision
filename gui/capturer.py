import pyrealsense2 as rs
import numpy as np
import open3d as o3d
from ahrs.filters import Madgwick

class Capturer:
    connectedDevices = []
    activeDevice = None
    deviceName = "Intel RealSense D435i"
    
    depthResolutions = [(640,480), (1280,720)]
    depthFrameRates = [15, 6]
    accelFrameRates = [63, 250]
    gyroFrameRates = [200, 400]
    depthCaptureRates = [30, 80]
    # accelCaptureRates = [8, 2]
    # gyroCaptureRates = [2, 1]

    # depth:     # color:        # motion:  # timestamp:
    # (480, 640) # (480, 640, 3) # (3)      # (1)
    # uint16     # uint8         # float64  # float64

    def __init__(self, threshold:float, device_changed_callback):
        self._align = rs.align(rs.stream.color)
        self.colorizer = rs.colorizer(0) # 0=JET
        self.context = rs.context()
        self.context.set_devices_changed_callback(device_changed_callback)
        self._depthPipeline = rs.pipeline()
        self._gyroPipeline = rs.pipeline()
        self._accelPipeline = rs.pipeline()
        self._depthConfig = rs.config()
        self._gyroConfig = rs.config()
        self._accelConfig = rs.config()

        self._filter = None
        self._translation = np.zeros(3, dtype=np.float64)
        self._rotQ = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self._gyroData = np.zeros(3, dtype=np.float64)
        self._accelData = np.zeros(3, dtype=np.float64)
        self._updateOnGyro = True

        self._threshold = threshold

        # self._gyroTimestamp = .0
        # self._accelTimestamp = .0
        self._frameId = -1
        self._colorFrame = None
        self._depthFrame = None
        self._colorImage = None
        self._depthImage = None
        self._pointCloud = None

    def startStreaming(self, resolution:tuple, depth_fps:int, gyro_fps:int,
                       accel_fps:int, serial:rs.camera_info.serial_number):
        serialOK = False
        for device in self.connectedDevices:
            if serial == device.get_info(rs.camera_info.serial_number):
                serialOK = True
                break
        if not serialOK: return False

        self._depthConfig.enable_device(serial)
        self._depthConfig.enable_stream(
            rs.stream.depth, resolution[0], resolution[1], rs.format.z16, depth_fps
        )
        self._depthConfig.enable_stream(
            rs.stream.color, resolution[0], resolution[1], rs.format.rgb8, depth_fps
        )
        self._gyroConfig.enable_device(serial)
        self._gyroConfig.enable_stream(
            rs.stream.gyro, rs.format.motion_xyz32f, gyro_fps
        )
        self._accelConfig.enable_device(serial)
        self._accelConfig.enable_stream(
            rs.stream.accel, rs.format.motion_xyz32f, accel_fps
        )

        try:
            self._accelPipeline.start(self._accelConfig, self._accelHandle)
            self._gyroPipeline.start(self._gyroConfig, self._gyroHandle)
            self._depthPipeline.start(self._depthConfig, self._depthHandle)
        except Exception as ex:
            print(ex)
            return False
    
        self._filter = Madgwick(frequency=max(depth_fps, gyro_fps))
        self._updateOnGyro = gyro_fps > depth_fps
        Capturer.activeDevice = device
        return True

    def stopStreaming(self):
        self._filter = None
        Capturer.activeDevice = None
        try:
            self._accelPipeline.stop()
            self._gyroPipeline.stop()
            self._depthPipeline.stop()
        except Exception as ex:
            print(ex)
            return False
        
        return True
    
    def _depthHandle(self, frame:rs.frame):
        if Capturer.activeDevice == None: return

        alignedFrames = self._align.process(rs.composite_frame(frame))
        self._colorFrame = alignedFrames.get_color_frame()
        self._depthFrame = alignedFrames.get_depth_frame()
        depthColoredFrame = self.colorizer.colorize(self._depthFrame)

        self._colorImage = np.array(self._colorFrame.get_data())
        self._depthImage = np.array(depthColoredFrame.get_data())
        
        self._frameId = self._depthFrame.get_frame_number()
    
    def _gyroHandle(self, frame:rs.frame):
        # timestamp = frame.get_timestamp()
        # if self._gyroTimestamp == 0.0:
        #     self._gyroTimestamp = timestamp
        # dt = (timestamp - self._gyroTimestamp)/1000
        # self._gyroTimestamp = timestamp

        data = frame.as_motion_frame().get_motion_data()
        self._gyroData = np.array([-data.z, -data.x, -data.y], dtype=np.float64)
        if self._updateOnGyro:
            self._rotQ = self._filter.updateIMU(self._rotQ, self._gyroData, self._accelData)

    def _accelHandle(self, frame:rs.frame):
        # timestamp = frame.get_timestamp()
        # if self._accelTimestamp == 0.0:
        #     self._accelTimestamp = timestamp
        # dt = (timestamp - self._accelTimestamp)/1000
        # self._accelTimestamp = timestamp

        data = frame.as_motion_frame().get_motion_data()
        self._accelData = np.array([-data.z, data.x, data.y], dtype=np.float64)
        if not self._updateOnGyro:
            self._rotQ = self._filter.updateIMU(self._rotQ, self._gyroData, self._accelData)

    def updatePointCloud(self) -> o3d.geometry:
        if self._colorFrame is None or self._depthFrame is None: return

        depthData = np.array(self._depthFrame.get_data())
        colorImage = o3d.geometry.Image(self._colorImage)
        depthImage =  o3d.geometry.Image(depthData)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            o3d.geometry.RGBDImage.create_from_color_and_depth(
                colorImage, depthImage, depth_scale=1000.0,
                depth_trunc=self._threshold, convert_rgb_to_intensity=False
            ),
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            )
        )
        pcd.transform([[ 1, 0, 0, 0],
                       [ 0, 0, 1, 0],
                       [ 0,-1, 0, 0],
                       [ 0, 0, 0, 1]])
        self._pointCloud = pcd
        return pcd

    def getFrameId(self):
        return self._frameId

    def getColorDepthImage(self):
        return self._colorImage, self._depthImage
    
    def getPointCloud(self):
        return self._pointCloud
    
    def getPose(self):
        rotation = np.array(o3d.geometry.get_rotation_matrix_from_quaternion(self._rotQ), np.float64)
        rotation = np.array([rotation[:,1]*-1,rotation[:,0]*-1,rotation[:,2]], np.float64)
        return rotation, self._translation
    
    def setThreshold(self, threshold:float):
        self._threshold = threshold
