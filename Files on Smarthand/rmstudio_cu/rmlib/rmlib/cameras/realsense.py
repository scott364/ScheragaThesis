import pyrealsense2 as rs
import numpy as np
import time
from rmlib.rmtools import poses as pmath

class RealSense:
    def __init__(self, realsense_config, _):
        # TCP offsets
        self.ci_cam_offset = realsense_config['ci_cam_offset']
        self.pc_cam_offset = realsense_config['pc_cam_offset']
        self.tcp_to_ci_cam_pose = pmath.translate_pose(np.eye(4), [self.ci_cam_offset[0], self.ci_cam_offset[1], self.ci_cam_offset[2]])
        self.tcp_to_pc_cam_pose = pmath.translate_pose(np.eye(4), [self.pc_cam_offset[0], self.pc_cam_offset[1], self.pc_cam_offset[2]])
        
        self.model = realsense_config['camera_model']
        frame_rate = 30
        visual_preset = 3
        self.camera_controller = None
        self.pc = rs.pointcloud()
        self.pipe = rs.pipeline()
        self.camera_set_time = time.time()
        self.device = self.find_device_that_supports_advanced_mode()
        self.depth_sensor = self.device.first_depth_sensor()
        self.camera_controller = rs.rs400_advanced_mode(self.device)

        depth_table = self.camera_controller.get_depth_table()
        depth_table.depthUnits = 100
        self.camera_controller.set_depth_table(depth_table)

        self.config = rs.config()

        if self.model == 'd410':
            self.depth_x_res = 1280
            self.depth_y_res = 720
            self.color_x_res = 1280
            self.color_y_res = 720
            self.config.enable_stream(
                rs.stream.depth, self.depth_x_res, self.depth_y_res, rs.format.z16, frame_rate)
            self.config.enable_stream(
                rs.stream.infrared, self.color_x_res, self.color_y_res, rs.format.rgb8, frame_rate)
        elif self.model == 'd435':
            self.depth_x_res = 1280
            self.depth_y_res = 720
            self.color_x_res = 1920
            self.color_y_res = 1080
            self.config.enable_stream(
                rs.stream.depth, self.depth_x_res, self.depth_y_res, rs.format.z16, frame_rate)
            self.config.enable_stream(
                rs.stream.color, self.color_x_res, self.color_y_res, rs.format.rgb8, frame_rate)
        elif self.model == 'd415':
            self.depth_x_res = 1280
            self.depth_y_res = 720
            self.color_x_res = 1920
            self.color_y_res = 1080
            self.config.enable_stream(
                rs.stream.depth, self.depth_x_res, self.depth_y_res, rs.format.z16, frame_rate)
            self.config.enable_stream(
                rs.stream.color, self.color_x_res, self.color_y_res, rs.format.rgb8, frame_rate)
        else:
            raise Exception('Camera model not supported:', self.model)

        self.depth_sensor.set_option(rs.option.visual_preset, visual_preset)
        self.start_camera()

        self.camera_thermal_check()
        self.connected = True

    def find_device_that_supports_advanced_mode(self):
        DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5",
                           "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07"]
        ctx = rs.context()
        ds5_dev = rs.device()
        devices = ctx.query_devices()
        for dev in devices:
            if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
                if dev.supports(rs.camera_info.name):
                    pass
                return dev
        raise Exception("No device that supports advanced mode was found")

    def get_info_dict(self):
        info = {
            'model' : self.model,
            'depth_x_res' : self.depth_x_res, 
            'depth_y_res' : self.depth_y_res, 
            'color_x_res' : self.color_x_res, 
            'color_y_res' : self.color_y_res,
            'cfx' : self.cfx, 
            'cfy' : self.cfy, 
            'ccx' : self.ccx, 
            'ccy' : self.ccy,
            'dfx' : self.dfx, 
            'dfy' : self.dfy, 
            'dcx' : self.dcx, 
            'dcy' : self.dcy, 
            'tcp_to_ci_cam_pose' : self.tcp_to_ci_cam_pose,
            'tcp_to_pc_cam_pose' : self.tcp_to_pc_cam_pose,
            'disparity_shift' : self.get_disparity_shift(),
        }
        return info
    
    def start_camera(self):
        """
        Starts connection with camera. 

        Parameters
        ----------
        visual_preset: int (1-4)
            The 4 visual presets can be viewer on the realsense viewer.
        """
        try:
            self.profile = self.pipe.start(self.config)
        except RuntimeError as rte:
            if 'Device or resource busy' in str(rte):
                self.device.hardware_reset()
                print("Sleep 60 seconds after realsense hardware reset")
                time.sleep(60)
                try:
                    self.profile = self.pipe.start(self.config)
                except Exception as e:
                    raise RuntimeError("Camera in use") from e
            else:
                raise RuntimeError("Error connecting:", rte)
        self.get_camera_intrinsics()

        align_to = rs.stream.color
        self.alignedFs = rs.align(align_to)

        self.set_laser_state(False)

    def get_camera_intrinsics(self):
        depth_stream_profile = self.profile.get_stream(
            rs.stream.depth).as_video_stream_profile()
        d_intrinsics = depth_stream_profile.get_intrinsics()
        self.dfx = d_intrinsics.fx
        self.dfy = d_intrinsics.fy
        self.dcx = d_intrinsics.ppx
        self.dcy = d_intrinsics.ppy

        if self.model == 'd410':
            color_stream_profile = self.profile.get_stream(
                rs.stream.infrared).as_video_stream_profile()
            c_intrinsics = color_stream_profile.get_intrinsics()
            self.cfx = c_intrinsics.fx
            self.cfy = c_intrinsics.fy
            self.ccx = c_intrinsics.ppx
            self.ccy = c_intrinsics.ppy

        if self.model == 'd435':
            color_stream_profile = self.profile.get_stream(
                rs.stream.color).as_video_stream_profile()
            c_intrinsics = color_stream_profile.get_intrinsics()
            self.cfx = c_intrinsics.fx
            self.cfy = c_intrinsics.fy
            self.ccx = c_intrinsics.ppx
            self.ccy = c_intrinsics.ppy
        
        if self.model == 'd415':
            color_stream_profile = self.profile.get_stream(
                rs.stream.color).as_video_stream_profile()
            c_intrinsics = color_stream_profile.get_intrinsics()
            self.cfx = c_intrinsics.fx
            self.cfy = c_intrinsics.fy
            self.ccx = c_intrinsics.ppx
            self.ccy = c_intrinsics.ppy

    def stop_camera(self):
        """ Disconnect camera """
        self.pipe.stop()

    def resume_camera(self):
        self.pipe.start(self.config)

    def restart_camera(self):
        self.pipe.stop()
        self.pipe.start()

    def set_laser_state(self, set_state):
        """
        Toggles laser projector. In some cases turning the laser off can be helpful with shiny objects.

        Parameters
        ----------
        state: bool
            True for turning the laser on. 
        """
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass

        if self.depth_sensor.supports(rs.option.emitter_enabled):
            if not set_state:
                self.depth_sensor.set_option(rs.option.emitter_enabled, 0.0)
                self.camera_set_time = time.time()
            else:
                self.depth_sensor.set_option(rs.option.emitter_enabled, 1.0)
        else:
            print('Error setting laser state #2')

        # Reset clock
        self.camera_set_time = time.time()

    def set_laser_power(self, power):
        """
        Sets the power of the laser. In some cases turning the laser off can be helpful with shiny objects.

        Parameters
        ----------
        power: float
            laser power
        """
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass

        if power > 1 or power < 0:
            raise RuntimeError("Set laser power from 0-1")
        power = power*150
        if self.depth_sensor.supports(rs.option.laser_power):
            self.depth_sensor.set_option(rs.option.laser_power, power)
        else:
            print("Error setting laser power")

        # Reset clock
        self.camera_set_time = time.time()
        return True

    def get_laser_state(self):
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass

        if self.depth_sensor.supports(rs.option.emitter_enabled):
            state = self.depth_sensor.get_option(rs.option.emitter_enabled)
        else:
            print("Error getting laser state")
        return state

    def get_laser_power(self):
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass

        if self.depth_sensor.supports(rs.option.laser_power):
            power = self.depth_sensor.get_option(rs.option.laser_power)
            power = power/150
        else:
            print("Error getting laser power")
        return power

    def set_disparity_shift(self, disparity_value):
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass

        depth_table = self.camera_controller.get_depth_table()
        depth_table.disparityShift = disparity_value
        self.camera_controller.set_depth_table(depth_table)

        # Reset clock
        self.camera_set_time = time.time()
        return True

    def get_disparity_shift(self):
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass

        depth_table = self.camera_controller.get_depth_table()
        disparity = depth_table.disparityShift
        return disparity

    def disparity_shift_dist_function(self, camera_dist):
        if camera_dist < 0.129:
            print("distance out of range")
            return 280
        elif camera_dist > 0.450:
            return 0
        poly = np.poly1d([52799, -71664, 37494, -9418, 1009])
        return int(poly(camera_dist))+3

    def set_disparity_shift_dist(self, tcp_dist):
        disp_shift = self.disparity_shift_dist_function(
            tcp_dist-self.tcp_to_ci_cam_pose[2, 3])
        self.set_disparity_shift(disp_shift)
        return disp_shift

    def tune_disparity_shift(self, start_disparity=0, end_disparity=400, disparity_step=25, output='none'):
        """
        Tunes the disparity shift. If disparity shift is not tuned, the cloud will not \
        render properly. This function starts by setting a low disparity and increases the disparity \
        while counting the number of points in the cloud each time. It then returns the disparity \
        at which it found the most points. This process is very slow so minimize its use as much as \
        possible.

        Paramters
        ---------
        start_disparity: int
            Disparity value at which to start looking. 
        end_disparity: int
            Disparity value at which to stop looking.
        disparity_step: int
            The step between each value. Increasing step size will increase speed but decrease accuracy.

        """

        disparity_values = np.arange(int(start_disparity), int(
            end_disparity), int(disparity_step))
        num_points = []
        for disparity_value in disparity_values:
            self.set_disparity_shift(disparity_value)
            pts = self.get_cloud()
            num_points.append(
                np.where(np.all(np.equal(pts, np.zeros(3)), axis=1) == False)[0].shape[0])

        disp_shift = disparity_values[np.argmax(num_points)]
        if output in ['info', 'all']:
            print("Disparity Shift: {}".format(disp_shift))
        self.set_disparity_shift(disp_shift)
        return disp_shift

    def set_auto_exposure_on(self, state):
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass
        self.depth_sensor.set_option(rs.option.enable_auto_exposure, state)
        # Reset clock
        self.camera_set_time = time.time()

    def set_exposure(self, exposure):
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass
        # set from 1 to 165000
        self.depth_sensor.set_option(rs.option.exposure, exposure)
        # Reset clock
        self.camera_set_time = time.time()

    def get_exposure(self):
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass
        return self.depth_sensor.get_option(rs.option.exposure)

    def set_auto_white_balance_on(self, state):
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass
        self.depth_sensor.set_option(
            rs.option.enable_auto_white_balance, state)
        # Reset clock
        self.camera_set_time = time.time()

    def set_white_balance_gain(self, gain):
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass
        # set from 1 to 165000
        self.depth_sensor.set_option(rs.option.white_balance, gain)
        # Reset clock
        self.camera_set_time = time.time()

    def get_white_balance_gain(self):
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass
        return self.depth_sensor.get_option(rs.option.gain)

    def get_camera_temperature(self):
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass
        dev = self.profile.get_device()
        depth_sensor = dev.first_depth_sensor()
        tempProj = depth_sensor.get_option(rs.option.projector_temperature)
        tempAsic = depth_sensor.get_option(rs.option.asic_temperature)
        return tempProj, tempAsic

    def camera_thermal_check(self):
        """ Check the thermal limits on the camera, raise exception if exceeded """
        camera_temps = self.get_camera_temperature()
        asic_limit = 110
        proj_limit = 60

        if camera_temps[0] > proj_limit or camera_temps[1] > asic_limit:
            # If opencm is imported deactivate gripper
            try:
                self.deactivate_gripper()
            except:
                pass
            # Turn laser off
            self.set_laser_state(0)
            raise Exception("Camera overheat!")
        return True

    def get_depth_image(self):
        # Turn laser on if off
        change_laser = False
        if not self.get_laser_state():
            self.set_laser_state(True)
            change_laser = True

        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass

        # Get depth image
        frames = self.pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_data = depth_frame.get_data()

        if change_laser:
            self.set_laser_state(False)

        return np.asanyarray(depth_data)

    def get_cloud(self):
        """
        Retrieves the point cloud from the realsense camera. 

        Parameters
        ----------
        post_proccessing: bool
            Enabling post processing will give a smoother and usually a more accurate cloud; however, \
            it will also slightly decrease the speed.
        """
        # Turn laser on if off
        change_laser = False
        if not self.get_laser_state():
            self.set_laser_state(True)
            change_laser = True

        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass

        # Get depth image
        frames = self.pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        # Set laser to starting state
        if change_laser:
            self.set_laser_state(False)

        # Convert depth to point cloud
        cloud = self.convert_depth_frame_to_cloud(depth_frame)
        return cloud

    def get_all(self):

        # Get ir data
        color_data = self.get_color_image()

        # Turn laser on if off
        change_laser = False
        if not self.get_laser_state():
            self.set_laser_state(True)
            change_laser = True

        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass

        # Get depth image
        frames = self.pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_data = np.asanyarray(depth_frame.get_data())

        # Set laser to starting state
        if change_laser:
            self.set_laser_state(False)

        cloud = self.convert_depth_frame_to_cloud(depth_frame)

        return color_data, depth_data, cloud

    def convert_depth_frame_to_cloud(self, depth_frame):
        points = self.pc.calculate(depth_frame)
        vtx = np.asanyarray(points.get_vertices())
        pts = np.vstack((vtx[:]['f0'], vtx[:]['f1'], vtx[:]['f2'])).T
        cloud = pts[np.where(np.all(np.equal(pts, np.zeros(3)), axis=1) == False)[0]]
        return cloud
        
    def get_color_image(self):
        # Wait for camera to finish prior operation
        while(time.time()-self.camera_set_time <= 0.5):
            pass
        frames = self.pipe.wait_for_frames()
        if self.model == 'd410':
            color_frames = frames.get_infrared_frame()
        elif self.model == 'd435':
            color_frames = frames.get_color_frame()
        elif self.model == 'd415':
            color_frames = frames.get_color_frame()
        color_data = color_frames.get_data()
        return np.asanyarray(color_data)

    def get_pixels_per_meter(self, distance, res=(720, 1280)):
        half_width = distance * (res[0] - self.ccy) / self.cfy
        scale = res[0]/(2.0*half_width)
        return scale

    def convert_depth_image_to_cloud(self, depth):
        rows, cols = depth.shape
        c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
        z = np.divide(depth, 10000.0)
        x = np.divide(np.multiply(z, np.subtract(c, self.dcx)), self.dfx)
        y = np.divide(np.multiply(z, np.subtract(r, self.dcy)), self.dfy)
        pts = np.dstack((x, y, z)).reshape(-1, 3)
        cloud = pts[np.where(np.all(np.logical_not(
            np.equal(pts, np.zeros(3))), axis=1))[0]]
        return cloud

#     def convert_cloud_to_depth_image(self, cloud):
#         #cloud = cloud.dot(np.linalg.inv(
#         #    np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])))
#         image = np.zeros((720, 1280), dtype=np.uint16)
#         x = cloud[:, 0]
#         y = cloud[:, 1]
#         z = cloud[:, 2]

#         c = np.floor(
#             np.add(np.divide(np.multiply(x, self.fx), z), self.cx)).astype(int)
#         r = np.floor(
#             np.add(np.divide(np.multiply(y, self.fy), z), self.cy)).astype(int)
#         idxs = np.where(np.logical_and(np.logical_and(
#             c >= 0, c < 1280), np.logical_and(r >= 0, r < 720)))
#         c = c[idxs]
#         r = r[idxs]
#         z = z[idxs]

#         z = np.multiply(z, 10000.0)
#         image[r, c] = z
#         return image
