import math
import numpy as np
import time
import cv2

from rmlib.rmtools.pmath import translate_pose , rotate_pose , cartesian_mtrx_of_poses , transform_points

from rmlib.rmtools.clouds import remove_far_z_points , downsample , segment_dbscan , filter_by_size , get_bounding_box , remove_close_z_points

class Cloud_Capturing:
    """ Getting large clouds """
    
    def get_view_dimensions( self , view_distance , rm_robot ):
        """ Return the real dimensions of the image plane at `view_distance` [m] """
        camera_z_offset    = -rm_robot.camera.tcp_to_pc_cam_pose[2,3]
        distance_to_camera = view_distance + camera_z_offset
#         self.logger.debug( "distance_to_camera: " + str( distance_to_camera ) + " , camera_z_offset: " + str( camera_z_offset ) )
        m_per_p = rm_robot.camera.get_pixels_per_meter( distance_to_camera )
        view_height = 720/m_per_p
        view_width = 1280/m_per_p
#         self.logger.debug( str( view_width ) + str( view_height ) )
        return view_width , view_height


    def generate_views(self, method, pose , rm_robot , view_distance=None, x_dim=None, y_dim=None, dim_tol=0.2, output=None):  
        """  """
        
        view_pose_matrix = np.zeros((1,1,1,4,4))
        if method == 'fixed':
            #Generates a fixed view point at the given trans
            view_pose_matrix[0,0,0,:] = pose.copy()
        
        if method == 'single':
            #Generates a single view point above the feature pose
            view_pose = pose.copy()
#             view_pose = translate_pose(
#                 view_pose, 
#                 x=-rm_robot.camera.tcp_to_pc_cam_pose[0,3], 
#                 y=-rm_robot.camera.tcp_to_pc_cam_pose[1,3], 
#                 z=-view_distance, 
#                 dir_pose='self')  
            view_pose = translate_pose(
                view_pose,
                [ -rm_robot.camera.tcp_to_pc_cam_pose[0,3] , -rm_robot.camera.tcp_to_pc_cam_pose[1,3], -view_distance ] ,
                dir_pose='self'
            )  
            view_pose_matrix[0,0,0,:] = view_pose

        if method == 'cartesian':
            #Generates view points based on the rectangle dimensions and the field of view
            if x_dim is None or y_dim is None:
                raise Exception("Parameters x_dim and y_dim are required for this method") 

            #Get View Dimensions
            view_dims = self.get_view_dimensions( view_distance , rm_robot )

            # Add Tol to Bin Dims
            x_dim = x_dim*(1+dim_tol)
            y_dim = y_dim*(1+dim_tol)

            # Get Number of View Points
            x_points = math.ceil(x_dim/view_dims[0])
            y_points = math.ceil(y_dim/view_dims[1])

            # Calc Spacing
            if x_points > 1: x_range = np.linspace(0, -x_dim+view_dims[0], x_points)
            else: x_range = [0]
            if y_points > 1: y_range = np.linspace(0, y_dim-view_dims[1], y_points)
            else: y_range = [0]
                

            # Generate Start pose
            start_pose = pose
#             if x_points > 1:start_pose = translate_pose(start_pose,x=(x_dim-view_dims[0])/2,frame='self')
            if x_points > 1:
                start_pose = translate_pose(start_pose, [ (x_dim-view_dims[0])/2 , 0.0 , 0.0 ] ,dir_pose='self')
#             if y_points > 1:start_pose = translate_pose(start_pose,y=-(y_dim-view_dims[1])/2,frame='self')
            if y_points > 1:
                start_pose = translate_pose(start_pose, [ 0.0 , -(y_dim-view_dims[1])/2 , 0.0 ] , dir_pose='self')
            
            start_pose = self.generate_views('single',start_pose , rm_robot , view_distance=view_distance)[0,0,0,:]

#             self.logger.debug( str( [ x_range , y_range ] ) )
            
            # Generate pose Matrix   
            view_pose_matrix = cartesian_mtrx_of_poses(start_pose, x_range, y_range, [0], frame='tool', output=None)

        if method == 'cylindrical':
            # Generates one to many view points based on the rectangle dimensions and the field of view
            if x_dim is None or y_dim is None:
                raise Exception("Parameters x_dim and y_dim are required for this method")

            #Get View Dimensions
            view_dims = self.get_view_dimensions(view_distance)

            # Add Tol to Bin Dims
            x_dim = x_dim*(1+dim_tol)
            y_dim = y_dim*1

            # Calc FOV
            view_angle = 2*math.degrees(math.atan((x_dim/2)/view_distance))

            # Get Number of View Points
            camera_view_angle = 100
            axis_points = math.ceil(y_dim/view_dims[1])
            theta_points = math.ceil(view_angle/camera_view_angle)

            # Calc Spacing
            if axis_points > 1: axis_range = np.linspace(0,y_dim-view_dims[1],axis_points) 
            else: axis_range = 0
            if theta_points > 1: theta_range = np.linspace(0,view_angle-camera_view_angle,theta_points)
            else: theta_range = 0

            # Generate Start pose
            start_pose = translate_pose(pose, y=-(y_dim-view_dims[1])/2, z=-view_distance, dir_pose='self')
            start_pose = rotate_pose(start_pose,ry=-math.radians((view_angle-camera_view_angle)/2), dir_pose='self')
            # Generate pose Matrix        
            view_pose_matrix = self.cylindircal_mtrx_of_poses(start_pose, [0], theta_range, axis_range, frame='tool', cylinder_axis='y', output=None)
        
        
            shape = view_pose_matrix.shape
            for z in range(shape[0]):
                for y in range(shape[1]):
                    for x in range(shape[2]):
                        view_pose_matrix[z,y,x,:] = translate_pose(view_pose_matrix[z,y,x,:],x=-rm_robot.camera.tcp_to_pc_cam_pose[0,3],y=-rm_robot.camera.tcp_to_pc_cam_pose[1,3])

        return view_pose_matrix
        #TODO add mehtods: 
        #3) add convex search pattern to get better views of objects

    def get_cloud_of_area(self, view_distance, dims,  rm_robot , view_method = 'single', output=None , startPose = None):
        # Captures a cloud from the current tcp based on the area, view distance, and method provided
        # NOTE: Specifying `startPose` calculates view poses relative to that starting pose instead of current, (ex: starting pose occupied!)
        
        capture = {}
        capture['cloud_raw'] = None

        if startPose is not None:
            pose = startPose
        else:
            pose = self.get_tcp_pose()
            
        x_dim=dims[0]
        y_dim=dims[1]
        
        # Generate Views
#         view_tol = max( dims ) * 0.
        #self.logger.debug( str( [ view_method, pose, view_distance, x_dim, y_dim ] ) )
        views = self.generate_views(view_method, pose , rm_robot, view_distance=view_distance, x_dim=x_dim, y_dim=y_dim )
        cloud_raw = []
        if 1:
            distOffset = -0.030 # To repair spotty clouds
        else:
            distOffset =  0.000
            
        print( "Set disparity shift for distance:" , view_distance + distOffset , '=' , view_distance , '+' , distOffset )
        rm_robot.camera.set_disparity_shift_dist( view_distance + distOffset )
        
        #TODO add code to set lambda function to cature cloud, depth image, image
        rm_robot.camera.set_laser_state(True)
        rm_robot.camera.set_laser_power(1)
        
        def per_pose():
            """ Action to take at each point, cannot take args """
            time.sleep( 1.0 )
            cloud_raw.append( transform_points( rm_robot.camera.get_cloud() , rm_robot.get_base_to_camera_pose() ) )
            print( "Camera at:" , rm_robot.get_base_to_camera_pose()[0:3,3] )
            
    
            
            
        
        rm_robot.move_through_mtrx_of_poses( views , per_pose ,
                                         method='123', path='optimal', speed_per=None )#, pauseEach = 0.25 )
        
        
        rm_robot.camera.set_laser_state(False)
        cloud_raw = np.vstack(cloud_raw)
        
        if output in ['all']:
            # View clouds
            print('Cloud Output:' , )
            cloud_ds = self.downsample_cloud(cloud_raw, leaf_size=0.003)
            view = self.PC_Viewer()
            view.add_axis(rm_robot.get_base_to_camera_pose())
            view.add_axis(rm_robot.arm.get_tcp_pose())
            view.add_cloud(cloud_ds)
            view.show(view='base')
            
        if output == 'gui':
#           save locally
            self.capture = {}
            self.capture['cloud_raw'] = cloud_raw
#           save to file
            view = self.PC_Viewer()
            cloud_ds = self.downsample_cloud(cloud_raw, leaf_size=0.003)
            view.add_cloud(cloud_ds)
            view.saveToFile(view='base')
            return True
        else:
            capture['cloud_raw'] = cloud_raw
            return capture
        
        
        
    def get_filtered_cloud_of_area(self, view_distance , zfarvalue , zclosevalue , dims,  rm_robot , 
                                         view_method = 'single', output=None , startPose = None , clipFudgeZ = 0.035,
                                         cloudLenMin = 1000 / 10 , cloudLenMax = 10000000 / 10 , search_radius=0.02 , db_neighbors = 6,
                                         dsLeafSize = 0.005 , 
                                         _DEBUG = 0 ):
        # Captures a cloud from the current tcp based on the area, view distance, and method provided
        # NOTE: Specifying `startPose` calculates view poses relative to that starting pose instead of current, (ex: starting pose occupied!)
        
        _FILTER_ON = 1
        
        capture = {}
        capture['cloud_raw'] = None
        
        zfarvalue -= rm_robot.camera.pc_cam_offset[2]
        zfarvalue += clipFudgeZ
        
        zclosevalue -= rm_robot.camera.pc_cam_offset[2]
        zclosevalue += clipFudgeZ
        
        print( "Filtering everything farther than" , zfarvalue , "from the camera" )
        
#         zfarvalue = view_distance + clipBelow_m

        if startPose is not None:
            pose = startPose
        else:
            pose = self.get_tcp_pose()
            
        x_dim=dims[0]
        y_dim=dims[1]
        
        # Generate Views
#         view_tol = max( dims ) * 0.
        #self.logger.debug( str( [ view_method, pose, view_distance, x_dim, y_dim ] ) )
        views = self.generate_views(view_method, pose , rm_robot, view_distance=view_distance, x_dim=x_dim, y_dim=y_dim )
        cloud_raw = []
        if 1:
            distOffset = -0.030 # To repair spotty clouds
        else:
            distOffset =  0.000
            
        print( "Set disparity shift for distance:" , view_distance + distOffset , '=' , view_distance , '+' , distOffset )
        rm_robot.camera.set_disparity_shift_dist( view_distance + distOffset )
        
        #TODO add code to set lambda function to cature cloud, depth image, image
        rm_robot.camera.set_laser_state(True)
        rm_robot.camera.set_laser_power(1)
        
        def per_pose():
            """ Action to take at each point, cannot take args """
            time.sleep( 1.0 )#one second
            
            #cloud_raw.append( transform_points( rm_robot.camera.get_cloud() , rm_robot.get_base_to_camera_pose() ) )
            color_img, depth_img, full_cloud = rm_robot.camera.get_all_aligned()
            #rm_robot.camera.get_cloud() 

            print( "Camera at:" , rm_robot.get_base_to_camera_pose()[0:3,3] )
            
            if _FILTER_ON:
                # Setup kernel
                steps = 10
                kernel = np.ones( (steps, steps), np.uint8 )
                #Erode
                depth_img = cv2.erode(depth_img, kernel, iterations=1) 
                #rm.viewer.show_image(depth_img)

                #Dilate
                depth_img = cv2.dilate(depth_img, kernel, iterations=1) 
                #rm.viewer.show_image(depth_img)

            # Convert depth image to point cloud
#             cloud = transform_points( rm_robot.camera.convert_aligned_depth_image_to_cloud(depth_img) , rm_robot.get_base_to_camera_pose() )
            cloud = rm_robot.camera.convert_aligned_depth_image_to_cloud(depth_img)
    
            # Remove points with z value > threshhold (remove table and floor)
            print( "cloud.shape:" , cloud.shape , "before z clip" )
        
            if _FILTER_ON:
                print( "cloud.shape before FAR clip:" , cloud.shape )
                cloud = remove_far_z_points(   cloud , zfarvalue   )
                print( "cloud.shape before NEAR clip:" , cloud.shape )
                cloud = remove_close_z_points( cloud , zclosevalue )

            print( "cloud.shape after clip:" , cloud.shape )

            if _DEBUG:
                total = 0.0
                n     = 0
                for i in range( cloud.shape[0] ):
                    if i%int(1E4) == 0:
                        total += cloud[i,2]
                        n     += 1
                        print( cloud[i,:] )
                print( "Average distance from cam:" , total/n )
                             
            cloud_raw.append( transform_points(cloud, rm_robot.get_base_to_camera_pose() ) )  
#             cloud_raw.append( cloud )  
                             
        def generateboundingbox(cloud):

            bbox, dims = get_bounding_box(cloud, lock_z=True)

            from pmath import pose_from_three_poses , combine_rot_and_trans_from_poses

            # Get pose of bounding box
            trans = np.eye(4)
            trans[:3,3] = [(bbox[0][0]+bbox[3][0])/2,(bbox[0][1]+bbox[3][1])/2, cloud[:,2].min()]
            pose_1 = np.eye(4)
            pose_2 = np.eye(4)
            pose_3 = np.eye(4)
            pose_1[:3,3] = bbox[0]
            pose_2[:3,3] = bbox[1]
            pose_3[:3,3] = bbox[2]
            pose = pose_from_three_poses(pose_1, pose_2, pose_3)
            pose = combine_rot_and_trans_from_poses(pose, trans)

            # Show
        #     scene = rm.viewer.Scene_3D()
        #     scene.add_cloud(cloud, colorize=True, color=[255,200,0])
        #     scene.add_axis(pose)
        #     scene.add_box(bbox)
        #     scene.add_origin()
        #     scene.render()
            #print("pose:")
            #print(pose)
            return pose,bbox
            
        
        rm_robot.move_through_mtrx_of_poses( views , per_pose ,
                                         method='123', path='optimal', speed_per=None )#, pauseEach = 0.25 )
        
        
        rm_robot.camera.set_laser_state(False)
                             
        #for cloudsnapshot in cloud_raw:
                 #temimage=rm_robot.camera.convert_cloud_to_depth_image(cloudsnapshot)      
            
        cloud_raw = np.vstack(cloud_raw)
        
        print( "cloud.shape before downsample:" , cloud_raw.shape )
        
        cloud_raw = downsample(cloud_raw, leaf_size = dsLeafSize , output=[])
        
        
        
        # Downsample points for speed
        
        
        
        
#         cloud = np.vstack(clouds)
            
        if _DEBUG:
            clouds = segment_dbscan( cloud_raw , search_radius=search_radius, min_samples=db_neighbors, output=['all'])
        else:
            clouds = segment_dbscan( cloud_raw , search_radius=search_radius, min_samples=db_neighbors, output=[])
        
        buffer = clouds[:]

        clouds = filter_by_size(clouds, cluster_size=[ cloudLenMin , cloudLenMax ], output=[])
            
        if _FILTER_ON:
            maxDex = 0
            maxLen = 0
            for i , cloud in enumerate( clouds ):
                if cloud.shape[0] > maxLen:
                    maxLen = cloud.shape[0]
                    maxDex = i

            boardCloud = clouds[ maxDex ].copy()
            print( "Candidate board cloud has type:" , type( boardCloud )  )
            print( "Candidate board cloud has shape:" , boardCloud.shape  ) 
        else:
            boardCloud = np.vstack(clouds)
        
        pose , bbox = generateboundingbox( boardCloud )                     
                             
        
        if output in ['all']:
            # View clouds
            print('Cloud Output:' , )
            cloud_ds = self.downsample_cloud( boardCloud , leaf_size=0.003)
            view = self.PC_Viewer()
            view.add_axis(rm_robot.get_base_to_camera_pose())
            view.add_axis(rm_robot.arm.get_tcp_pose())
            view.add_cloud(cloud_ds)
            view.show(view='base')
        return boardCloud , pose , bbox