import cv2
import open3d
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn import linear_model
from pyntcloud import PyntCloud

class Cloud_Segmentation:
    def segment_cloud_dbscan(self, cloud, search_radius=0.005, min_samples=4, output=[None]):
        """
        Segments cloud via DBSCAN. DBSCAN is a spreading algorithm that looks for points within radius to add to segmented cloud\
        and subsequently uses that new point to spread.

        Parameters
        ---------
        cloud: [n,3] ndarray
            Point cloud.
        search_radius: float (optional)
            Radius that DBSCAN uses to search around current seed point.
        min_cluster_size: int (optional)
            Minimum amounts of points in a region to be considered an object.

        Returns
        -------
        object_clouds: list of m [k,3] ndarrays
            A list of point cloud objects.
        """
        
        if cloud.size == 0:
            return []

        db = DBSCAN(eps=search_radius,min_samples=min_samples).fit(cloud)

        objects_ids = np.unique(db.labels_)
        num_objects = objects_ids.shape[0]

        if objects_ids[0] == -1:
            cloud = cloud[np.where(db.labels_ != -1)]
            db.labels_ = db.labels_[np.where(db.labels_ != -1)] 

        objects_clouds = []
        for i in objects_ids:
            object_idxs = np.where(db.labels_ == i)
            if len(object_idxs[0]) < 1:
                cloud = cloud[np.where(db.labels_ != i)]
                db.labels_ = db.labels_[np.where(db.labels_ != i)]
            else:
                objects_clouds.append(cloud[object_idxs])
        
        if 'all' in output:
            # View clouds
            print('Segment Cloud DBScan Output:')
            view = self.PC_Viewer()
            view.add_cloud(cloud,colorize=True,color=[100,100,100])
            for cloud in objects_clouds:
                view.add_cloud(cloud,colorize=True)
            if 'save_to_file' in output:
                view.save_to_file()
            else:
                view.show(view='base')
            
        return objects_clouds

    def get_cloud_from_mask(self,mask,depth,height,vertical=False):
        """
        Converts a mask from an IR image to a point cloud by overlaying the image mask onto the depth image. 

        Parameters
        ----------
        mask: [720,1280] ndarray
            Mask should be zeros except for at the indeces of the desired object. 
        depth: [720,1280] ndarray
            Depth data retrieved from camera.
        height: float (m)
            Height of object.
        vertical: bool (optional)
            Make the object cloud surface perpendicular to the camera view, at an average object height.

        Returns
        -------
        object_cloud: [n,3] ndarray
            Point cloud of object.
            
        """

        ransac = linear_model.RANSACRegressor()

        ring_depth_idxs = np.where(np.logical_and((mask!=0),(depth!=0.0)))
        ring_mask_idxs = np.where(mask!=0.0)

        offset = False

        X = np.vstack(ring_depth_idxs).T
        X2 = np.vstack(ring_mask_idxs).T
        y = depth[ring_depth_idxs]
        if len(y)<500:
            depth_idxs = np.where(depth!=0)
            perm = np.random.permutation(int((np.vstack(depth_idxs).shape)[1]/4))
            X = (np.vstack(depth_idxs).T)[perm]
            y = (depth[depth_idxs].T - height*1000)[perm]
            offset = True

        if vertical:
            y2 = int(np.average(y))
        else:
            ransac.fit(X,y)
            y2 = ransac.predict(X2)

        if offset:
            y2 -= height * 10000

        ring_filled_image = np.zeros(mask.shape)
        ring_filled_image[ring_mask_idxs] = y2

        ring_cloud = self.convert_depth_image_to_point_cloud(ring_filled_image)

        return ring_cloud
    
#     def segment_boxes(self,cloud,obj_dims,leaf_size,search_radius):
        
#         cloud_nt = self.remove_planar_surface(cloud,obj_dims[2]/2.0)

#         min_cluster_size = (obj_dims[0]*obj_dims[1]/(7.*leaf_size))*1000.

#         if len(cloud_nt) > 0:
#             obj_clouds = self.segment_cloud(cloud_nt,min_cluster_size=int(min_cluster_size),search_radius=search_radius)
#         else:
#             obj_clouds = None

#         return obj_clouds

    def segment_rings_outer(self,cloud,depth,ir,ring_dims,leaf_size,tolerance,vertical=False):
        """
        Segment rings based on the outer diameter. 
        
        Parameters
        ----------
        cloud: [n,3] ndarray
            Point cloud.
        depth: [m,k] ndarray
            Depth data from realsense camera.
        ir: [m,k] ndarray
            IR image from realsense camera. 
        ring_dims: [3,] list
            Dimensions of ring: [inner_diameter, outer_diameter, height]
        leaf_size: float
            The downsampling leaf size used for the passed cloud. (m)
        tolerance: float
            Dimensions comparison tolerance for object. (m)
        vertical: bool
            If vertical is true, the algorithm assumes the ring is laying flat relative \
            to the camera.
            
        Returns
        -------
        ring_objs: list of [l,3] ndarrays
            List of clouds corrisponding to ring objects. 
        """
        cloud_vg = self.downsample_cloud(cloud,0.01)
        height_est = self.get_distance_estimate(cloud_vg)
        scale = self.get_pixels_per_meter(height_est)

        ring_IR = int(ring_dims[0]*scale/2.0)
        ring_OR = int(ring_dims[1]*scale/2.0)

        ring_thickness = ring_OR-ring_IR
        ring_tolerance = int(tolerance*scale)

        masks = []
        circles = cv2.HoughCircles(ir,cv2.HOUGH_GRADIENT, 1, 40, param1=15, param2=30, minRadius=ring_OR-ring_tolerance, maxRadius=ring_OR+ring_tolerance)
        try:
            circles = circles.reshape(-1,3)
        except:
            return None

        for circle in circles:
            mask = np.zeros(ir.shape)
            circle[2] = circle[2]-int(ring_thickness/2.0)
            cv2.circle(mask,(circle[0],circle[1]),circle[2],(255,255,255),ring_thickness)
            masks.append(mask)

        if masks is None:
            return None
        ring_objs = []
        for mask in masks:
            ring_obj = self.get_cloud_from_mask(mask,depth,ring_dims[2],vertical=vertical)
            ring_obj_vg = self.downsample_cloud(ring_obj,leaf_size)
            ring_objs.append(ring_obj)        
        if len(ring_objs) == 0:
            ring_objs = None
        return ring_objs
    
#     def segment_rings_inner(self,cloud,depth,ir,ring_dims,leaf_size,tolerance,vertical=False):
#         """
#         Segment rings based on the inner diameter. 
        
#         Parameters
#         ----------
#         cloud: [n,3] ndarray
#             Point cloud.
#         depth: [m,k] ndarray
#             Depth data from realsense camera.
#         ir: [m,k] ndarray
#             IR image from realsense camera. 
#         ring_dims: [3,] list
#             Dimensions of ring: [inner_diameter, outer_diameter, height]
#         leaf_size: float
#             The downsampling leaf size used for the passed cloud. (m)
#         tolerance: float
#             Dimensions comparison tolerance for object. (m)
#         vertical: bool
#             If vertical is true, the algorithm assumes the ring is laying flat relative \
#             to the camera.
            
#         Returns
#         -------
#         ring_objs: list of [l,3] ndarrays
#             List of clouds corrisponding to ring objects. 
#         """
        
#         cloud_vg = self.downsample_cloud(cloud,0.01)
#         height_est = self.get_distance_estimate(cloud_vg)
#         scale = self.get_scale_for_real_to_pixel(height_est)

#         ring_IR = int(ring_dims[0]*scale/2.0)
#         ring_OR = int(ring_dims[1]*scale/2.0)

#         ring_thickness = ring_OR-ring_IR
#         ring_tolerance = int(tolerance*scale)

#         masks = []
#         circles = cv2.HoughCircles(ir,cv2.HOUGH_GRADIENT, 1, 40, param1=15, param2=30, minRadius=ring_IR-ring_tolerance, maxRadius=ring_IR+ring_tolerance)
#         try:
#             circles = circles.reshape(-1,3)
#         except:
#             return None

#         for circle in circles:
#             mask = np.zeros(ir.shape)
#             circle[2] = circle[2]+int(ring_thickness/2.0)
#             cv2.circle(mask,(circle[0],circle[1]),circle[2],(255,255,255),ring_thickness)
#             masks.append(mask)

#         if masks is None:
#             return None
#         ring_objs = []
#         for mask in masks:
#             ring_obj = self.get_cloud_from_mask(mask,depth,ring_dims[2],vertical=vertical)
#             ring_obj_vg = self.downsample_cloud(ring_obj,leaf_size)
#             ring_objs.append(ring_obj)        
#         if len(ring_objs) == 0:
#             ring_objs = None
#         return ring_objs
    
#     def segment_cylinders(self,cloud,depth,ir,cyl_dims,leaf_size,search_radius,tolerance):
        
#         if cyl_dims[0] >= 0.015:
#             ### Large cylinder ###
#             cloud_nt = self.remove_planar_surface(cloud,cyl_dims[0]/3.0)

#             pcd = open3d.PointCloud()
#             pcd.points = Vector3dVector(cloud_nt)
#             open3d.estimate_normals(pcd, KDTreeSearchParamHybrid(
#                     radius = 0.01, max_nn = 30))
#             open3d.orient_normals_to_align_with_direction(pcd,np.array([0.0,0.0,-1.0]))

#             thetas = np.arccos(np.array(pcd.normals)).dot([0,0,1])

#             itt = 0
#             cyl_trans = None
#             angle_tolerance = 0.3
#             cyl_objs = []

#             upward_cloud = np.array(pcd.points)[np.where(thetas>3.14-angle_tolerance)]
#             if len(upward_cloud) > 0:
#                 up_objs = self.segment_cloud(upward_cloud,min_cluster_size=int(0.1/(2*leaf_size)),search_radius=search_radius)
#                 if len(up_objs) > 0:
#                     up_objs = self.sort_clouds_height(up_objs)
#                     for cyl_obj in up_objs:
#                         cyl_objs.append(cyl_obj)
                        
#         else:
#             ### Small cylinder ###
#             mask = np.zeros(ir.shape)
#             ir = np.round(ir*255.0/ir.max()).astype(np.uint8)

#             mask[np.where(ir>60)] = 255

#             filtered_cloud = self.get_cloud_from_mask(rsc,mask,depth,cyl_dims[0],vertical=True)
#             filtered_cloud_vg = self.downsample_cloud(filtered_cloud,leaf_size)

#             cyl_objs = self.segment_cloud(filtered_cloud_vg,leaf_size*2.5,0.05/(5.0*leaf_size))
            
#         return cyl_objs
    
#     def segment_gui(self,min_cluster_size=50,max_cluster_size=10000000,curve_threshold=0.013,angle_threshold=0.025,k=10,method=None,output=None):
#         """
#         Segments self.cloud for gui without needing to pass it a cloud
        
#         Parameters
#         ---------
#         min_cluster_size: int (optional)
#             Minimum amounts of points in a region to be considered an object.
#         curve_threshold: float (optional)
#             Threshold for the maximum curvature a point can have relative to its surrounding points, to be added as a seed point for the region. 
#         angle_threshold: float (optional)
#             Threshold for the angle between the current point's normal vector, and the normal vector of the neighboring point. 
#         k: int
#             The number of nearest neighbor to be calculated for each point in the cloud.
            
#         Returns
#         -------
#         object_clouds: list of m [k,3] ndarrays
#             A list of point cloud objects.
#         """
#         if method in ['dbScan']:
#             return self.segment_cloud(self.cloud)
#         if method in ['nbScan']:
#             return self.segment_cloud_nscan(slef.cloud)
#         else:
#             print('no method specified')
#             return self.cloud
            
        
        
    

