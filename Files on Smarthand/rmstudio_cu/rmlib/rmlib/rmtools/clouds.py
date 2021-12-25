import numpy as np
try:
	import open3d
except:
	pass

from sklearn import linear_model
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
from scipy.spatial import ConvexHull
from rmlib.rmtools import poses as pmath

from pyntcloud import PyntCloud
import pandas as pd

try:
	from rmlib.rmtools.cpp import nscan as nscan
except:
	pass

try:
    __IPYTHON__
    from . import viewer
except:
    pass

def save(cloud, file):
    np.save(file, cloud)
    
def load(file):
    cloud = np.load(file)
    return cloud
    
def downsample(cloud, leaf_size=0.005, output=[]):
    """
    Downsample point cloud by taking mean position of points in a grid of voxels. 

    Parameters
    ----------
    cloud: [n,3] ndarray
        Point cloud to be compressed.
    leaf_size: float (optional)
        Edge length of voxels.

    Return
    ------
    downsampled_cloud: [k,3] ndarray
        The downsampled point cloud.
    """
    if leaf_size >= 0.005:
        min_x, min_y, min_z = cloud.min(axis=0)
        max_x, max_y, max_z = cloud.max(axis=0)
        x = np.arange(min_x, max_x, leaf_size)
        y = np.arange(min_y, max_y, leaf_size)
        z = np.arange(min_z, max_z, leaf_size)

        voxel_x = np.clip(np.searchsorted(
            x, cloud[:, 0]) - 1, 0, x.shape[0])
        voxel_y = np.clip(np.searchsorted(
            y, cloud[:, 1]) - 1, 0, y.shape[0])
        voxel_z = np.clip(np.searchsorted(
            z, cloud[:, 2]) - 1, 0, z.shape[0])

        voxel_id = np.ravel_multi_index([voxel_x, voxel_y, voxel_z], [
                                        x.shape[0], y.shape[0], z.shape[0]])

        sort_idx = np.argsort(voxel_id)
        voxel_id_sorted = voxel_id[sort_idx]
        first_id = np.concatenate(
            ([True], voxel_id_sorted[1:] != voxel_id_sorted[:-1]))
        ids_count = np.diff(np.nonzero(first_id)[0])
        ids_idxs = np.split(sort_idx, np.cumsum(ids_count))

        cloud_vg = np.zeros((len(ids_idxs), 3))
        for i, idxs in enumerate(ids_idxs):
            cloud_vg[i] = np.mean(cloud[idxs], axis=0)

    else:
#         pcd = open3d.geometry.PointCloud()
#         pcd.points = open3d.utility.Vector3dVector(cloud)
#         downpcd = pcd.voxel_down_sample(voxel_size=leaf_size)
#         cloud_vg = np.asarray(downpcd.points)
        
        pcd = open3d.PointCloud()
        pcd.points = open3d.Vector3dVector(cloud)
        downpcd = open3d.voxel_down_sample(pcd,voxel_size=leaf_size)
        cloud_vg = np.asarray(downpcd.points)

    if 'all' in output or 'final' in output:
        # View clouds
        print('Downsample Output:')
        scene = viewer.Scene_3D()
        scene.add_cloud(cloud_vg)
        if 'save_to_file' in output:
            scene.save_to_file()
        else:
            scene.render()
                
    return cloud_vg

def segment_dbscan(cloud, search_radius=0.005, min_samples=4, output=[None]):
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

        object_clouds = []
        for i in objects_ids:
            object_idxs = np.where(db.labels_ == i)
            if len(object_idxs[0]) < 1:
                cloud = cloud[np.where(db.labels_ != i)]
                db.labels_ = db.labels_[np.where(db.labels_ != i)]
            else:
                object_clouds.append(cloud[object_idxs])
        
        if 'all' in output:
            # View clouds
            print('Segment Cloud DBScan Output:')
            scene = viewer.Scene_3D()
            scene.add_cloud(cloud,colorize=True,color=[100,100,100])
            for cloud in object_clouds:
                scene.add_cloud(cloud, colorize=True)
            if 'save_to_file' in output:
                scene.save_to_file()
            else:
                scene.render()
            
        return object_clouds

def segment_nbscan(cloud, curve_threshold=0.013, angle_threshold=0.025, k=10, output=[]):
        """
        Segments cloud via NSCAN. NSCAN is a spreading algorithm based on normal estimation. The region will spread to the \
        surrounding point only if the angle between the two points' normal vectors are within the angle_threshold, and the \
        curvature at that point is less than the curve_threshold.

        Parameters
        ---------
        cloud: [n,3] ndarray
            Point cloud.
        min_cluster_size: int (optional)
            Minimum amounts of points in a region to be considered an object.
        curve_threshold: float (optional)
            Threshold for the maximum curvature a point can have relative to its surrounding points, to be added as a seed point for the region. 
        angle_threshold: float (optional)
            Threshold for the angle between the current point's normal vector, and the normal vector of the neighboring point. 
        k: int
            The number of nearest neighbor to be calculated for each point in the cloud.

        Returns
        -------
        object_clouds: list of m [k,3] ndarrays
            A list of point cloud objects.

        """
        min_cluster_size=1
        if cloud.size <=k:
            return []

        # Columns of X,Y,Z data
        dataFramePts = {'x': cloud[:, 0], 'y': cloud[:, 1], 'z': cloud[:, 2]}
        # Indices for all of the points in the cloud
        indices = range(len(cloud[:, 0]))
        # Point cloud struct for the points
        pycl = PyntCloud(pd.DataFrame(dataFramePts), index=indices)

        k_neighbors = pycl.get_neighbors(k=k)  # kd-tree search

        # http://pointclouds.org/documentation/tutorials/normal_estimation.php
        # The solution for estimating the surface normal is therefore reduced to an analysis of the eigenvectors and eigenvalues
        # (or PCA – Principal Component Analysis) of a covariance matrix created from the nearest neighbors of the query point.
        # Eigenvalues used for surface normals and curvature
        ev = pycl.add_scalar_field("eigen_values", k_neighbors=k_neighbors)
        pycl.add_scalar_field("curvature", ev=ev)

        curvature = np.array(pycl.points['curvature({})'.format(k+1)])
        
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(cloud)
        pcd.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=k))
        pcd.orient_normals_towards_camera_location()
#         pcd = open3d.PointCloud()
#         pcd.points = open3d.Vector3dVector(cloud)
#         open3d.estimate_normals(pcd, search_param=open3d.KDTreeSearchParamHybrid(radius=0.5, max_nn=k))
#         open3d.orient_normals_towards_camera_location(pcd)
        normals = np.asarray(pcd.normals)

        x = cloud[:, 0]
        y = cloud[:, 1]
        z = cloud[:, 2]
        nx = normals[:, 0]
        ny = normals[:, 1]
        nz = normals[:, 2]
        c = curvature

        # Normals calculated, begin algo proper

        nscan.set_x(*x)
        nscan.set_y(*y)
        nscan.set_z(*z)
        nscan.set_nx(*nx)
        nscan.set_ny(*ny)
        nscan.set_nz(*nz)
        nscan.set_c(*c)

        nscan.initialize()
        nscan.connect(k, *k_neighbors.flatten())

        objectTuples = nscan.NSCAN(
            min_cluster_size, curve_threshold, angle_threshold)

        nscan.clear()

        object_clouds = []
        for i in range(len(objectTuples)):
            object_clouds.append(np.array(objectTuples[i]))
                
        if 'all' in output:
            # View clouds
            print('Segment Cloud NBScan Output:')
            scene = viewer.Scene_3D()
            scene.add_cloud(cloud, colorize=True,color=[100,100,100])
            for cloud in object_clouds:
                scene.add_cloud(cloud, colorize=True)
            if 'save_to_file' in output:
                scene.save_to_file()
            else:
                scene.render()

        return object_clouds

def remove_planar_surface(cloud, rmv_tolerance=0.0025, rmv_high=True, rmv_low=False, output=[]):
    """ 
    Removes points that are contained in an estimated planar surface computed with RANSAC Regression. 
    Usefull for separating objects from the surface that they rest on. 

    Parameters
    ----------
    cloud: [n,3] ndarray
        Point cloud.
    rmv_tolerance: float (optional)
        Thickness tolerance for surface removal (m).
    rmv_high: bool (optional)
        Removes all points higher (in the +Z direction) than the plane
    rmv_low: bool (optional)
        Removes all points lower (in the -Z direction) than the plane

    Return
    ------
    cloud_cleaned: [k,3] ndarray
        Point cloud excluding the points of the estimated surface.
    """

    surface_est = get_planar_surface_estimate(cloud.copy())[:, 2]
    dist = cloud[:, 2]-surface_est

    if rmv_high & (not rmv_low):  # remove higher
        not_table_idxs = np.where(np.logical_and(
            np.abs(dist) > rmv_tolerance, dist < 0.0))
    elif (not rmv_high) & rmv_low:  # remove lower
        not_table_idxs = np.where(np.logical_and(
            np.abs(dist) > rmv_tolerance, dist > 0.0))
    elif rmv_high & rmv_low:  # remove both
        not_table_idxs = []
    else:  # remove neither
        not_table_idxs = np.where(np.abs(dist) > rmv_tolerance)

    cloud_nt = cloud[not_table_idxs]

#     if 'all' in output or 'final' in output:
#         # View clouds
#         print('Remove Plannar Surface Output:')
#         view = self.PC_Viewer()
#         view.add_cloud(cloud_nt)
#         if 'save_to_file' in output:
#             view.save_to_file()
#         else:
#             view.show(view='base')

    return cloud_nt

def get_planar_surface_estimate(cloud, rtnEst=False):
        """
        Generates a planar cloud that follows that of a planar surface estimated with RANSAC Regression.

        Parameters
        ----------
        cloud: [n,3] ndarray
            Point cloud.

        Returns
        -------
        plane_cloud: [n,3] ndarray
            Planar point cloud that follows the estimated surface.
        """

        x_y_coords_pts = cloud[:, :2]
        z_coord_pts = cloud[:, -1]

        ransac = linear_model.RANSACRegressor()
        ransac.fit(x_y_coords_pts, z_coord_pts)

        surface_est = x_y_coords_pts.dot(
            ransac.estimator_.coef_) + ransac.estimator_.intercept_

        plane_cloud = cloud
        plane_cloud[:, 2] = surface_est

        if rtnEst:
            return plane_cloud, {'coeff': ransac.estimator_.coef_, 'intercept': ransac.estimator_.intercept_}
        else:
            return plane_cloud
        
def filter_by_size(clouds, dim_1=[0, 100], dim_2=[0, 100], dim_3=[0, 100], cluster_size=[10, 10000000], max_clouds_returned=1000, output=[]):
    accepted_clouds = []
    feature_pose_list = []
    for test_cloud in clouds:
        if len(accepted_clouds) < max_clouds_returned:
            add_cloud = False
            length = len(test_cloud)
            if length > cluster_size[0] and length < cluster_size[1]:
                dims = get_cloud_dimensions(test_cloud)
                if dim_1[0] < dims[0] < dim_1[1]:
                    if dim_2[0] < dims[1] < dim_2[1]:
                        add_cloud = True
                    if dim_3[0] < dims[1] < dim_3[1]:
                        add_cloud = True
                if dim_2[0] < dims[0] < dim_2[1]:
                    if dim_1[0] < dims[1] < dim_1[1]:
                        add_cloud = True
                    if dim_3[0] < dims[1] < dim_3[1]:
                        add_cloud = True
                if dim_3[0] < dims[0] < dim_3[1]:
                    if dim_1[0] < dims[1] < dim_1[1]:
                        add_cloud = True
                    if dim_2[0] < dims[1] < dim_2[1]:
                        add_cloud = True

            if add_cloud:
                if 'all' in output:
                    print('axis_1:', sorted(dims)[0], ' axis_2:', sorted(
                        dims)[1], ' axis_3:', sorted(dims)[2])
                accepted_clouds.append(test_cloud)

        else:
            break

    if accepted_clouds is None:
        raise Exception("No clouds passed through filter")

#     if 'all' in output or 'final' in output:
#         # View clouds
#         print('Filter Clouds Output:')
#         view = self.PC_Viewer()
#         view.add_list_of_clouds(accepted_clouds, color_scheme='ordered')
#         if 'save_to_file' in output:
#             view.save_to_file()
#         else:
#             view.show()

    return accepted_clouds

def get_cloud_dimensions(cloud):
    """
    Finds the dimensions of a point cloud.

    Parameters
    ----------
    cloud: [n,3] ndarray
        Point cloud object.

    Returns
    -------
    dimensions: [3,] ndarray
        The dimensions of the bounding box (lenght,width,height) where length > width.
    """
    _, dims = get_bounding_box(cloud)
    return dims

def get_bounding_box(cloud):
#     '''
#     Finds vertices of the bounding box of an object cloud. (Least volume)

#     Parameters
#     ----------
#     cloud: [n,3] ndarray
#         Point cloud.

#     Return
#     ------
#     vertices: [8,3] ndarray
#         Coordinates for vertices of the object bouding box.\n
#         [min_x,min_y,max_z]    vertex 0   (1)-----(0)   
#         [max_x,min_y,max_z]    vertex 1    !\      !\
#         [min_x,max_y,max_z]    vertex 2    ! \     Z \
#         [max_x,max_y,max_z]    vertex 3    ! (3)=====(2)  view
#         [min_x,min_y,min_z]    vertex 4   (5)-|X--(4) |     ^
#         [max_x,min_y,min_z]    vertex 5     \ |     \ |     |
#         [min_x,max_y,min_z]    vertex 6      \|      Y|     _
#         [max_x,max_y,min_z]    vertex 7      (7)=====(6)    V
#     dimensions: [3,] ndarray
#         The dimensions of the bounding box (lenght,width,height) where length > width.
#     '''
    
    pca = PCA(n_components=3)
    pca_cloud = pca.fit_transform(cloud)
    hull = ConvexHull(pca_cloud[:, :2])

    max_z = pca_cloud[:, 2].min()
    min_z = pca_cloud[:, 2].max()
    if pca.components_[2, 2] < 0:
        direction = -1
        max_z = pca_cloud[:, 2].max()
        min_z = pca_cloud[:, 2].min()

    volume_bb = 10000000
    for simplex in hull.simplices:
        u = pca_cloud[simplex[0], :2]
        v = pca_cloud[simplex[1], :2]
        slope = (v[1] - u[1])/(v[0] - u[0])
        theta = np.arctan(slope)
        rot = np.array([[np.cos(theta), -np.sin(theta), 0.0],
                        [np.sin(theta), np.cos(theta), 0.0], [0.0, 0.0, 1.0]])
        pc = pca_cloud.dot(rot)
        min_x = pc[:, 0].min()
        max_x = pc[:, 0].max()
        min_y = pc[:, 1].min()
        max_y = pc[:, 1].max()
        vol = np.abs(min_x-max_x)*np.abs(min_y-max_y)
        if vol < volume_bb:
            vertices = np.zeros((8, 3))
            vertices[0, :] = [min_x, min_y, max_z]
            vertices[1, :] = [max_x, min_y, max_z]
            vertices[2, :] = [min_x, max_y, max_z]
            vertices[3, :] = [max_x, max_y, max_z]
            vertices[4, :] = [min_x, min_y, min_z]
            vertices[5, :] = [max_x, min_y, min_z]
            vertices[6, :] = [min_x, max_y, min_z]
            vertices[7, :] = [max_x, max_y, min_z]
            vertices = vertices.dot(np.linalg.inv(rot))
            volume_bb = vol

        dim1 = np.linalg.norm(vertices[1]-vertices[0])
        dim2 = np.linalg.norm(vertices[2]-vertices[0])
        dim3 = np.linalg.norm(vertices[4]-vertices[0])
        if dim1 > dim2:
            bb_dim = [dim1, dim2, dim3]
        else:
            bb_dim = [dim2, dim1, dim3]

    return pca.inverse_transform(vertices), bb_dim

def get_cloud_poses(clouds, frame='camera', trans_method='pca'):
    """
    Finds the origin of the object cloud.

    Parameters
    ----------
    object_cloud: [n,3] ndarray
        Sequence of points in object cloud.

    Return
    ------
    trans: [4,4] ndarray
        The transformation matrix representing the position and orientation of the object. \
        The axis lies at the highest point on the object, with the y_axis aligned with the \
        principle axis of the cloud.
        [r11,r12,r13,tx]\n
        [r21,r22,r23,ty]\n
        [r31,r32,r33,tz]\n
        [ 0 , 0 , 0 , 1]
    """
    # Single Cloud
    if type(clouds) is not list:
        cloud_list = [clouds]
    # List of Clouds
    else:
        cloud_list = clouds

    poses = []
    # For each cloud in the list
    for cloud in cloud_list:
        pca = PCA(n_components=3)
        pca_cloud = pca.fit_transform(cloud)
        hull = ConvexHull(pca_cloud[:, :2])

        max_z = pca_cloud[:, 2].min()
        min_z = pca_cloud[:, 2].max()
        if pca.components_[2, 2] < 0:
            direction = -1
            max_z = pca_cloud[:, 2].max()
            min_z = pca_cloud[:, 2].min()

        # Find min volume
        volume_bb = 10000000
        for simplex in hull.simplices:
            u = pca_cloud[simplex[0], :2]
            v = pca_cloud[simplex[1], :2]
            slope = (v[1] - u[1])/(v[0] - u[0])
            theta = np.arctan(slope)
            rot = np.array([[np.cos(theta), -np.sin(theta), 0.0],
                            [np.sin(theta), np.cos(theta), 0.0], [0.0, 0.0, 1.0]])
            pc = pca_cloud.dot(rot)
            min_x = pc[:, 0].min()
            max_x = pc[:, 0].max()
            min_y = pc[:, 1].min()
            max_y = pc[:, 1].max()
            vol = np.abs(min_x-max_x)*np.abs(min_y-max_y)
            if vol < volume_bb:
                vertices = np.zeros((4, 3))
                origin = np.array([min_x, min_y, max_z])
                if (max_x-min_x > max_y-min_y):
                    vertices[0, :] = np.array(
                        [max_x, min_y, max_z]) - origin
                    vertices[1, :] = np.array(
                        [min_x, max_y, max_z]) - origin
                else:
                    vertices[1, :] = np.array(
                        [max_x, min_y, max_z]) - origin
                    vertices[0, :] = np.array(
                        [min_x, max_y, max_z]) - origin
                x_offset = (pc[:, 0].max() + pc[:, 0].min())/2.0
                y_offset = (pc[:, 1].max() + pc[:, 1].min())/2.0
                vertices[2, :] = np.array([min_x, min_y, min_z]) - origin
                vertices = vertices.dot(np.linalg.inv(rot))
                if frame == 'camera':
                    translation = np.array([x_offset, y_offset, max_z])
                if frame == 'base':
                    translation = np.array([x_offset, y_offset, min_z])
                translation = translation.dot(np.linalg.inv(rot))
                volume_bb = vol

        rot = np.array([vertices[0]/np.linalg.norm(vertices[0]), vertices[1] /
                        np.linalg.norm(vertices[1]), vertices[2]/np.linalg.norm(vertices[2])]).T
        rot_pca = pca.components_.T
        # Init pose
        pose = np.eye(4)
        # Set the rotation
        pose[:3, :3] = rot_pca.dot(rot)
        # Set the translation
        # If using the centroid/COM/unweighted average point as center
        if trans_method == 'centroid':
            pose[:3, 3] = cloud_centroid(cloud)
        elif trans_method == 'pca':
        # If using the PCA translation
            pose[:3, 3] = pca.inverse_transform(translation)
        pose[:3, 1] = np.cross(pose[:3, 2], pose[:3, 0])

        pose = pmath.rotate_pose(pose, [0, 0, 1.5707], dir_pose='self')
        if frame == 'base':
            pose = pmath.rotate_pose(
                pose, [0, 3.14159, 0], dir_pose='self')

        poses.append(pose)

    if type(clouds) is not list:
        return poses[0]
    else:
        return poses
    
def cloud_centroid(cloud):
    # Return the R3 unweighted centroid of the points
    length = float(cloud.shape[0])
    sum_x = np.sum(cloud[:, 0])
    sum_y = np.sum(cloud[:, 1])
    sum_z = np.sum(cloud[:, 2])
    return np.array([sum_x/length, sum_y/length, sum_z/length])

def sort_by_size(clouds, large_to_small=True, output=[]):
    """
    Sorts clouds based on the amount of points in the cloud.

    Parameters
    ----------
    clouds: list of k [n,3] ndarrays corrisponding to clouds.

    Returns
    -------
    sorted_clouds: list of k [n,3] ndarrays corrisponding to clouds sorted from largest to smallest.

    """

    clouds.sort(key=lambda x: len(x))
    if large_to_small:
        clouds.reverse()

#     if 'all' in output or 'final' in output:
#         # View clouds
#         print('Sort Clouds By Size Output:')
#         view = self.PC_Viewer()
#         view.add_list_of_clouds(clouds, color_scheme='ordered')
#         if 'save_to_file' in output:
#             view.save_to_file()
#         else:
#             view.show(view='base')
    
    if 'all' in output or 'final' in output:
        # View clouds
        print('Sort Clouds By Size Output:')
        scene = viewer.Scene_3D()
        scene.add_clouds(clouds, color_scheme='ordered')
        if 'save_to_file' in output:
            scene.save_to_file()
        else:
            scene.render()
    
    
    return clouds


def remove_far_points(cloud, dist):
    """Remove all points with x, y, or z values greater than dist"""
    idxs = np.where(np.logical_and(abs(cloud[:, 0]) < dist, abs(cloud[:, 1]) < dist, abs(cloud[:, 2]) < dist))
    return cloud[idxs]

