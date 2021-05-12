import numpy as np
import open3d
from sklearn import linear_model
from sklearn.decomposition import PCA
from scipy.spatial import ConvexHull
import pmath

from pyntcloud import PyntCloud #NOT FOR OPEN SOURCE
import pandas as pd #NOT FOR OPEN SOURCE

class Cloud_Processing:
    """ Functions for get information from, or modifying, point clouds """

    def cloud_centroid(self, cloud):
        # Return the R3 unweighted centroid of the points
        length = float(cloud.shape[0])
        sum_x = np.sum(cloud[:, 0])
        sum_y = np.sum(cloud[:, 1])
        sum_z = np.sum(cloud[:, 2])
        return np.array([sum_x/length, sum_y/length, sum_z/length])

    def downsample_cloud(self, cloud, leaf_size=0.005, output=[]):
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
            pcd = open3d.PointCloud()
            pcd.points = open3d.Vector3dVector(cloud)
            downpcd = open3d.voxel_down_sample(pcd, voxel_size=leaf_size)
            cloud_vg = np.asarray(downpcd.points)

        if 'all' in output or 'final' in output:
            # View clouds
            print('Downsample Output:')
            view = self.PC_Viewer()
            view.add_cloud(cloud_vg)
            if 'save_to_file' in output:
                view.save_to_file()
            else:
                view.show(view='base')

        return cloud_vg

    def filter_clouds_size(self, clouds, dim_1=[0, 100], dim_2=[0, 100], dim_3=[0, 100], cluster_size=[10, 10000000], max_clouds_returned=1000, output=[]):
        accepted_clouds = []
        feature_pose_list = []
        for test_cloud in clouds:
            if len(accepted_clouds) < max_clouds_returned:
                add_cloud = False
                length = len(test_cloud)
                if length > cluster_size[0] and length < cluster_size[1]:
                    dims = self.get_cloud_dimensions(test_cloud)
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

        if 'all' in output or 'final' in output:
            # View clouds
            print('Filter Clouds Output:')
            view = self.PC_Viewer()
            view.add_list_of_clouds(accepted_clouds, color_scheme='ordered')
            if 'save_to_file' in output:
                view.save_to_file()
            else:
                view.show()

        return accepted_clouds

    def remove_planar_surface(self, cloud, rmv_tolerance=0.0025, rmv_high=True, rmv_low=False, output=[]):
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

        surface_est = self.get_planar_surface_estimate(cloud.copy())[:, 2]
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

        if 'all' in output or 'final' in output:
            # View clouds
            print('Remove Plannar Surface Output:')
            view = self.PC_Viewer()
            view.add_cloud(cloud_nt)
            if 'save_to_file' in output:
                view.save_to_file()
            else:
                view.show(view='base')

        return cloud_nt

    def remove_far_points(self, cloud, dist):
        '''Remove all points with z values greater than dist'''
        idxs = np.where(cloud[:, 2] < dist)
        return cloud[idxs]

    def get_planar_surface_estimate(self, cloud, rtnEst=False):
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

    def get_distance_estimate(self, cloud):
        """
        Estimates the distance from the camera to the cloud based on RANSAC Regression.

        Parameters
        ----------
        cloud: [n,3] ndarray
            Point cloud.

        Returns
        -------
        distance: float (m)
            Estimated distance to the cloud.
        """

        surface_est = self.get_planar_surface_estimate(cloud)[:, 2]
        return np.average(surface_est)

    def crop_to_inner_dims(self, cloud, feature):
        inner_rec = self.generate_rect(
            feature['pose'], feature['inner_dimensions'])
        return self.crop_cloud_rect(cloud, inner_rec)

    def sort_clouds_height(self, clouds, high_to_low=True, output=[]):
        """
        Sorts clouds based on their maximum height.

        Parameters
        ----------
        clouds: list of k [n,3] ndarrays
            List of point clouds.

        Returns
        -------
        sorted_clouds: list of k [n,3] ndarrays corrisponding to clouds sorted from closest to furthest.
        """

        max_z = np.array([cloud[:, 2].max() for cloud in clouds])
        clouds = [clouds[idx] for idx in np.argsort(max_z)]
        if not high_to_low:
            clouds.reverse()

        if 'all' in output or 'final' in output:
            # View clouds
            print('Sort Clouds By Height Output:')
            view = self.PC_Viewer()
            view.add_list_of_clouds(clouds, color_scheme='ordered')
            if 'save_to_file' in output:
                view.save_to_file()
            else:
                view.show(view='base')
        return clouds

    def sort_clouds_size(self, clouds, large_to_small=True, output=[]):
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

        if 'all' in output or 'final' in output:
            # View clouds
            print('Sort Clouds By Size Output:')
            view = self.PC_Viewer()
            view.add_list_of_clouds(clouds, color_scheme='ordered')
            if 'save_to_file' in output:
                view.save_to_file()
            else:
                view.show(view='base')

        return clouds

    def sort_clouds_distance(self, clouds, pose, close_to_far=True, centroidCenter=False, output=[]):
        """
                Parameters
        ----------
        clouds: list of k [n,3] ndarrays corrisponding to clouds.
        pose: pose to measure distance from

        Returns
        -------

        clouds_and_poses: zipped sorted list of clouds and poses

        """
        poses = self.get_cloud_pose(
            clouds, frame='base', use_centroid=centroidCenter)
        clouds_and_poses = zip(clouds, poses)

        def sort_fun(x):
            return pmath.get_disance_between_poses(pose, x[1])
#             return self.sort_clouds_distance(pose, x[1])

        clouds_and_poses = sorted(clouds_and_poses, key=sort_fun)
        if not close_to_far:
            clouds_and_poses.reverse()

        if 'all' in output or 'final' in output:
            # View clouds
            print('Sort Clouds By Distance Output:')
            view = self.PC_Viewer()
            lstOfClouds = np.array([elem[0] for elem in clouds_and_poses])
#             view.add_list_of_clouds(clouds_and_pose, color_scheme='ordered')
            view.add_list_of_clouds(lstOfClouds, color_scheme='ordered')
#             view.add_list_of_clouds(clouds_and_poses, color_scheme='ordered')
            if 'save_to_file' in output:
                view.save_to_file()
            else:
                view.show(view='base')
        return clouds_and_poses

    def crop_cloud_box(self,cloud,box,rtn_inner_cloud = False):
        """
        Crop cloud to be within a box.

        Parameters
        ----------
        cloud: [n,3] ndarray
            Point cloud to be cropped.

        vertices: [8,3] ndarray
            vertices of box to crop to.\n
            [min_x,min_y,max_z]\n
            [max_x,min_y,max_z]\n
            [min_x,max_y,max_z]\n
            [max_x,max_y,max_z]\n
            [min_x,min_y,min_z]\n
            [max_x,min_y,min_z]\n
            [min_x,max_y,min_z]\n
            [max_x,max_y,min_z]

        Returns
        -------
        cropped_cloud: [m,3] ndarray
            Point cloud cropped to within the box.
        """
        
        if not rtn_inner_cloud:
            cropped_cloud = cloud[np.where(self.is_point_in_box(box,cloud))]
    
        else:
            cropped_cloud = cloud[np.where(np.logical_not(self.is_point_in_box(box,cloud)))]
        
        return cropped_cloud

    def crop_cloud_rect(self, cloud, vertices):
        """
        Crops points in cloud to within specified vertices. Vertices do not have to be orthagonal. \
        Does not crop with respect to z values.

        Parameters
        ----------
        cloud: [n,3] ndarray
            Point cloud to be cropped.\n
            [min_x,min_y]\n
            [max_x,min_y]\n
            [min_x,max_y]

        vertices: [4,2] ndarray
            Vertices of rectangle with (x,y) positions.

        Return
        ------
        cropped_cloud: [m,3] ndarray
            Cropped point cloud.
        """
        u = vertices[1] - vertices[0]
        v = vertices[2] - vertices[0]

        a = u.dot(cloud.T)
        b = v.dot(cloud.T)

        check_x = np.logical_and(a <= u.dot(
            vertices[1]), a >= u.dot(vertices[0]))
        check_y = np.logical_and(b <= v.dot(
            vertices[2]), b >= v.dot(vertices[0]))

        return cloud[np.logical_and(check_x, check_y)]

    def get_bounding_rect(self, cloud):
        """
        Finds bounding rectangle of object cloud. Rectangle sits tangent to highest point on cloud.

        Parameters
        ----------
        cloud: (n,3) ndarray
            Sequence of points in object cloud.

        Return
        ------
        vertices: (4,3) ndarray
            Coordinates for vertices of the object bouding rectangle\
            Order:\
            [min_x,min_y,z1]
            [max_x,min_y,z2]
            [min_x,max_y,z3]
            [max_x,max_y,z4]
        """

        pca = PCA(n_components=3)
        pca_cloud = pca.fit_transform(cloud)
        hull = ConvexHull(pca_cloud[:, :2])

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
                vertices[0, :] = [min_x, min_y, 0]
                vertices[1, :] = [max_x, min_y, 0]
                vertices[2, :] = [min_x, max_y, 0]
                vertices[3, :] = [max_x, max_y, 0]
                vertices = vertices.dot(np.linalg.inv(rot))
                volume_bb = vol
        return pca.inverse_transform(vertices)

    def get_bounding_box(self, cloud):
        """
        Finds vertices of the bounding box of an object cloud. (Least volume)

        Parameters
        ----------
        cloud: [n,3] ndarray
            Point cloud.

        Return
        ------
        vertices: [8,3] ndarray
            Coordinates for vertices of the object bouding box.\n
            [min_x,min_y,max_z]    vertex 0   (1)-----(0)   
            [max_x,min_y,max_z]    vertex 1    !\      !\
            [min_x,max_y,max_z]    vertex 2    ! \     Z \
            [max_x,max_y,max_z]    vertex 3    ! (3)=====(2)  view
            [min_x,min_y,min_z]    vertex 4   (5)-|X--(4) |     ^
            [max_x,min_y,min_z]    vertex 5     \ |     \ |     |
            [min_x,max_y,min_z]    vertex 6      \|      Y|     _
            [max_x,max_y,min_z]    vertex 7      (7)=====(6)    V
        dimensions: [3,] ndarray
            The dimensions of the bounding box (lenght,width,height) where length > width.
        """

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

    def extrude_rect(self, rectangle, height):
        """
        Extrudes rectangle down a specified height relative to the orientation of the rectangle \
        to make a bounding box.

        Parameters
        ----------
        rectangle: [4,3] ndarray
            Corrdinates of vertices for bounding rectangle.\n
            [min_x,min_y,max_z]\n
            [max_x,min_y,max_z]\n
            [min_x,max_y,max_z]\n
            [max_x,max_y,max_z]
        height: float
            Height of extrusion (m).

        Return
        ------
        bounding_box: [8,3] ndarray
            Coordinates for vertices of the object bouding box.\n
            [min_x,min_y,max_z]\n
            [max_x,min_y,max_z]\n
            [min_x,max_y,max_z]\n
            [max_x,max_y,max_z]\n
            [min_x,min_y,min_z]\n
            [max_x,min_y,min_z]\n
            [min_x,max_y,min_z]\n
            [max_x,max_y,min_z]
        """

        edge1 = np.subtract(rectangle[1], rectangle[0])
        edge2 = np.subtract(rectangle[2], rectangle[0])

        edge1_norm = np.divide(edge1, np.linalg.norm(edge1))
        edge2_norm = np.divide(edge2, np.linalg.norm(edge2))
        transform = np.cross(edge1_norm, edge2_norm)
        transform = transform/np.linalg.norm(transform)
        if transform[2] < 0:
            transform = -transform
        vertices = np.zeros((8, 3))
        vertices[0:4, :] = rectangle
        for i, point in enumerate(rectangle):
            vertices[i+4, :] = np.add(point, transform*height)
        return vertices

    def get_cloud_dimensions(self, cloud):
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
        _, dims = self.get_bounding_box(cloud)
        return dims

    def compare_dimensions(self, object_cloud, model_dims, tolerance=0., transform=None, check_height=False, smaller_than=False):
        """
        Compares the dimensions of a point cloud object to specified dimensions

        Parameters
        ----------
        object_cloud: [n,3] ndarray
            The point cloud of the object.
        model_dims: [3,] ndarray
            The dimensions of the object to be compared to [length,width,height] where length>width. \
            Note that if any of the dimensions is set to -1 the function will ignore that dimension \
            in its comparison.
        tolerance: float (optional)
            Tolerance of object dimension comparison (m).
        smaller_than: bool (optional)
            If this parameter is True, this function will return true if the object cloud is \
            smaller than the model dimensions reguardless of the tolerance. (but if it is larger \
            it still must be within the tolerance)
        transformation_matrix: [4,4]) ndarray (optional)
            Matrix used to transform cloud before producing dimensions.\n
            [r11,r12,r13,tx]\n
            [r21,r22,r23,ty]\n
            [r31,r32,r33,tz]\n
            [ 0 , 0 , 0 , 1]

        Return
        ------
        match: bool
            True if object dimensions are within the tolerance and if the aspect ratio is reasonable.
        """

        model_length = model_dims[0]
        model_width = model_dims[1]
        model_height = model_dims[2]

        match = False
        model_asp = model_width/model_length

        if transform is None:
            _, obj_dims = self.get_bounding_box(object_cloud)
        else:
            object_cloud_transformed = pmath.transform_points(
                object_cloud, np.linalg.inv(transform))
            obj_dims = np.array([object_cloud_transformed[:, 0].max()-object_cloud_transformed[:, 0].min(),
                                 object_cloud_transformed[:, 1].max(
            )-object_cloud_transformed[:, 1].min(),
                object_cloud_transformed[:, 2].max()-object_cloud_transformed[:, 2].min()])

        obj_length = np.array(obj_dims[:2]).max()
        obj_width = np.array(obj_dims[:2]).min()
        obj_height = np.array(obj_dims[2])

        obj_asp = obj_width/obj_length

        if model_length == -1:
            obj_length = -1
            model_asp = obj_asp
        if model_width == -1:
            obj_width = -1
            model_asp = obj_asp

        height_check = True
        if check_height:
            height_check = False
            if smaller_than:
                if obj_height-model_height <= tolerance:
                    height_check = True
            else:
                if abs(obj_height-model_height) <= tolerance:
                    height_check = True

        if smaller_than:
            if (obj_width-model_width <= tolerance) and \
               (obj_length-model_length <= tolerance) and \
               (abs(obj_asp-model_asp) <= 0.4) and \
               (height_check):
                match = True
        else:
            if (abs(obj_width-model_width) <= tolerance) and \
               (abs(obj_length-model_length) <= tolerance) and \
               (abs(obj_asp-model_asp) <= 0.4) and \
               (height_check):
                match = True
        return match

    def project_point_onto_line(self, x0, y0, z0, x1, y1, z1, x2, y2, z2):
        """
        Projects points onto a vector.

        Parameters
        ----------
        x0: [n,1] array
            X value of points.
        y0: [n,1] array
            Y value of points.
        x1: float
            X value of vector start.
        y1: float
            Y value of vector start.
        x2: float
            X value of vector end.
        y2: float
            Y value of vector end.

        Return
        ------
        vector: [n,2] array
            Array of projected vectors.

        References
        ----------
        reference https://en.wikipedia.org/wiki/Vector_projection

        """

        a = np.vstack([x1, y1, z1]).T
        b = np.vstack([x2, y2, z2]).T
        p = np.vstack([x0, y0, z0]).T

        ap = p-a
        ab = b-a
        return a + np.sum(ap*ab, axis=1)[np.newaxis].T/np.sum(ab*ab, axis=1)[np.newaxis].T * ab

    def check_projection_onto_line_segment(self, x0, y0, z0, x1, y1, z1, x2, y2, z2):
        """
        Finds indexes of valid point projections.

        Parameters
        ----------
        x0: (n,1) array
            X value of point.
        y0: (n,1) array
            Y value of point.
        x1: float
            X value of vector start.
        y1: float
            Y value of vector start.
        x2: float
            X value of vector end.
        y2: float
            Y value of vector end.

        Return
        ------
        indices: [1,k] ndarray
            Array of indices corrisponding to valid projections.
        """
        proj = self.project_point_onto_line(x0, y0, 0, x1, y1, 0, x2, y2, 0)
        if x1 < x2:
            valid_idx = np.where(np.logical_and(
                proj[:, 0] > x1, proj[:, 0] < x2))
        elif x1 > x2:
            valid_idx = np.where(np.logical_and(
                proj[:, 0] < x1, proj[:, 0] > x2))
        return valid_idx

    def point_line_dist(self, x0, y0, x1, y1, x2, y2, check=False):
        """
        Finds minimum distance from points to vector.

        Parameters
        ----------
        x0: [n,1] array
            X value of point.
        y0: [n,1] array
            Y value of point.
        x1: float
            X value of vector start.
        y1: float
            Y value of vector start.
        x2: float
            X value of vector end.
        y2: float
            Y value of vector end.

        Return
        ------
        distance: float
            Minimum distance from point to vector (m).

        References
        ----------
        reference http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
        """
        if check == False:
            dist = np.abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / \
                np.sqrt((x2-x1)**2+(y2-y1)**2)
        elif check == True:
            valid_idxs = self.check_projection_onto_line_segment(
                x0, y0, x1, y1, x2, y2)
            dist = np.abs(
                (x2-x1)*(y1-y0[valid_idxs]) - (x1-x0[valid_idxs])*(y2-y1))/np.sqrt((x2-x1)**2+(y2-y1)**2)
        return dist

    def get_pose_for_CABB(self, box):
        """
        Find the transformation matrix to center allign a box (center of the box at the origin and axes alligned with principle axes).

        Parameters
        ---------
        box: [8,3] ndarray
            Vertices of the bounding box for allignment.\n
            [min_x,min_y,max_z]\n
            [max_x,min_y,max_z]\n
            [min_x,max_y,max_z]\n
            [max_x,max_y,max_z]\n
            [min_x,min_y,min_z]\n
            [max_x,min_y,min_z]\n
            [min_x,max_y,min_z]\n
            [max_x,max_y,min_z]

        Returns
        -------
        transformation_matrix: [4,4] ndarray
            Neccessary transformation matrix to be used to transform box.\n
            [r11,r12,r13,tx]\n
            [r21,r22,r23,ty]\n
            [r31,r32,r33,tz]\n
            [ 0 , 0 , 0 , 1]
        """

        rot = np.array(((box[1]-box[0])/np.linalg.norm(box[1]-box[0]),
                        (box[2]-box[0])/np.linalg.norm(box[2]-box[0]),
                        (box[4]-box[0])/np.linalg.norm(box[4]-box[0])))
        box_rot = box.dot(rot.T)
        transl = (box_rot[0] +
                  0.5*(box_rot[1]-box_rot[0]) +
                  0.5*(box_rot[2]-box_rot[0]) +
                  0.5*(box_rot[4]-box_rot[0]))
        trans_mat = np.eye(4)
        trans_mat[:3, :3] = rot
        trans_mat[:3, 3] = transl
        return trans_mat

    def get_pose_for_AABB(self, box):
        """
        Find the transform matrix to make a list of points axis alligned (corner of the box at the origin)

        Parameters
        ---------
        box: [8,3] ndarray
            Vertices of the bounding box for allignment.

        Returns
        -------
        transformation_matrix: [4,4] ndarray
            Neccessary transformation matrix to be used to transform box.\n
            [r11,r12,r13,tx]\n
            [r21,r22,r23,ty]\n
            [r31,r32,r33,tz]\n
            [ 0 , 0 , 0 , 1]
        """

        rot = np.array(((box[1]-box[0])/np.linalg.norm(box[1]-box[0]),
                        (box[2]-box[0])/np.linalg.norm(box[2]-box[0]),
                        (box[4]-box[0])/np.linalg.norm(box[4]-box[0])))

        trans_mat = np.eye(4)
        trans_mat[:3, :3] = rot
        trans_mat[:3, 3] = box[0, :]
        return trans_mat

    def get_rotation_for_line(self, lA, lB):
        """
        Finds the rotation matrix to allign lA to lB.

        Parameters
        ----------
        lA: [3,] ndarray
            The vector (starting at the origin) that you want to rotate.
        lB: [3,] ndarray
            The vector (starting at the origin) that you want to allign to.

        Returns
        -------
        transformation_matrix: [4,4] ndarray
            Neccessary transformation matrix for line.\n
            [r11,r12,r13,tx]\n
            [r21,r22,r23,ty]\n
            [r31,r32,r33,tz]\n
            [ 0 , 0 , 0 , 1]
        """

        lA_norm = lA/np.linalg.norm(lA)
        lB_norm = lB/np.linalg.norm(lB)

        # Orthagonal vector to A & B (axis of rotation)
        lC = np.cross(lA_norm, lB_norm)
        Cx, Cy, Cz = lC/np.linalg.norm(lC)
        theta = np.arccos(lA_norm.dot(lB_norm))

        sint = np.sin(theta)
        cost = np.cos(theta)

        rot_mat = np.array(([(cost)+(Cx*Cx*(1-cost)),   (Cx*Cy*(1-cost))-(Cz*sint), (Cx*Cz*(1-cost))+(Cy*sint)],
                            [(Cy*Cx*(1-cost))+(Cz*sint), (cost) +
                             (Cy*Cy*(1-cost)),   (Cy*Cz*(1-cost))-(Cx*sint)],
                            [(Cz*Cx*(1-cost))-(Cy*sint), (Cz*Cy*(1-cost))+(Cx*sint), (cost)+(Cz*Cz*(1-cost))]))

        return np.linalg.inv(rot_mat)

    def do_boxes_collide(self, A, B):
        """
        Checks collision between two rectangular prisms. Find the plane of separation with the \
        assumption that it is either parallel to one of the faces of one of the boxes or that it \
        is orthagonal to the cross product of two of the axis of the boxes.

        Parameters
        ----------
        A: [8,3] ndarray
            The vertices for the first box.\n
            [min_x,min_y,max_z]\n
            [max_x,min_y,max_z]\n
            [min_x,max_y,max_z]\n
            [max_x,max_y,max_z]\n
            [min_x,min_y,min_z]\n
            [max_x,min_y,min_z]\n
            [min_x,max_y,min_z]\n
            [max_x,max_y,min_z]
        B: [8,3] ndarray
            The vertices for the second box.\n
            Order same as A.

        Returns
        -------
        collision: bool
            Returns True if the boxes collide.
        """

        x_axis = np.array((1, 0, 0))

        Ax = np.array(A[1]-A[0])/np.linalg.norm(A[1]-A[0])
        Ay = np.array(A[4]-A[0])/np.linalg.norm(A[4]-A[0])
        Az = np.array(A[2]-A[0])/np.linalg.norm(A[2]-A[0])

        Bx = np.array(B[1]-B[0])/np.linalg.norm(B[1]-B[0])
        By = np.array(B[4]-B[0])/np.linalg.norm(B[4]-B[0])
        Bz = np.array(B[2]-B[0])/np.linalg.norm(B[2]-B[0])

        collision_count = 0

        #################################################################################
        #################################################################################
        trans_mat = self.get_pose_for_AABB(A)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)

        # Case 1: L = Ax
        if ((A_al[:, 0].max() > B_al[:, 0].min()) and (A_al[:, 0].min() < B_al[:, 0].max())):
            collision_count += 1

        # Case 2: L = Ay
        if ((A_al[:, 1].max() > B_al[:, 1].min()) and (A_al[:, 1].min() < B_al[:, 1].max())):
            collision_count += 1

        # Case 3: L = Az
        if ((A_al[:, 2].max() > B_al[:, 2].min()) and (A_al[:, 2].min() < B_al[:, 2].max())):
            collision_count += 1

        ##################################################################################

        trans_mat = self.get_pose_for_AABB(B)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)

        # Case 4: L = Bx
        if ((B_al[:, 0].max() > A_al[:, 0].min()) and (B_al[:, 0].min() < A_al[:, 0].max())):
            collision_count += 1

        # Case 5: L = By
        if ((B_al[:, 1].max() > A_al[:, 1].min()) and (B_al[:, 1].min() < A_al[:, 1].max())):
            collision_count += 1

        # Case 6: L = Bz
        if ((B_al[:, 2].max() > A_al[:, 2].min()) and (B_al[:, 2].min() < A_al[:, 2].max())):
            collision_count += 1

        ##################################################################################

        # Case 7: L = Ax X Bx
        trans_mat = self.get_rotation_for_line(np.cross(Ax,Bx),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 8: L = Ax X By
        trans_mat = self.get_rotation_for_line(np.cross(Ax,By),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 9: L = Ax X Bz
        trans_mat = self.get_rotation_for_line(np.cross(Ax,Bz),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 10: L = Ay X Bx
        trans_mat = self.get_rotation_for_line(np.cross(Ay,Bx),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 11: L = Ay X By
        trans_mat = self.get_rotation_for_line(np.cross(Ay,By),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 12: L = Ay X Bz
        trans_mat = self.get_rotation_for_line(np.cross(Ay,Bz),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 13: L = Az X Bx
        trans_mat = self.get_rotation_for_line(np.cross(Az,Bx),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 14: L = Az X By
        trans_mat = self.get_rotation_for_line(np.cross(Az,By),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        # Case 15: L = Az X Bz
        trans_mat = self.get_rotation_for_line(np.cross(Az,Bz),x_axis)
        A_al = A.dot(trans_mat[:3,:3].T)
        B_al = B.dot(trans_mat[:3,:3].T)
        if ((A_al[:,0].max() > B_al[:,0].min()) and (A_al[:,0].min() < B_al[:,0].max())):
            collision_count += 1

        if collision_count == 15:
            collision = True
        else:
            collision = False

        return collision

    def is_point_in_box(self, box, point):
        """
        Check if box will collide with points in cloud.

        Parameters
        ----------
        box: [8,3] ndarray
            Vertices of box.\n
            [min_x,min_y,max_z]\n
            [max_x,min_y,max_z]\n
            [min_x,max_y,max_z]\n
            [max_x,max_y,max_z]\n
            [min_x,min_y,min_z]\n
            [max_x,min_y,min_z]\n
            [min_x,max_y,min_z]\n
            [max_x,max_y,min_z]
        point: [n,3] ndarray
            Coordinates of points.

        Return
        ------
        collision: [n,] ndarray of booleans
            True if the point is within the box.
        """

        AA_pose = self.get_pose_for_AABB(box)
        AABB = pmath.transform_points(box, AA_pose)
        AA_point = pmath.transform_points(point, AA_pose)

        x_check = np.logical_and(
            AA_point[:, 0] >= AABB[:, 0].min(), AA_point[:, 0] <= AABB[:, 0].max())
        y_check = np.logical_and(
            AA_point[:, 1] >= AABB[:, 1].min(), AA_point[:, 1] <= AABB[:, 1].max())
        z_check = np.logical_and(
            AA_point[:, 2] >= AABB[:, 2].min(), AA_point[:, 2] <= AABB[:, 2].max())

        return np.logical_and(x_check, np.logical_and(y_check, z_check))

    def is_point_in_gripper(self, gripper_boxes, point):
        """
        Check if proposed grasp will collide with points in cloud.
        """
        # gripper box 0
        AA_pose0 = self.get_pose_for_AABB(gripper_boxes[0])
        AABB0 = pmath.transform_points(gripper_boxes[0], AA_pose0)
        AA_point0 = pmath.transform_points(point, AA_pose0)
        x_check0 = np.logical_and(
            AA_point0[:, 0] >= AABB0[:, 0].min(), AA_point0[:, 0] <= AABB0[:, 0].max())
        y_check0 = np.logical_and(
            AA_point0[:, 1] >= AABB0[:, 1].min(), AA_point0[:, 1] <= AABB0[:, 1].max())
        z_check0 = np.logical_and(
            AA_point0[:, 2] >= AABB0[:, 2].min(), AA_point0[:, 2] <= AABB0[:, 2].max())
        # gripper box 1
        AA_pose1 = self.get_pose_for_AABB(gripper_boxes[1])
        AABB1 = pmath.transform_points(gripper_boxes[1], AA_pose1)
        AA_point1 = pmath.transform_points(point, AA_pose1)
        x_check1 = np.logical_and(
            AA_point1[:, 0] >= AABB1[:, 0].min(), AA_point1[:, 0] <= AABB1[:, 0].max())
        y_check1 = np.logical_and(
            AA_point1[:, 1] >= AABB1[:, 1].min(), AA_point1[:, 1] <= AABB1[:, 1].max())
        z_check1 = np.logical_and(
            AA_point1[:, 2] >= AABB1[:, 2].min(), AA_point1[:, 2] <= AABB1[:, 2].max())
        # gripper box 2
        AA_pose2 = self.get_pose_for_AABB(gripper_boxes[2])
        AABB2 = pmath.transform_points(gripper_boxes[2], AA_pose2)
        AA_point2 = pmath.transform_points(point, AA_pose2)
        x_check2 = np.logical_and(
            AA_point2[:, 0] >= AABB2[:, 0].min(), AA_point2[:, 0] <= AABB2[:, 0].max())
        y_check2 = np.logical_and(
            AA_point2[:, 1] >= AABB2[:, 1].min(), AA_point2[:, 1] <= AABB2[:, 1].max())
        z_check2 = np.logical_and(
            AA_point2[:, 2] >= AABB2[:, 2].min(), AA_point2[:, 2] <= AABB2[:, 2].max())
        # gripper box 3
        AA_pose3 = self.get_pose_for_AABB(gripper_boxes[3])
        AABB3 = pmath.transform_points(gripper_boxes[3], AA_pose3)
        AA_point3 = pmath.transform_points(point, AA_pose3)
        x_check3 = np.logical_and(
            AA_point3[:, 0] >= AABB3[:, 0].min(), AA_point3[:, 0] <= AABB3[:, 0].max())
        y_check3 = np.logical_and(
            AA_point3[:, 1] >= AABB3[:, 1].min(), AA_point3[:, 1] <= AABB3[:, 1].max())
        z_check3 = np.logical_and(
            AA_point3[:, 2] >= AABB3[:, 2].min(), AA_point3[:, 2] <= AABB3[:, 2].max())

        return np.logical_or(np.logical_or(np.logical_and(x_check0, np.logical_and(y_check0, z_check0)),
                                           np.logical_and(x_check1, np.logical_and(y_check1, z_check1))),
                             np.logical_or(np.logical_and(x_check0, np.logical_and(y_check0, z_check0)),
                                           np.logical_and(x_check1, np.logical_and(y_check1, z_check1))))

    def generate_box(self, t, dim):
        # TODO add origin options

        # Top-center
        if isinstance(t, list) and len(t) == 6:
            t = self.convert_pose_to_transform(t)

        box = np.zeros((8, 3))

        box[0] = [-dim[0]/2, -dim[1]/2, +dim[2]]
        box[1] = [+dim[0]/2, -dim[1]/2, +dim[2]]
        box[2] = [-dim[0]/2, +dim[1]/2, +dim[2]]
        box[3] = [+dim[0]/2, +dim[1]/2, +dim[2]]
        box[4] = [-dim[0]/2, -dim[1]/2, 0]
        box[5] = [+dim[0]/2, -dim[1]/2, 0]
        box[6] = [-dim[0]/2, +dim[1]/2, 0]
        box[7] = [+dim[0]/2, +dim[1]/2, 0]

        box = pmath.transform_points(box, t)
        return box

    def generate_rect(self, t, dim):
        # TODO add origin options

        # Top-center
        if isinstance(t, list) and len(t) == 6:
            t = self.trans_vec_to_matrix(t)

        rec = np.zeros((4, 3))

        rec[0] = [-dim[0]/2, -dim[1]/2, 0]
        rec[1] = [+dim[0]/2, -dim[1]/2, 0]
        rec[2] = [-dim[0]/2, +dim[1]/2, 0]
        rec[3] = [+dim[0]/2, +dim[1]/2, 0]

        rec = pmath.transform_points(rec, t)
        return rec

    def is_cloud_on_fov_edge(self, cloud, tolerance=50):

        depth_img = self.camera.convert_point_cloud_to_depth_image(cloud)
        rows, cols = depth_img.shape
        baseline = 0.055
        hrz_fov = 69.4
        distance = self.get_distance_estimate(cloud.copy())
        # Calculate region of bad data based on distance from scene
        bad_data_ratio = baseline/(2*distance*np.tan(np.deg2rad(hrz_fov/2)))
        bad_data_edge = cols - cols*bad_data_ratio
        return (depth_img[0:tolerance, :] > 0).any() or (depth_img[:, int(bad_data_edge-tolerance):int(bad_data_edge)] > 0).any() or (depth_img[:, 0:tolerance] > 0).any() or (depth_img[-tolerance:, :] > 0).any()

    def remove_cut_off_clouds(self, clouds, tolerance=30):

        for idx, cloud in reversed(list(enumerate(clouds))):
            if self.is_cloud_on_fov_edge(cloud, tolerance):
                del clouds[idx]
        return clouds

    def is_cloud_circular(self, cloud, tol=0.005):
        cloud_pose = self.get_cloud_pose(cloud)
        diams = [0]*36
        for i in range(len(diams)):
            diams[i] = self.get_cloud_width(cloud, cloud_pose)
            cloud_pose = pmath.rotate_pose(cloud_pose, rotation_vec = [0,0,np.deg2rad(5)])
        avg_diam = np.mean(diams)
        diam_spread = abs(diams-avg_diam)
        return (diam_spread <= tol).all()