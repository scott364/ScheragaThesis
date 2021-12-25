import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pyntcloud import PyntCloud
import warnings

#TODO
# def save_image_to_file(image, file):

# def load_image_from_file(file)
#     return image
    
# def save_cloud_to_file(cloud, file):

# def load_cloud_from_file(file):
#     return cloud

def show_image(image):
    fig, ax = plt.subplots(figsize=(20, 20))
    ax.set_yticks(np.arange(0, 720, 80))
    ax.set_xticks(np.arange(0, 1280, 80))
    ax.imshow(image)
    plt.show()
#     ax.imshow(image, vmin=0, vmax=150, cmap='gist_gray')
    
def show_cloud(cloud, colorize=False, color='rand'):
    """
    View a point cloud.

    Parameters
    ----------
    cloud: [n,3] ndarray
        Point cloud
    """
    axis = [
        {
            "color": "red",
            "hexcolor": "#ff0000",
            "vertices": [[0, 0, 0], [0.1, 0, 0]]
        },
        {
            "color": "green",
            "hexcolor": "#00ff00",
            "vertices": [[0, 0, 0], [0, 0.1, 0]]
        },
        {
            "color": "blue",
            "hexcolor": "#0000ff",
            "vertices": [[0, 0, 0], [0, 0, 0.1]]
        }
    ]

    if cloud.shape[1] == 3 and colorize == True:
        clr = np.zeros(cloud.shape)
        if color is None:
            color = [75, 244, 66]
        elif color == 'rand':
            color = np.random.randint(0, 255, 3)
        clr[:] = color
        cloud = np.hstack((cloud, clr))

    elif cloud.shape[1] == 3 and colorize == False:
        clr = np.zeros(cloud.shape) + 255
        cloud = np.hstack((cloud, clr))
        
    if cloud.shape[1] == 6:
        cloud_df = pd.DataFrame((cloud), columns=['x', 'y', 'z', 'red', 'green', 'blue'])
        cloud_pc = PyntCloud(cloud_df)
    
    warnings.filterwarnings('ignore')
    plotObj = cloud_pc.plot(backend="pythreejs", initial_point_size=0.001, polylines=axis)
    warnings.filterwarnings('always')
    return plotObj

def show_clouds(clouds):
    scene = Scene_3D()
    scene.add_list_of_clouds(clouds)
    scene.render()


#TODO
# class Scene_2D():
    
class Scene_3D():
    def __init__(self):
        self.cloud = None
        self.lines = []
        self.add_empty()
        self.colors = []

    def add_cloud(self, cloud, colorize=False, color='rand'):
        """
        Add point cloud to viewer object.

        Parameters
        ----------
        cloud: [n,3] ndarray
            Point cloud.
        colorize: bool (optional)
            Add color to the cloud (white cloud if false).
        color: [3,] list or color
            RGB color (random as default).
        """
        if cloud.shape[1] == 3 and colorize == True:
            clr = np.zeros(cloud.shape)
            if color is None:
                color = [75, 244, 66]
            elif color == 'rand':
                color = np.random.randint(0, 255, 3)
            clr[:] = color
            cloud = np.hstack((cloud, clr))

        elif cloud.shape[1] == 3 and colorize == False:
            clr = np.zeros(cloud.shape) + 255
            cloud = np.hstack((cloud, clr))

        if cloud.shape[1] == 6:
            if type(self.cloud) == type(None):
                self.cloud = cloud
            else:
                self.cloud = np.vstack((self.cloud, cloud))

    def add_clouds(self, clouds, color_scheme='ordered'):
        clr0 = [43, 160, 44]
        clr1 = [31, 119, 180]
        clr2 = [255, 127, 14]
        clr3 = [214, 39, 40]
        clr4 = [148, 102, 189]
        clr5 = [140, 86, 75]
        clr6 = [227, 118, 194]
        clr7 = [188, 189, 33]
        clr8 = [24, 190, 207]
        clr9 = [42, 250, 125]

        color_list = [clr1, clr2, clr3, clr4, clr5, clr6, clr7, clr8, clr9]
        color_dict_list = []
        color_dict = [
            {
                "colors": []
            }
        ]

        for i, cloud in enumerate(clouds):
            if color_scheme == 'ordered':
                if i >= len(color_list):
                    self.add_cloud(cloud, colorize=True, color=clr0)
                    color_dict_list.append(clr0)

                else:
                    self.add_cloud(cloud, colorize=True,
                                   color=color_list[i])
                    color_dict_list.append(color_list[i])

            elif color_scheme == 'random':
                self.add_cloud(i, colorize=True, color='rand')
                color_dict_list.append(clr0)

            color_dict[0]['colors'] = color_dict_list
            self.colors.append(color_dict)

    def clear(self):
        """
        Clear the scene object.
        """
        self.cloud = None
        self.lines = []
        self.add_empty()
        
    def add_empty(self):
        cloud = np.array(([0, 0, 0]))
        color = np.zeros(cloud.shape)
        cloud = np.hstack((cloud, color))
        self.cloud = cloud

    def add_axis(self, pose):
        """
        Add axis to view object.

        Parameters
        ----------
        transform: [4,4] ndarray
            Transformation matrix describing the axis.
        """
        start_point = np.hstack((np.zeros((3, 3)), np.ones((3, 1))))
        end_point = np.hstack((np.eye(3)*0.025, np.ones((3, 1))))

        start_trans = pose.dot(start_point.T)
        end_trans = pose.dot(end_point.T)

        axis = [
            {
                "color": "red",
                "hexcolor": "#ff0000",
                "vertices": [list(start_trans[:3, 0]), list(end_trans[:3, 0])]
            },
            {
                "color": "green",
                "hexcolor": "#00ff00",
                "vertices": [list(start_trans[:3, 1]), list(end_trans[:3, 1])]
            },
            {
                "color": "blue",
                "hexcolor": "#0000ff",
                "vertices": [list(start_trans[:3, 2]), list(end_trans[:3, 2])]
            }
        ]

        self.lines.extend(axis)
        
    def add_axes(self, poses):
        for pose in poses:
            self.add_axis(pose)

    def add_line(self, vector, start=[0, 0, 0], color="grey"):
        """
        Add line to viewer.

        Parameters
        ----------
        vector: [3,] list
            [x, y, z] values of vector.
        start: [3,] list
            [x, y, z] coordinates of starting point
        color: [3,] list or color
            RGB color (grey as default).
        """
        line = {
            "color": color,
            "vertices": [list(start), list(start+vector)]
        }
        self.lines.append(line)

    def add_box(self, vertices, color='#FFA500'):
        """
        Add box to viewer given vertices.

        Parameters
        ----------
        vertices: [8,3] ndarray
            Vertices of the box to be added.\n
            [min_x,min_y,max_z]\n
            [max_x,min_y,max_z]\n
            [min_x,max_y,max_z]\n
            [max_x,max_y,max_z]\n
            [min_x,min_y,min_z]\n
            [max_x,min_y,min_z]\n
            [min_x,max_y,min_z]\n
            [max_x,max_y,min_z]
        color: [3,] list or color
            RGB color.
        """

        box = [
            {"color": color,
             "vertices": [list(vertices[0]), list(vertices[1])]},
            {"color": color,
             "vertices": [list(vertices[1]), list(vertices[3])]},
            {"color": color,
             "vertices": [list(vertices[3]), list(vertices[2])]},
            {"color": color,
             "vertices": [list(vertices[2]), list(vertices[0])]},

            {"color": color,
             "vertices": [list(vertices[4]), list(vertices[5])]},
            {"color": color,
             "vertices": [list(vertices[5]), list(vertices[7])]},
            {"color": color,
             "vertices": [list(vertices[7]), list(vertices[6])]},
            {"color": color,
             "vertices": [list(vertices[6]), list(vertices[4])]},

            {"color": color,
             "vertices": [list(vertices[0]), list(vertices[4])]},
            {"color": color,
             "vertices": [list(vertices[1]), list(vertices[5])]},
            {"color": color,
             "vertices": [list(vertices[2]), list(vertices[6])]},
            {"color": color,
             "vertices": [list(vertices[3]), list(vertices[7])]}
        ]
        self.lines.extend(box)

    def add_rect(self, vertices, color='#FFD700'):
        """
        Add rectangle to viewer given vertices.

        Parameters
        ----------
        vertices: [8,3] ndarray
            Vertices of the box to be added.\n
            [min_x,min_y,z]\n
            [max_x,min_y,z]\n
            [min_x,max_y,z]\n
            [max_x,max_y,z]
        color: [3,] list or color
            RGB color.
        """
        box = [
            {"color": color,
             "vertices": [list(vertices[0]), list(vertices[1])]},
            {"color": color,
             "vertices": [list(vertices[0]), list(vertices[2])]},
            {"color": color,
             "vertices": [list(vertices[2]), list(vertices[3])]},
            {"color": color,
             "vertices": [list(vertices[3]), list(vertices[1])]}
        ]
        self.lines.extend(box)

    def publish(self, view='camera'):
        """
        Publish the point cloud 
        
        """

        if view == 'camera':
            pose = np.zeros((4, 4))
            pose[0][1] = -1
            pose[1][0] = -1
            pose[2][2] = -1
            pose[3][3] = 1
            pose[2][3] = 0.5

            # Transform points
            temp1 = self.cloud[:, np.array([0, 1, 2])]
            temp2 = self.cloud[:, np.array([3, 4, 5])]

            temp1 = np.hstack((temp1, np.ones((self.cloud.shape[0], 1))))
            temp1 = temp1.dot(pose.T)[:, :3]

            view_cloud = np.hstack((temp1, temp2))

            for line in self.lines:
                points = np.array(line['vertices'][0])
                points = np.hstack((points, np.ones((1,))))
                points = points.dot(pose.T)
                line['vertices'][0] = points.tolist()[:3]

                points = np.array(line['vertices'][1])
                points = np.hstack((points, np.ones((1,))))
                points = points.dot(pose.T)
                line['vertices'][1] = points.tolist()[:3]

        elif view == 'base':
            view_cloud = self.cloud
            self.add_axis(np.eye(4))

        cloud_df = pd.DataFrame((view_cloud), columns=[
                                'x', 'y', 'z', 'red', 'green', 'blue'])
        cloud_pc = PyntCloud(cloud_df)

        return cloud_pc

    def render(self, height=750, width=1200, view='camera'):
        """
        Display the point cloud

        Parameters
        ----------
        height: int
            Height of viewer.
        width: int
            Width of viewer.
        """
        cloud_pc = self.publish(view=view)
        warnings.filterwarnings('ignore')
        plotObj = cloud_pc.plot(initial_point_size=0.001, backend="pythreejs", polylines=self.lines, height=height, width=width)
        warnings.filterwarnings('always')

    def save_to_file(self, file, view='camera'):
        """
        Display the point cloud
       
        """
        cloud_pc = self.publish(view=view)
        cloud_pc.to_file(self.rmstudio_path +
                         'lib/html/rm_dashboard/models/point_cloud.ply')
        df = pd.DataFrame(self.lines)
        df.to_json(self.rmstudio_path +
                   'lib/html/rm_dashboard/models/dem_lines.json')
        clrs = pd.DataFrame(self.colors)
        clrs.to_json(self.rmstudio_path +
                     'lib/html/rm_dashboard/models/dem_colors.json')
        
    #TODO
#     def load_from _file(file)



