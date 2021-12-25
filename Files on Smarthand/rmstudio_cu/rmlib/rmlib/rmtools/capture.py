import pickle
from datetime import datetime

def load_capture(file):
    capture = pickle.load( open( file, "rb" ) )
    return capture

class Capture:
    def __init__(self):
        self.created_datetime = datetime.now()
        self.data_set_id = None
        self.color_image = None
        self.depth_image = None
        self.cloud = None
        self.clouds = None
        self.camera_pose = None
        self.camera_info = None
        self.robot_capture = None
    
    def save(self, file):
        # Save capture to file
        pickle.dump( self, open( file, "wb" ) )
        pass

        