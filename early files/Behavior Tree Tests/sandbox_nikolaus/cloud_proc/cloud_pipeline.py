from .cloud_capturing import Cloud_Capturing
from .cloud_poses import Cloud_Poses
from .cloud_segmentation import Cloud_Segmentation
from .cloud_processing import Cloud_Processing
from .viewer_threejs import Viewer

inheritance_list = [ Cloud_Capturing , Cloud_Poses , Cloud_Segmentation , Cloud_Processing , Viewer ]

class Cloud_Pipeline( *inheritance_list ):
    
    def __init__( self ):
        
        print( "Cloud_Pipeline object created from components: " , inheritance_list )