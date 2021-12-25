import sys
print("Python version")
print (sys.version)
print("Version info.")
print (sys.version_info)

import time
from network_data import FT_Listener
import numpy as np

np.set_printoptions( precision = 3 )



audience = list()

def get_listener():
    """ Create a listener, start it's flush process, and return it """
    listener = FT_Listener()
    listener.run_idle()
    return listener

for i in range(4):
    audience.append( get_listener() )

test = 3

for i in range( 4 ):
    for lstnr in audience:
        datum = None
        if test == 1:
            datum = lstnr.fetch_datum()
        elif test == 2:
            datum = lstnr.get_wrench_raw()
        elif test == 3:
            datum = lstnr.get_wrench_smooth()
        print( datum )
    print()
    time.sleep( 1 )