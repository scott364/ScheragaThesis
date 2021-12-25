import os
from os.path import expanduser
import sys


for remPath in [ expanduser( path ) for path in [ "../../rmlib", "../../rmlib/rmlib/rmtools" ] ]:
    print( "Loaded:" , remPath , "Exists?:" , os.path.isdir( remPath ) )
    sys.path.insert( 0 , remPath )

import rmlib
import rmlib.rmtools