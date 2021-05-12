import os
from os.path import expanduser
import sys


for remPath in [ expanduser( path ) for path in [ "~/dev_rmstudio/rmlib/rmlib/rmtools" , "~/dev_rmstudio/rmlib" ] ]:
    print( "Loaded:" , remPath , "Exists?:" , os.path.isdir( remPath ) )
    sys.path.insert( 0 , remPath )

import rmlib
import rmlib.rmtools
import rmlib.rmtools.assembly_trees