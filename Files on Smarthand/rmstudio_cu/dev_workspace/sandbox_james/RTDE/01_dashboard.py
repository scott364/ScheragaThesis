from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from dashboard_client import DashboardClient
import time
from time import sleep
from math import degrees
import numpy as np

robotHost = "192.168.0.101"
ctrl = RTDEControl( robotHost )
rcvr = RTDEReceive( robotHost )
dash = DashboardClient( robotHost )

ctrl.endTeachMode() # Needed before asking the robot to do things programmatically
sleep( 0.25 )

dash.connect()
sleep( 0.25 )

print( "Dashboard Connected?:", dash.isConnected(), ", Is running?:", dash.isConnected() )

#model = dash.getRobotModel()
#print( "Got a model with type:", type( model ) )
#print( model )
""" Got a model with type: <class 'str'>
could not understand: 'get robot model' """

dash.disconnect()