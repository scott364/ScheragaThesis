########## INIT ###################################################################################

import sys, os
sys.path.append( os.path.expanduser( "~/dev_rmstudio/rmlib/rmlib/sensors/" ) )

##### Imports #####
import socket, time, struct, json
from pprint import pprint
from time import sleep
import numpy as np

from network_data_server import FT_Server
from network_data_client import FT_Client
from ft_utils import AddressDispatcher


### XMLRPC ### XMLRPC ### XMLRPC ### XMLRPC ### XMLRPC ### XMLRPC ### XMLRPC ### XMLRPC ### XMLRPC
COMM_PAUSE = 0.001


class OptoForce:
    """ XMLRPC Interface to the OptoForce HEX-H B196 """
    
    
    def __init__( self , initDict , _ ):
        """ Start the force sensor streaming on UDP """
        self._DEBUG = 0
        
        # Vars #
        self.connected = 0
        self.ft_thresh = 0.50
        # 0. Load stream params
        self.params = initDict
        self.alpha  = initDict['filter']['alpha']
        self.beta   = initDict['filter']['beta']
        self.dataHz = initDict['dataHz']
        # 1. Start the background data server
        print( "\nAbout to start xml-rpc server at" )
        self.srvr = FT_Server( self.alpha, self.beta, initDict['ip_address'], dataHz = self.dataHz )
        print( self.srvr.localTuple, self.srvr.remoteTupl, '\n' ) 
        self.srvr.run()
        # 2. Start the foreground data client
        print( "About to start xml-rpc client" )
        self.clnt = FT_Client()
        self.clnt.set_connection( initDict['local_ip'], initDict['local_port'] )
        sleep( 5.0 )
        self.connected = 1
        self.open_force_sensor()
        self.bias_wrist_force()
        
    
    def open_force_sensor( self ):
        """ Start the foreground data client """
        self.connected = 1

    
    def get_wrist_force( self ):
        """
        Retrieves the force and torque vector from the OptoForce.
        
        Returns
        -------
        force_torque_vector: [6,] list
            The first 3 elements desribe the force along each axis, and the second \
            3 describe the torque about each axis.\n
            [Fx, Fy, Fz, Tx, Ty, Tz]
        """
        return self.clnt.get_wrench_raw()
    
    
    def unbiased_wrist_force( self ):
        """ Retrieves the force and torque vector from the OptoForce, but with no offset """
        return self.clnt.get_wrench_raw()
    
    
    def bias_wrist_force( self ):
        """
        Bias the wrist forces. Call this before expecting a force, so the 
        force sensor is properly calibrated for the gripper's current orientation. 
        """
        if np.linalg.norm( self.get_wrist_force() ) > self.ft_thresh:
            sleep( 0.25 )
            self.clnt.bias_wrist()
            sleep( 3.00 )
        return 1
    
    
    def close_force_sensor( self ):
        """ Shut down connections """
        self.srvr.stop()
        self.connected = 0
        print( "All XML-RPC connections to OptoForce CLOSED!" )
        
        
    
### TCP ### TCP ### TCP ### TCP ### TCP ### TCP ### TCP ### TCP ### TCP ### TCP ### TCP ### TCP ###


class OptoForce_tcp:
    """ TCP Interface to the OptoForce HEX-H B196 """
    
    
    def __init__( self , initDict , _ ):
        """ Construct init info and send to sensor """
        self._DEBUG = 0
        
        # Vars #
        self.force_sensor_ip = initDict[ "ip_address" ]
        
        try:
            self.rti = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            self.ci  = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            self.rti.settimeout(1)
            self.ci.settimeout(1)
            self.open_force_sensor()

            # Is this data even used?
            msg = dict()
            msg["message_id"] = ""
            msg["command"]    = dict()
            msg["command"]["id"]                             = "configuration"
            msg["command"]["robot_cycle"]                    = 1
            msg["command"]["sensor_cycle"]                   = 4
            msg["command"]["max_translational_speed"]        = 1.0
            msg["command"]["max_rotational_speed"]           = 1.0
            msg["command"]["max_translational_acceleration"] = 1.0
            msg["command"]["max_rotational_acceleration"]    = 1.0
            
            if self._DEBUG:
                print( "About to send message:" )
                pprint( msg )

            msg = (json.dumps(msg) + "\r\n\r\n").encode()
            self.ci.send(msg)
            data = self.ci.recv( 1024 )
            self.get_wrist_force()
        except Exception as err: 
            raise Exception('Error Connecting to Force Sensor:' + str( err ) )
            
    def revive_FT_client_socket( self ):
        """ Test for a response an restart """
        # This function may not actually help?
        
        hasErr = True
        loopLm = 10
        i      =  0
        while hasErr and ( i < loopLm ):
            try:
                self.get_wrist_force()
                hasErr = False
            except Exception as err:
                print( "There was some socket error:" , err )
                hasErr = True
            if hasErr:
                print( "Attempt" , i+1 )
                try:
                    self.__init__()
                except Exception as err:
                    print( "While attempting to reconnect, encountered:" , err )
            sleep(1)
            print( "Socket INET:" , socket.AF_INET , '\n' )
            i += 1
        if not hasErr:
            print( "\nSUCCESS in reviving FT socket" )
        else:
            print( "\nFAILURE in reviving FT socket" )
            

    def get_wrist_force( self ):
        """
        Retrieves the force and torque vector from the OptoForce.
        
        Returns
        -------
        force_torque_vector: [6,] list
            The first 3 elements desribe the force along each axis, and the second \
            3 describe the torque about each axis.\n
            [Fx, Fy, Fz, Tx, Ty, Tz]
        """
        try:
            message = bytes(2) + b'\x02\x02' + bytes(72) 
            self.rti.send(message)
            data = self.rti.recv(58)
            return list(struct.unpack('!6f',data[-24:]))
        except:
            return [ float('nan') for i in range(6) ]

        
    def bias_wrist_force( self ):
        """
        Bias the wrist forces. Call this before expecting a force, so the 
        force sensor is properly calibrated for the gripper's current orientation. 
        """
        
        msg = dict()
        msg["message_id"] = ""
        msg["command"] = dict()
        msg["command"]["id"] = "bias"

        msg = (json.dumps(msg) + "\r\n\r\n").encode()
        self.ci.send(msg)
        data = self.ci.recv(1024)
        self.get_wrist_force()
        return data

    
    def close_force_sensor( self ):
        """ Close all TCP connections to the sensor """
        rti.close()
        ci.close()
        self.connected = False
        print( "All TCP connections to OptoForce CLOSED!" )

        
    def open_force_sensor( self ):
        """ Open all TCP connections to the sensor """
        try:
            self.rti.connect( ("{}".format(self.force_sensor_ip), 32000) )
            self.ci.connect(  ("{}".format(self.force_sensor_ip), 32002) )
            time.sleep( 0.1 )
            self.connected = True
        except Exception as ex:
            print( "Encountered error while trying to connect to the force sensor" , ex )
            self.connected = False
            

    

