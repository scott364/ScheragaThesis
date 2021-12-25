import socket , time , struct , json , sys

class OptoForce:
    """ Open a connection to the Optoforce sensor """
    
    def get_init_msg( self ):
        """ Long message to send to the Optoforce at start """
        
        msg = dict()
        
        msg["message_id"] = ""
        msg["command"] = dict()
        msg["command"]["id"] = "configuration"
        msg["command"]["robot_cycle"] = 1
        msg["command"]["sensor_cycle"] = 4
        msg["command"]["max_translational_speed"] = 1.0
        msg["command"]["max_rotational_speed"] = 1.0
        msg["command"]["max_translational_acceleration"] = 1.0
        msg["command"]["max_rotational_acceleration"] = 1.0

        msg = ( json.dumps(msg) + "\r\n\r\n" ).encode()
        
        return msg
    
    def get_bias_msg( self ):
        """ Short message to bias the wrist """
        
        msg = dict()
        
        msg["message_id"] = ""
        msg["command"] = dict()
        msg["command"]["id"] = "bias"

        msg = (json.dumps(msg) + "\r\n\r\n").encode()
        
        return msg
    
    def __init__( self , initDict , _ ):
        """ Open and connect the sockets """
        
        self.force_sensor_ip = initDict[ "ip_address" ]
        self.biasMsg         = self.get_bias_msg()
        self.singleReqMsg    = bytes(2) + b'\x02\x02' + bytes(72)
        self.dataPort        = 32000
        self.cmndPort        = 32002
        self.timeout         = 1
        self._NONBLOCK       = 0
        self.N_bias          = 5
        self.PANIC_TIME      = 0.25
        
        try:
            
            self.dataSock = socket.socket( socket.AF_INET , socket.SOCK_STREAM )
            self.cmndSock = socket.socket( socket.AF_INET , socket.SOCK_STREAM )
            
            self.dataSock.settimeout( 1 )
            self.cmndSock.settimeout( 1 )
            
            self.open_force_sensor()

            self.cmndSock.send( self.get_init_msg() )
            data = self.cmndSock.recv(1024)
            
#             self.set_timeout( 0 ) # On query: [Errno 11] Resource temporarily unavailable
            
        except Exception as err: 
            raise Exception( 'Error Connecting to Force Sensor: ' + str( err ) )
            
    def set_timeout( self , TO_sec = 1 ):
        """ Set the timeout in seconds for the two sockets """
        self.timeout = TO_sec
        self.dataSock.settimeout( TO_sec )
        self.cmndSock.settimeout( TO_sec )
        
    def get_wrist_force(self):
        """
        Retrieves the force and torque vector from the OptoForce.
        
        Returns
        -------
        force_torque_vector: [6,] list
            The first 3 elements desribe the force along each axis, and the second \
            3 describe the torque about each axis.\n
            [Fx, Fy, Fz, Tx, Ty, Tz]
        """
        
        bgn = time.time()
        
        if self._NONBLOCK:
            self.dataSock.send( self.singleReqMsg , socket.MSG_DONTWAIT ) # Attempt non-blocking
            data = self.dataSock.recv( 58 , socket.MSG_DONTWAIT )
        else:
            self.dataSock.send( self.singleReqMsg ) # Timeout is 1 sec
            data = self.dataSock.recv( 58 )
        
        elapsed = ( time.time() - bgn )
        if elapsed >= self.PANIC_TIME:
            print( "PANIC!: Last FT request took >>>" , elapsed , "<<< seconds" )
            
        return list( struct.unpack( '!6f' , data[-24:] ) )

    def bias_wrist_force(self):
        """
        Bias the wrist forces. Call this before expecting a force, so the \
        force sensor is properly calibrated for the gripper's current orientation. 
        """
        for i in range( self.N_bias ):
            try:
                self.cmndSock.send( self.biasMsg )
                data = self.cmndSock.recv(1024)
                break
            except Exception as err:
                print( "bias_wrist_force: Error ~" , err )
        
        return data

    def close_force_sensor(self):
        dataSock.close()
        cmndSock.close()

    def open_force_sensor(self):
        try:
            self.dataSock.connect( ( "{}".format( self.force_sensor_ip ) , self.dataPort ) )
            self.cmndSock.connect( ( "{}".format( self.force_sensor_ip ) , self.cmndPort ) )
            time.sleep( 0.1 )
            self.connected = True
        except Exception as ex:
            print( "Encountered error while trying to connect to the force sensor" , ex )
            self.connected = False
            

    

