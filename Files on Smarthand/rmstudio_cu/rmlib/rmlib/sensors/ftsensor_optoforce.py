"""
ftsensor_optoforce.py
2021-06, James Watson for Robotic Materials
XML-RPC Interface for the optoforce, 2021-06-03: Complete re-write
"""

########## INIT ###################################################################################

import sys, os
sys.path.append( os.path.expanduser( "~/dev_rmstudio/rmlib/rmlib/sensors/" ) )

##### Imports #####
import socket, time, struct, json, threading, signal, dill, builtins
from pprint import pprint
from time import sleep
import numpy as np

import multiprocessing, ctypes
from multiprocessing import Pool
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler

from ft_utils import AddressDispatcher, Runner
from remote_FT_client import RemoteFTclient

##### Network/Data Params #######################
_RAWFT_MODE = 1
_NETPAUSE_S = 0.005 # Sleep for less than 5ms does not allow contexts to switch?

## Sensor ##
FTS_PORT = 49152 # Per the documentation


##### Data Params ###############################
FORCE_DIV  =  10000.0 # -------------- Default Force  divide value
TORQUE_DIV = 100000.0 # -------------- Default Torque divide value
FLOATS_FTS =  6 # -------------------- ( Fx, Fy, Fz, Tx, Ty, Tz )
FLOATS_DAT = 1+FLOATS_FTS+FLOATS_FTS # ( 1x time + 6x unfiltered + 6x filtered )
DATUM_SIZE = FLOATS_DAT * 4 # -------- ( FLOATS_DAT ) * ( 4 byte float )
RESPONS_SZ = 36
    

    
########## Socket Communication ###################################################################

def send_datagram( sck, commandBytes, wait_s = 0.020 ):
    """ Send the command over the socket and wait a bit """
    sck.send( commandBytes )
    sleep( wait_s )

def send_dg_hard( sck, commandBytes, wait_s = 0.020 ):
    """ Try hard to send the command over the socket """
    dataLen = len( commandBytes )
    numByte = 0
    while dataLen != numByte:
        numByte = sck.send( commandBytes )
        sleep( wait_s )



########## XML-RPC Helpers ########################################################################


class RequestHandler( SimpleXMLRPCRequestHandler ):
    """ Restrict to a particular path? """
    rpc_paths = ( '/RPC2', )
    
    
class Shared_FT_Data:
    """ Instances update, Class is the authority """
    
    # FIXME: RUN THE ACTUAL TEST
    # * Make sure local and remote are timely and (nearly) identical
    # * Make sure the datum and base are both updated
    # * Record and/or print everything
    
    shared_datum_base = multiprocessing.Array( ctypes.c_double, 6 )
    shared_f_bias     = multiprocessing.Value( ctypes.c_bool, 0 )
    
    @classmethod
    def set_ft( cls, datum ):
        """ Attempt to set a new authoritative datum """
        try:
            if len( datum ) == 6: # Do a quick sanity check
                cls.shared_datum_base[:] = datum
        except ValueError:
            pass
        
    @classmethod
    def do_bias( cls ):
        cls.shared_f_bias = 1
#         print( "Ask for bias!", cls.p_bias() )
        
    @classmethod
    def un_bias( cls ):
#         print( "Don't bias!", cls.p_bias() )
        cls.shared_f_bias = 0
        
    @classmethod
    def p_bias( cls ):
        return bool( cls.shared_f_bias )
    
    # Get Reading #
    def get_ft( self ):
        """ Return the current datum """
#         return Shared_FT_Data.shared_datum 
        shared_datum      = np.ctypeslib.as_array( Shared_FT_Data.shared_datum_base.get_obj() )
        shared_datum      = shared_datum.reshape( 6 )
        return shared_datum.tolist()
    
    # Bias Wrist #
    def bias_ft( self ):
        """ Bias the wrist """
        self.do_bias()
        return 1
    
    
    
def create_ft_xmlrpc_server( configTuple ):
    """ Create an XML-RPC server that is either local or remote """
    # 0. Unpack copnfig
    print( "\n Config Tuple:" , configTuple , '\n' )
    ipad = configTuple['ip']
    port = configTuple['port']
    # 1. Create the XML-RPC object
    server = SimpleXMLRPCServer( ( ipad, port )                    ,
                                 requestHandler = RequestHandler ,
                                 logRequests    = False          )
    # 2. Register the `FT_RegObj` and server query functions
    instance = Shared_FT_Data()
    
    if 0:
        server.register_instance( instance )
    else:
        server.register_function( instance.get_ft  )
        server.register_function( instance.bias_ft )
        
    server.register_introspection_functions()

    # 3. Run the server's main loop (This will be done in its own process)
    print( "XML-RPC serving FT data from", ( ipad, port ) )
    
    if 0:
        while not _BAD_FLAG:
            server.handle_request()
    else:
        server.serve_forever()

    

########## SensorCmd ##############################################################################


class OptoForceCmd:
    """ Container class for OptoForce commands """
    
    ## Datagrams ##
    DG_cmd = struct.Struct('! 2H I') #- 2x uint8, 1x uint32, Big-endian (Network)
    DG_res = struct.Struct('! 3I 6i') # 3x uint32, 6x int32, Big-endian (Network)
    COMMANDS = { 
            'set_speed_100' : DG_cmd.pack( 0x1234 , #- Header
                                           0x0082 , #- Set speed
                                               10 ), # Speed /* 1000 / SPEED = Speed in Hz */
            'set_speed_50' : DG_cmd.pack( 0x1234 , #- Header
                                          0x0082 , #- Set speed
                                              20 ), # Speed /* 1000 / SPEED = Speed in Hz */
            'set_filter_0' : DG_cmd.pack( 0x1234 , #- Header
                                          0x0081 , #- Set filter
                                               0 ), # No filter
            'set_bias_0' : DG_cmd.pack( 0x1234 , #- Header
                                        0x0042 , #- Set bias
                                             0 ), # No bias
            'set_bias_1' : DG_cmd.pack( 0x1234 , #- Header
                                        0x0042 , #- Set bias
                                             1 ), # Yes bias
            'send_10' : DG_cmd.pack( 0x1234 , #- Header
                                     0x0002 , #- Data Request
                                         10 ), # Number of samples
            'send_01' : DG_cmd.pack( 0x1234 , #- Header
                                     0x0002 , #- Data Request
                                          1 ), # Number of samples
            'send_02' : DG_cmd.pack( 0x1234 , #- Header
                                     0x0002 , #- Data Request
                                          2 ), # Number of samples
            'stop_data' : DG_cmd.pack( 0x1234 , #- Header
                                       0x0000 , #- Data Request
                                           10 ), # Number of samples
        }
        
        
    @staticmethod
    def unpack_response( res ):
        """ Unpack the response into a python list """
        return list( OptoForceCmd.DG_res.unpack( res )[3:] ) # Trim off the header

    
########## OptoForce ##############################################################################


class OptoForce( Runner ):
    """ XMLRPC Interface to the OptoForce HEX-H B196 """
    
    
    ##### Init ##################################
    
    
    def connect_to_sensor( self ):
        """ Establish communication with the OptoForce """
        self.sensorAddr = (self.sensorIP, FTS_PORT)
        print( "Connecting to the force sensor at", self.sensorAddr, "...", end = " " )
        self.sock_r = socket.socket( socket.AF_INET    , # Internet
                                     socket.SOCK_DGRAM ) # UDP
        self.sock_r.settimeout(5)
        # Set the time-to-live for messages to 1 so they do not go past the local network segment.
        self.ttl_r = struct.pack('b', 1)
        try:
            self.sock_r.connect( self.sensorAddr ) # Clients connect
            self.sock_r.setblocking( 0 )
            print( "SUCCESS!" )
        except socket.error as err:
            print( "FAILURE: Could not connect because:\n\t" , err )
            
            
    def prime_optoforce( self ):
        """ Set the optoforce up to begin streaming """
        print( "Preparing OptoForce for data transmission ...", end = " " )
        send_datagram( self.sock_r, self.cmd.COMMANDS['set_speed_50']  )
        send_datagram( self.sock_r, self.cmd.COMMANDS['set_filter_0'] )
        send_datagram( self.sock_r, self.cmd.COMMANDS['set_bias_0']   )
        send_datagram( self.sock_r, self.cmd.COMMANDS['send_01'], wait_s = 0.100 ) 
        print( "COMPLETED!" )
    
    
    def __init__( self , initDict , _ ):
        """ Start the force sensor streaming on UDP """
        self._DEBUG = 0
        
        ## Sensor Config ##
        self.params    = initDict
        self.ft_thresh = 0.30
        self.connected = 1 # ----- Set status flag
        self.ft_state  = Shared_FT_Data()
        
        ## INPUT: Sensor Communication Datagrams ##
        self.sensorIP = initDict['ip_address']
        self.cmd      = OptoForceCmd()
        self.maxReq   =  5
        self.cadence  = 10
        self.i        = self.cadence
        self.j        = 0
        
        ## OUTPUT: FT samples to be sent over XML-RPC  ##
        self.dataHz = initDict['dataHz']
        self._lock      = 0 # ---- Prohibits read during update
        
        super().__init__( self.dataHz )
        
        ## XML-RPC ##
        self.localIPdr  = initDict['local_ip']
        self.localPort  = initDict['local_port'] # ----------- Port that loopback neighbors connect to
        self.localTuple = { # ------------------------------ Serves behaviors and RMStudio motion primitives
            'ip' : self.localIPdr, 
            'port' : self.localPort,
        }
        self.remoteIPdr = initDict['remote_ip'] # - IP address that subnet neighbors connect to
        self.remotePort = initDict['remote_port'] # Port that subnet neighbors connect to
        self.remoteTupl = { # --------------------- Serves other machines that want to listen in over network
            'ip' : self.remoteIPdr, 
            'port' : self.remotePort,
        } 
        self.maxPool    = initDict['max_workers']
        self.wrkrPool   = None
        self.wrkrRes    = None
        
        ## GO ##
        self.open_force_sensor()
        
        
        
    ##### Sensor Communication ##################
        
    def recv_datum( self ):
        """ Attempt to recieve a datum from the socket """
        rtnDat  = []
        dataLen = 0
        for i in range( self.maxReq ):
            if dataLen != RESPONS_SZ:
                rtnDat  = []
                dataLen = 0
                try:
                    data, _ = self.sock_r.recvfrom( RESPONS_SZ ) # buffer size is 1024 bytes
                    dataLen = len( data )
                    if dataLen == RESPONS_SZ:
                        rtnDat = self.cmd.unpack_response( data )
                        for i in range(3):
                            rtnDat[i  ] /= FORCE_DIV
                            rtnDat[i+3] /= TORQUE_DIV
                except BlockingIOError as err:
                    pass
            else:
                break
            return rtnDat
        
    
    def force_request_10( self ):
        """ Request 10 FT readings at the set rate """
        send_datagram( self.sock_r, self.cmd.COMMANDS['send_10'], wait_s = _NETPAUSE_S )
        send_datagram( self.sock_r, self.cmd.COMMANDS['send_10'], wait_s = _NETPAUSE_S )
        
        
    def recv_ft( self ):
        """ Fetch one FT reading from the socket buffer """
        datum = self.recv_datum()
        self.ft_state.set_ft( datum )
        if len( datum ) == 6:
            return np.array( datum )
        else:
            return np.array( self.ft_state.get_ft() )
        
        
    def get_wrist_force( self ):
        """ Reading for use within this object, No software bias required """
        send_datagram( self.sock_r, self.cmd.COMMANDS['send_01'], wait_s = _NETPAUSE_S )
        datum = self.recv_datum()
        self.ft_state.set_ft( datum )
        if len( datum ) == 6:
            return np.array( datum )
        else:
            return np.array( self.ft_state.get_ft() )
        

    def bias_wrist_force( self ):
        """ Send the wrist a bias command, wait for actual bias to occur """
        _DEBUG = 0
        if _DEBUG:  print( "About to bias ..." )
        while np.linalg.norm( self.get_wrist_force() ) > self.ft_thresh:
            send_dg_hard( self.sock_r, self.cmd.COMMANDS['set_bias_1'], wait_s = 0.002 )
            if _DEBUG:  print( "biasing!" )
            sleep( 0.25 )
        else:
            if _DEBUG:  print( "bias not needed!" )
        

    
    ##### Data Streaming ########################
        
        
    def task( self ):
        """ Keep the buffer clear until it is needed """
        try:
            
            self.i += 1
            if self.i >= self.cadence:
                send_datagram( self.sock_r, self.cmd.COMMANDS['stop_data'], wait_s = _NETPAUSE_S )
                self.force_request_10()
                self.i = 0
            
            if 1:
                self.recv_ft()
            else:
                # 2021-06-17: No improvement
                self.j += 1
                if self.j % int( self.cadence/2.0 ) == 0:
                    self.recv_ft()
                else:
                    dump = self.sock_r.recvfrom( 1024 ) # buffer size is 1024 bytes
            
            if self.ft_state.p_bias():
                self.bias_wrist_force()
                self.ft_state.un_bias()
                
        except BlockingIOError as err:
            pass
        
        
    def get_ft_data_client( self ):
        """ Return an object that is a client for local force streaming """
        return RemoteFTclient( "localhost", 20000 )
        
        
    def serve( self ):
        """ Serve FT data locally and remotely """
        
        print( "Set up server at:" , self.localTuple )
        print( "Set up server at:" , self.remoteTupl )
        
        # 1. Cache SIGINT handler for this process
        original_sigint_handler = signal.signal( signal.SIGINT, signal.SIG_IGN ) 
        # 2. Init worker pool, does something with signal handlers, maybe?
        self.wrkrPool = Pool( processes = self.maxPool ) # maximum number of requests at once.
        # 3. Restore the cached handler
        signal.signal( signal.SIGINT, original_sigint_handler )
        # 4. This list `map`s to the servers that will be created
        servers_list = [  self.localTuple, self.remoteTupl  ] # 2021-05-28: I think these are the only two that are needed
        # 5. Start XML-RPC and make it available local and remote
        
        def err_cb( ex ):
            """ Error callback for worker pool """
            global _BAD_FLAG
            nonlocal self # Required to reach the enclosing object
            print( "Encountered the following error:\n", ex )
            print( "Terminating workers ...", end = " " )
            self.wrkrPool.terminate()      
#             _BAD_FLAG = 1
            print( "TERMINATED" )
        
        self.wrkrRes = self.wrkrPool.map_async( create_ft_xmlrpc_server, 
                                                servers_list,
                                                error_callback = err_cb )
        
        print( "XML-RPC servers RUNNING!" )
            
            
    def open_force_sensor( self ):
        """ Start the foreground data client """
        self.connect_to_sensor() # Connect
        self.prime_optoforce() # - Set the optoforce params via UDP commands
        self.run() # ------------- Begin accepting data from the sensor
        sleep( 1.0 ) # ----------- Wait a bit, needed?
        self.serve() # ----------- Begin offering data over XML-RPC
        
        
    def close_force_sensor( self ):
        """ Shut down connections """
        self.wrkrPool.terminate() # Shut down all threads
        self.stop() # ------------- Stop streaming
        self.sock_r.close() # ----- Disconnect from sensor
        self.connected = 0 # ------ Set status flag
        
