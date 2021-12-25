########## INIT ###################################################################################
import sys, os
sys.path.append( os.path.expanduser( "~/dev_rmstudio/rmlib/rmlib/sensors/" ) )

##### Imports #####
import struct, time, threading, signal
from time import sleep
from multiprocessing import Pool
import numpy as np
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
from optoforce import OptoForce_Client
from ft_utils import Runner

##### Constants #####
FLOATS_FTS =     6 # ----------------- ( Fx, Fy, Fz, Tx, Ty, Tz )
FLOATS_DAT = 1+FLOATS_FTS+FLOATS_FTS # ( 1x time + 6x unfiltered + 6x filtered )
DATUM_SIZE = FLOATS_DAT * 4 # -------- ( FLOATS_DAT ) * ( 4 byte float )
RAWDATSIZE = FLOATS_FTS * 4 # -------- Reduced datagram without smoothing

########## NETWORK AGENTS #########################################################################
_NETPAUSE_S = 0.005 # Sleep for less than 5ms does not allow contexts to switch?
REQUEST_RAW = 1



class RequestHandler( SimpleXMLRPCRequestHandler ):
    """ Restrict to a particular path? """
    rpc_paths = ( '/RPC2', )
    
    
    
class FT_RegObj:
    """ Registered object that connects XML-RPC to the OptoForce """
    
    # Attach to server #
    def __init__( self, server ):
        """ Connect this registered object to the server that houses it """
        self.srvr = server
        
    # Get Reading #
    def get_ft( self ):
        """ Return the current datum """
        return self.srvr.currDatum.tolist()
    
    # Bias Wrist #
    def bias_ft( self ):
        """ Bias the wrist """
        self.srvr.optoClient.bias_wrist()
        return 1
    
    

def create_ft_xmlrpc_server( configTuple ):
    """ Create an XML-RPC server that is either local or remote """
    # 0. Unpack copnfig
    ( ipad, port, ft_srvr ) = configTuple
    # 1. Create the XML-RPC object
    server = SimpleXMLRPCServer( ( ipad, port )                    ,
                                 requestHandler = RequestHandler ,
                                 logRequests    = False          )
    # 2. Register the `FT_RegObj` and server query functions
    server.register_instance( FT_RegObj( ft_srvr ) )
    server.register_introspection_functions()

    # 3. Run the server's main loop (This will be done in its own process)
    print( "XML-RPC serving FT data from", ( ipad, port ) )
    server.serve_forever()
    


class FT_Server( Runner ):
    """ Get data and repeat it to all registered subscribers """

    
    def __init__( self, alpha, beta, sensorAddress, localPort = 20000, remotePort = 10000, stepLim = 0.100, dataHz = 50.0,
                        maxWorkerPool = 10 ):
        """ Establish connection with OptoForce """
        
        ## INPUT: Datastream from OptoForce compute ##
        super().__init__( dataHz ) # ---------------------------------------------- Poll optoforce at this rate (See `task`)
        self.optoClient = OptoForce_Client( alpha, beta, sensorAddress, stepLim ) # Create optoforce wrapper
        self.optoClient.prime_optoforce() # --------------------------------------- Init optoforce connections
        
        ## OUTPUT: FT samples to be used by one local  ##
        self._lock      = threading.Lock() # ---- Prohibits read during update
        self.readInput  = np.zeros( 6 ) # ------- Input directly from sensor
        self.currDatum  = np.zeros( 6 ) # ------- Current cached sensor reading
        self.localPort  = localPort # ----------- Port that loopback neighbors connect to
        self.localTuple = ( '127.0.0.1', localPort , self ), # Serves behaviors and RMStudio motion primitives
#         self.localTuple = ( '0.0.0.0', localPort , self ), # Serves behaviors and RMStudio motion primitives
        self.remotePort = remotePort # ---------- Port that subnet neighbors connect to
        self.remoteTupl = ( '', remotePort, self ) # - Serves other machines that want to listen in over network
        self.maxPool    = maxWorkerPool
        

        
    def task( self ):
        """ Keep the buffer clear until it is needed """
        try:
            # 1. Read data
            self.readInput = self.optoClient.raw_datum()
            # 2. Occupy the lock as little as possible
            with self._lock: # NOTE: Any code that modifies `currDatum` must use this lock, reading is OK
                self.currDatum = self.readInput
        except BlockingIOError as err:
            pass
    
    
    def serve( self ):
        """ Serve FT data locally and remotely """
        
        print( "Set up server at:" , self.localTuple )
        print( "Set up server at:" , self.remoteTupl )
        
        # 1. Cache SIGINT handler for this process
        original_sigint_handler = signal.signal( signal.SIGINT, signal.SIG_IGN ) 
        # 2. Init worker pool, does something with signal handlers, maybe?
        multiWorkerPool = Pool( processes = self.maxPool ) # maximum number of requests at once.
        # 3. Restore the cached handler
        signal.signal( signal.SIGINT, original_sigint_handler )
        # 4. This list `map`s to the servers that will be created
        servers_list = [  self.localTuple, self.remoteTupl  ] # 2021-05-28: I think these are the only two that are needed
        # 5. Start XML-RPC and make it available local and remote
        try:
            multiWorkerPool.map( create_ft_xmlrpc_server, servers_list )
        except KeyboardInterrupt:
            print("\nCaught KeyboardInterrupt, terminating workers")
            multiWorkerPool.terminate()      