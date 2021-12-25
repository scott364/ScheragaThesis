########## INIT ###################################################################################

import sys, os
sys.path.append( os.path.expanduser( "~/dev_rmstudio/rmlib/rmlib/sensors/" ) )

##### Imports #####
import struct, time
from time import sleep
from optoforce import OptoForce_Client
from ft_utils import UDP_Bidirectional, AddressDispatcher, Runner

##### Constants #####
# _NETPAUSE_S = 0.005 # Sleep for less than 5ms does not allow contexts to switch?
HOME_ADDR   = '127.0.0.1'
HOME_PORT   = 10000
FLOATS_FTS  =  6 # -------------------- ( Fx, Fy, Fz, Tx, Ty, Tz )
FLOATS_DAT  = 1+FLOATS_FTS+FLOATS_FTS # ( 1x time + 6x unfiltered + 6x filtered )
DATUM_SIZE  = FLOATS_DAT * 4 # -------- ( FLOATS_DAT ) * ( 4 byte float )
RAWDATSIZE  = FLOATS_FTS * 4

########## NETWORK AGENTS #########################################################################
_NETPAUSE_S = 0.005 # Sleep for less than 5ms does not allow contexts to switch?
REQUEST_RAW = 0


class FT_Server( Runner ):
    """ Get data and repeat it to all registered subscribers """

    def __init__( self, alpha, beta, stepLim = 0.100, dataHz = 50.0 ):
        """ Establish connection with OptoForce """
        self._RAWMODE = REQUEST_RAW

        super().__init__( dataHz )

        self.DG_commnd = struct.Struct('! 2H') # -- 2x uint8, Big-endian (Network)

        ## INPUT: Datastream from OptoForce compute ##
        self.optoClient = OptoForce_Client( alpha, beta, stepLim )
        self.optoClient.prime_optoforce()

        ## Clients ##
        self.clients = list()


    def add_connection( self, conn ):
        """ Create a connection and add it to the collection """
        self.clients.append(  conn  )


    def recv_cmd( self, client ):
        """ Attempt to recieve a datum from the socket """
        rtnDat   = []
        dataLen  = 0

        data    = client.recv_lim( 4, 2 ) 
        dataLen = len( data )

        if dataLen == 4:
            rtnDat = list( self.DG_commnd.unpack( data ) ) 


        return rtnDat


    def handle_listener_command( self, listenerCommand ):
        """ Respond to a command from a client """
        if len( listenerCommand ) == 2:
            # print( "Recevied command:" , listenerCommand, 
            #        "of type" , type(listenerCommand) , "and length" , len(listenerCommand)  )
            cmd = listenerCommand[0]
            # arg = listenerCommand[1]

            ## 0xAAAA : Set wrist bias ##
            if cmd == 0xAAAA:
#                 print( "Broadcaster received a wrist bias command!" )
                self.optoClient.bias_wrist()        
            ## 0xAAAA : Set wrist bias ##
            if cmd == 0xBBBB:
#                 print( "Broadcaster received a wrist bias command!" )
                self.optoClient.update_soft_bias()        


    def task( self ):
        """ Receive datum, then broadcast it to all subscribers """
        # 1. Receive datum
        if self._RAWMODE:
            datumBytes = self.optoClient.raw_datum()
        else:
            datumBytes = self.optoClient.calc_datum()
        # 2. Broadcast it to all subscribers
        for c in self.clients:
            cmd = self.recv_cmd( c )
            if len( cmd ):
                self.handle_listener_command( cmd )
            c.send( datumBytes )
            
            
    def shutdown( self ):
        """ Stop all background threads """
        # NOTE: MUST stop the clients FIRST
        for cl in self.clients:
            cl.stop()
        print( "Stopped all clients!" )
        self.stop()
        print( "Stopped server!" )



class FT_Client( Runner ):
    """ Get data and ask the server for bias """

    def set_connection( self, sendIP, sendPort, recvIP, recvPort ):
        """ Set the connection """
        self.conn = UDP_Bidirectional( sendIP, sendPort, recvIP, recvPort )

    def add_connection( self, conn ):
        """ Set the connection (To be used with `create_mutual_bidir_UDP`) """
        self.conn = conn

    def __init__( self, dataHz = 50.0 ):
        """ Set vars """
        self._RAWMODE = REQUEST_RAW
        super().__init__( dataHz )
        self.flush     = 1
        self.conn      = None
        self.DG_wrench = struct.Struct('! 13f') # ( 1x time + 6x unfiltered + 6x filtered ), Big-endian (Network)
        self.DG_rawdat = struct.Struct('! 6f') # ( 6x unfiltered ), Big-endian (Network)
        self.DG_commnd = struct.Struct('! 2H') # -- 2x uint8, Big-endian (Network)

    def task( self ):
        """ Keep the buffer clear until it is needed """
        if self.flush:
            try:
#                 if self._RAWMODE:
#                     self.conn.recv( RAWDATSIZE ) 
#                 else:
#                     self.conn.recv( DATUM_SIZE ) 
                self.conn.recv( 1024 ) 
            except BlockingIOError as err:
                pass
#         sleep( 0.001 )

    def fetch_datum( self ):
        """ Attempt to recieve a datum from the socket """
        self.flush = 0
        datum = self.DG_wrench.unpack(  self.conn.recv( DATUM_SIZE )  )
        self.flush = 1
        return datum


    def get_wrench_raw( self ):
        """ Return only the current unfiltered wrench """
        
        if self._RAWMODE:
            self.flush = 0
#             sleep( _NETPAUSE_S ) 
            datum = self.DG_rawdat.unpack(  self.conn.recv( RAWDATSIZE )  )
            self.flush = 1
        else:
            datum = self.fetch_datum()[1:7]
        
        return datum
        

    def get_wrench_smooth( self ):
        """ Return only the current filtered wrench """
        datum = self.fetch_datum()
        return datum[7:]


    def bias_wrist( self ):
        """ Ask the wrist to bias itself """
        cmd = self.DG_commnd.pack( 0xAAAA, 0x0000 )
        self.conn.send( cmd )
        
        
    def update_soft_bias( self ):
        """ Ask the wrist to bias itself """
        cmd = self.DG_commnd.pack( 0xBBBB, 0x0000 )
        self.conn.send( cmd )
        