########## INIT ###################################################################################
_NETPAUSE_S = 0.005 # Sleep for less than 5ms does not allow contexts to switch?

##### Imports ###################################
import socket, time, struct
from time import sleep
import numpy as np
# from utils import send_datagram

##### Network/Data Params #######################
_NETPAUSE_S = 0.005 # Sleep for less than 5ms does not allow contexts to switch?

## Sensor ##
FTS_IPAD = "192.168.0.3"
FTS_PORT = 49152 # Per the documentation
FTS_ADDR = ( FTS_IPAD, FTS_PORT )

##### Data Params ###############################
FORCE_DIV  =  10000.0 # -------------- Default Force  divide value
TORQUE_DIV = 100000.0 # -------------- Default Torque divide value
FLOATS_FTS =  6 # -------------------- ( Fx, Fy, Fz, Tx, Ty, Tz )
FLOATS_DAT = 1+FLOATS_FTS+FLOATS_FTS # ( 1x time + 6x unfiltered + 6x filtered )
DATUM_SIZE = FLOATS_DAT * 4 # -------- ( FLOATS_DAT ) * ( 4 byte float )
RESPONS_SZ = 36


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

##### OptoForce_Client ##########################


class OptoForce_Client:

    def __init__( self, alpha, beta, stepLim = 0.100 ):
        """ Connect to sockets """
        
        ### Bias ###
        self.ft_thresh = 0.30
#         self.soft_bias = np.zeros( 6 )

        ### Datagrams ###
        self.DG_cmd = struct.Struct('! 2H I') #- 2x uint8, 1x uint32, Big-endian (Network)
        self.DG_res = struct.Struct('! 3I 6i') # 3x uint32, 6x int32, Big-endian (Network)
        self.DG_dat = struct.Struct('! 13f') # -- 13x float32, Big-endian (Network)
        self.DG_raw = struct.Struct('! 6f') # -- 13x float32, Big-endian (Network)
        self.bgnTim = time.time()

        self.COMMANDS = {
            'set_speed' : self.DG_cmd.pack( 0x1234 , #- Header
                                            0x0082 , #- Set speed
                                                10 ), # Speed /* 1000 / SPEED = Speed in Hz */
            'set_filter' : self.DG_cmd.pack( 0x1234 , #- Header
                                             0x0081 , #- Set filter
                                                  0 ), # No filter
            'set_bias_0' : self.DG_cmd.pack( 0x1234 , #- Header
                                             0x0042 , #- Set bias
                                                  0 ), # No bias
            'set_bias_1' : self.DG_cmd.pack( 0x1234 , #- Header
                                             0x0042 , #- Set bias
                                                  1 ), # Yes bias
            'send_10' : self.DG_cmd.pack( 0x1234 , #- Header
                                          0x0002 , #- Data Request
                                              10 ), # Number of samples
            'send_01' : self.DG_cmd.pack( 0x1234 , #- Header
                                          0x0002 , #- Data Request
                                               1 ), # Number of samples
            'send_02' : self.DG_cmd.pack( 0x1234 , #- Header
                                          0x0002 , #- Data Request
                                               2 ), # Number of samples
            'stop_data' : self.DG_cmd.pack( 0x1234 , #- Header
                                            0x0000 , #- Data Request
                                                10 ), # Number of samples
        }

        ### RECV: Sensor Stream ###

        print( "Connecting to the force sensor ...", end = " " )

        self.sock_r = socket.socket( socket.AF_INET    , # Internet
                                     socket.SOCK_DGRAM ) # UDP
        # Set the time-to-live for messages to 1 so they do not go past the local network segment.
        self.ttl_r = struct.pack('b', 1)

        try:
            self.sock_r.connect( FTS_ADDR ) # Clients connect
            self.sock_r.setblocking( 0 )
            print( "SUCCESS!" )
        except socket.error as err:
            print( "Could not connect because:" , err )

        ### Exponential Smoothing ###
        self.A     = alpha
        self.B     = beta
        self.t_lim = stepLim
        self.tm1   = time.time()
        self.s_t   = [ 0.0 for i in range( FLOATS_FTS ) ]
        self.s_tm1 = [ 0.0 for i in range( FLOATS_FTS ) ]
        self.datum = [ 0.0 for i in range( FLOATS_DAT ) ]


    def recv_datum( self ):
        """ Attempt to recieve a datum from the socket """
        rtnDat  = []
        dataLen = 0
        while dataLen != RESPONS_SZ:
            rtnDat  = []
            dataLen = 0
            try:
                data, _ = self.sock_r.recvfrom( RESPONS_SZ ) # buffer size is 1024 bytes
                dataLen = len( data )
                if dataLen == RESPONS_SZ:
                    rtnDat = list( self.DG_res.unpack( data )[3:] ) # Trim off the header
                    for i in range(3):
                        rtnDat[i  ] /= FORCE_DIV
                        rtnDat[i+3] /= TORQUE_DIV
            except BlockingIOError as err:
                pass
#                 sleep( _NETPAUSE_S )
        return np.array( rtnDat )
    
    
#     def bias_datum( self ):
#         """ Use the hardware and software biases together """
#         return np.subtract( self.recv_datum(), self.soft_bias )


    def data_init( self ):
        """ Set up necessary vars before estimation can begin """
        self.s_t = self.recv_datum()
        self.t   = time.time()
        self.b_t = 0.0
        self.dt  = min( self.t - self.tm1, 
                        self.t_lim )


    def prime_optoforce( self ):
        """ Set the optoforce up to begin streaming """
        # Set the optoforce params via UDP commands
        send_datagram( self.sock_r, self.COMMANDS['set_speed']  )
        send_datagram( self.sock_r, self.COMMANDS['set_filter'] )
        send_datagram( self.sock_r, self.COMMANDS['set_bias_0']   )
        # Prime the optoforce and the estimate for streaming
        send_datagram( self.sock_r, self.COMMANDS['send_01'], wait_s = 0.100 ) 
        self.data_init()


    def data_step( self, x_t ):
        """ Update the state estimate and return it """

        # A. Update the t-1 params
        self.tm1   = self.t
        self.s_tm1 = self.s_t
        self.b_tm1 = self.b_t
        self.dtm1  = dtm1 = self.dt

        # B. Compute new params
        t  = time.time()
        dt = min( t - self.tm1, 
                  self.t_lim )
        # Smoothed data
        fctr = dt/dtm1 if (dt < dtm1) else dtm1/dt
        s_t  = self.A*x_t + (1.0-self.A)*(self.s_tm1 + self.b_tm1*fctr) 
        # Smoothed slope
        m_t      = (self.s_t - self.s_tm1)/self.dt
        self.b_t = self.B*m_t*fctr + (1-self.B)*self.b_tm1 

        # C. Cache Data
        self.t   = t
        self.dt  = dt
        self.s_t = s_t

        # Z. Return 
        rtnDat     = np.zeros( FLOATS_DAT )
        rtnDat[0  ] = t # ------ 1x time
        rtnDat[1:7] = x_t # ---- 6x unfiltered
        rtnDat[7: ] = self.s_t # 6x filtered
        return rtnDat


    def calc_datum( self ):
        """ Fetch, Filter, Pack, and Return wrench array """
        # 1. Fetch
        send_datagram( self.sock_r, self.COMMANDS['send_01'], wait_s = 0.001 )
        x_t = self.recv_datum()
#         x_t = self.bias_datum()
        # 2. Filter, Pack, and Return
        return self.DG_dat.pack( *self.data_step( x_t ) )
    
    
    def raw_datum( self ):
        # 1. Fetch
        send_datagram( self.sock_r, self.COMMANDS['send_01'], wait_s = _NETPAUSE_S )
        return self.DG_raw.pack( *self.recv_datum() )
#         return self.DG_raw.pack( *self.bias_datum() )
    
    
    def get_hard_reading( self ):
        """ Reading for use within this object, No software bias required """
        send_datagram( self.sock_r, self.COMMANDS['send_01'], wait_s = _NETPAUSE_S )
        return self.recv_datum()
    
    
    def bias_wrist( self ):
        """ Send the wrist a bias command, wait for actual bias to occur """
        N    = 2
        orig = self.get_hard_reading()
        nPnt = orig
        oLen = np.linalg.norm( orig )
        nLen = oLen
        pLen = oLen/5.0
        if (nLen > pLen) or (nLen > self.ft_thresh):
            for i in range(N):
                if (nLen <= pLen) and (nLen <= self.ft_thresh):
                    break
                send_dg_hard( self.sock_r, self.COMMANDS['set_bias_1'], wait_s = 0.002 )
                nPnt = self.get_hard_reading()
                nLen = np.linalg.norm( nPnt )
                if nLen > self.ft_thresh:
                    send_dg_hard( self.sock_r, self.COMMANDS['set_bias_1'], wait_s = 0.002 )
        
        
#     def update_soft_bias( self ):
#         """ Get an unloaded reading with no soft bias and use it as the new soft bias """
#         softNorm = np.linalg.norm( self.bias_datum() )
#         if softNorm > self.ft_thresh:
#             self.soft_bias = self.get_hard_reading()
            