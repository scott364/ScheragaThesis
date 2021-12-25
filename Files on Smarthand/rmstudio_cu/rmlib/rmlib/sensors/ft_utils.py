########## INIT ###################################################################################

##### Imports #####
import socket, time, threading
from time import sleep

##### Constants #####
_NETPAUSE_S = 0.005 # Sleep for less than 5ms does not allow contexts to switch?
HOME_ADDR   = '127.0.0.1'
HOME_PORT   = 10000



########## GENERAL UTILITIES ######################################################################


class HeartRate:
    """ Sleeps for a time such that the period between calls to sleep results in a frequency <= 'Hz' """
    
    def __init__( self , Hz ):
        """ Create a rate object with a Do-Not-Exceed frequency in 'Hz' """
        self.period = 1.0 / Hz; # Set the period as the inverse of the frequency
        self.last = time.time()
    
    def sleep( self ):
        """ Sleep for a time so that the frequency is not exceeded """
        elapsed = time.time() - self.last
        if elapsed < self.period:
            time.sleep( self.period - elapsed )
        self.last = time.time()


        
class Runner:
    """ Runs a task for as long as it is `running` """

    def __init__( self, runHz, daemon = 0 , pause_s = 0.10 ):
        """ Set flag """
        self.running  = 0
        self.worker   = None
        self.p_daemon = daemon
        self.pause_s  = pause_s
        self.rate     = HeartRate( runHz )
        self.paused   = 0
        
    def set_pause( self, pause = 1 ):
        """ Set the paused status """
        self.paused = pause
        
    def p_paused( self ):
        """ Return whether the `paused` flag was set """
        return bool( self.paused )

    def repeat( self ):
        """ Repeat `task` until asked to stop """
        while self.running:
            if not self.paused:
                self.task()
            self.rate.sleep()
        print( "\nTask stopped!" )

    def run( self ):
        """ Kick off a thread """
        # NOTE: This function assumes that `task` is a member function that takes no args after `self`
        # 1. Set flag
        self.running = 1
        # 2. Create and start thread
        self.worker  = threading.Thread( target = self.repeat, args = (), daemon = self.p_daemon )
        self.worker.start()
        # 3. Allow thread to be scheduled
        sleep( _NETPAUSE_S )

    def stop( self ):
        """ Ask the thread to stop, then join it """
        # sleep( _NETPAUSE_S )
        # 1. Ask the thread to stop
        self.running = 0 
        # 2. Wait for thread to notice
        sleep( _NETPAUSE_S )
        # 3. Join it
        self.worker.join()

    def task( self ):
        """ VIRTUAL PLACEHOLDER """
        raise NotImplementedError( "You must OVERRIDE `self.task`!" )



class UDP_Bidirectional:
    """ Bidirectional communication over UDP """

    def __init__( self, sendIP, sendPort, recvIP, recvPort, commPause_s = _NETPAUSE_S ):
        """ Set up a socket for communication """
        # VARS #
        self.pause = commPause_s
        # SEND #
        self.addrSend = (sendIP, sendPort)
        self.sockSend = socket.socket( socket.AF_INET    , # Internet
                                       socket.SOCK_DGRAM ) # UDP
        self.sockSend.setblocking( 0 )
        # RECV #
        self.addrRecv = (recvIP, recvPort)
        self.sockRecv = socket.socket( socket.AF_INET    , # Internet
                                       socket.SOCK_DGRAM ) # UDP
        self.sockRecv.bind( self.addrRecv ) # A client connects/listens to an address                
        self.sockRecv.setblocking( 0 )


    def send( self, msgBytes ):
        """ Try HARD to send `msgBytes` over outgoing the socket, return number of bytes actually sent """
        dataLen = len( msgBytes )
        numByte = 0
        while dataLen != numByte:
            numByte = self.sockSend.sendto( msgBytes, self.addrSend )
            sleep( self.pause )
        return numByte
            

    def recv( self, numBytes = 1024 ):
        """ Try HARD to read up to `numBytes` from the incoming socket, Return them """
        data = list()
        while not len( data ):
            data = []
            try:
                data = self.sockRecv.recv( numBytes )
            except BlockingIOError:
                sleep( self.pause )
                pass
        return data


    def recv_lim( self, numBytes = 1024, Ntries = 3 ):
        """ Try HARD to read up to `numBytes` from the incoming socket, Return them """
        data = list()
        for i in range( Ntries ):
            try:
                data = self.sockRecv.recv( numBytes )
                if len( data ):
                    break
            except BlockingIOError:
                sleep( self.pause )
        return data


def create_mutual_bidir_UDP( A_IP, A_Port, B_IP, B_Port, commPause_s = _NETPAUSE_S ):
    """ Create to mutual connections to be used by connected agants """
    conn_B = UDP_Bidirectional( A_IP, A_Port, B_IP, B_Port, commPause_s )
    conn_A = UDP_Bidirectional( B_IP, B_Port, A_IP, A_Port, commPause_s )
    return conn_A, conn_B


class AddressDispatcher:
    """ Try to keep the ports straight """

    def __init__( self, bgnAddress = HOME_ADDR, bgnPort = HOME_PORT, commPause_s = _NETPAUSE_S ):
        """ Store the base address and initial port """
        self.addr = bgnAddress
        self.port = bgnPort
        self.paus = commPause_s

    def get_port( self ):
        """ Assign and return a new port """
        self.port += 1
        return self.port

    def get_address( self ):
        """ Assign a new full address for use by a socket """
        return ( self.addr, self.get_port() )

    def connect_mutual_bidir_UDP( self, agent_A, agent_B ):
        """ Give `agent_A` and `agent_B` a mutual connection to each other """
        # NOTE: This function assumes that both agents have a member function `add_connection`
        (AgA_IPAD, AgA_PORT) = self.get_address()
        (AgB_IPAD, AgB_PORT) = self.get_address()
        conn_A, conn_B = create_mutual_bidir_UDP( AgA_IPAD, AgA_PORT, AgB_IPAD, AgB_PORT, self.paus )
        agent_A.add_connection( conn_A )
        agent_B.add_connection( conn_B )