########## INIT ###################################################################################

##### Imports #####
import time, threading
from time import sleep

##### Constants #####
_NETPAUSE_S = 0.005 # Sleep for less than 5ms does not allow contexts to switch?


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


class RunUntilAnyT:
    """ Runs a function for as long as `stop_func` returns F """
    
    
    def __init__( self, func_list, callback,  runHz, daemon = 0 , pause_s = 0.10 ):
        """ Set flag """
        self.funcList = func_list
        self.callback = callback
        self.running  = 1
        self.p_daemon = daemon
        self.pause_s  = pause_s
        self.rate     = HeartRate( runHz )
        self.worker   = None
    
    
    def repeat( self ):
        """ Repeat `task` until asked to stop """
        while self.running:
            for f in self.funcList:
                if f.__call__(): # Do not allow implicit `self`
                    print( "COND MET" )
                    self.stop()
                    self.running = 0
                    self.callback.__call__()
                    return None
            self.rate.sleep()
            
            
    def run( self ):
        """ Kick off a thread """
        # NOTE: This function assumes that `task` is a member function that takes no args after `self`
        # 1. Set flag
        self.running = 1
        # 2. Create and start thread
        self.worker = threading.Thread( target = self.repeat, args = (), daemon = self.p_daemon )
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
#         if self.worker.is_alive():
#             self.worker.join()
        
        
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