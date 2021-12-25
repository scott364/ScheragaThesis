import xmlrpc.client

class RemoteFTclient:
    """ Wrapper class for XML-RPC connection to the OptoForce sensor """
    
    def __init__( self, remoteServerIP, remoteServerPort ):
        """ Set up the connection """
        self.IPdr = remoteServerIP
        self.port = remoteServerPort
        self.addr = 'http://' + str( remoteServerIP ) + ':' + str( remoteServerPort )
        self.prxy = xmlrpc.client.ServerProxy( self.addr )
        print( "Connected to" , self.addr )
        
    def get_wrist_force( self ):
        """ Get a force-torque reading """
        return self.prxy.get_ft()
    
    def bias_wrist_force( self ):
        """ Zero the force-torque sensor """
        return self.prxy.bias_ft()
        