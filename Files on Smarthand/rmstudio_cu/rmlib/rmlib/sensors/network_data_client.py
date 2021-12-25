"""####### USAGE ##################################################################################

"""

########## INIT ###################################################################################

##### Imports #####
import xmlrpc
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler



########## FT_Client ##############################################################################


class FT_Client:
    """ Get data and ask the server for bias """

    
    def set_connection( self, recvIP, recvPort ):
        """ Set the connection """
        try:
            print( "Attempting to connect to", 'http://' + str( recvIP ) + ':' + str( recvPort ) )
            self.rpCaller = xmlrpc.client.ServerProxy( 'http://' + str( recvIP ) + ':' + str( recvPort ) )
            print( "Connection estasblished!" , self.rpCaller )
        except ConnectionRefusedError as ex:
            print( "Could not establish a connection!", ex )

        
    def __init__( self ):
        """ Set vars """
        self.rpCaller = None
        

    def fetch_datum( self ):
        """ Attempt to recieve a datum from the socket """
        try:
            return self.rpCaller.get_ft()
        except ConnectionRefusedError as ex:
            print( "The connection was refused!", ex )

    def get_wrench_raw( self ):
        """ Return only the current unfiltered wrench """
        try:
            return self.rpCaller.get_ft()
        except ConnectionRefusedError as ex:
            print( "The connection was refused!", ex )
        

    def get_wrench_smooth( self ):
        """ Return only the current filtered wrench """
        try:
            return self.rpCaller.get_ft()
        except ConnectionRefusedError as ex:
            print( "The connection was refused!", ex )


    def bias_wrist( self ):
        """ Ask the wrist to bias itself """
        try:
            self.rpCaller.bias_ft()
        except ConnectionRefusedError as ex:
            print( "The connection was refused!", ex )
        
