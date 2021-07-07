import socket
import numpy as np
import struct
from scipy.spatial.transform import Rotation as R
from remote_FT_client import RemoteFTclient
from time import sleep

HOST2 = '192.168.0.103'
PORT2= 65481

FTclient = RemoteFTclient( '192.168.0.103', 10000 )
#print( FTclient.prxy.system.listMethods() )

def rot2rpy(Rt, degrees=False):
    """ Converts a rotation matrix into rpy / intrinsic xyz euler angle form.
    Args:    Rt (ndarray): The rotation matrix.
    Returns: ndarray: Angles in rpy / intrinsic xyz euler angle form.
    """
    return R.from_matrix(Rt).as_euler('xyz', degrees=degrees)


def rpy2rot(rpy, degrees=False):
    """Converts rpy / intrinsic xyz euler angles into a rotation matrix.
    Args:    rpy (array-like): rpy / intrinsic xyz euler angles.
             degrees (bool) : Whether or not euler_angles are in degrees (True), or radians (False).
    Returns: ndarray: The rotation matrix.
    """
    return R.from_euler('xyz', rpy, degrees=degrees).as_matrix()


# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = (HOST2, PORT2)
print('connecting to {} port {}'.format(*server_address))
sock.connect(server_address)


print( "Bias wrist force" )
FTclient.bias_wrist_force()
try:
     while True:
        # Send data

        inputstring=0
        #while True:
            #while inputstring!='end':
        inputstring=input("") #Press Enter to continue...
        if (inputstring=='h' or inputstring=='j' or inputstring=='k' or inputstring=='l' or 
            inputstring=='u' or inputstring=='o'or inputstring=='y' or inputstring=='i' or 
            inputstring=='z' or inputstring=='x' or inputstring=='d' or inputstring=='c'or
            inputstring=='home',inputstring=='end',inputstring=='obs'):

            data1=inputstring.encode('ascii')    
            sock.sendall(data1)
        if (inputstring=='end'):
            print('end episode')
            break
        """
        data = sock.recv(16)
            #amount_received += len(data)
        print('received {!r}'.format(data))
        """
        if (inputstring=='obs'):
            data = sock.recv(64)  #48 bytes
            #print(data)
            #value = struct.unpack('f',data)
            #print(data)
            while data==b'':
                data = sock.recv(64)  
            #print(data)   
            unpacked = struct.unpack('ffffffffffffffff', data)
            #print(unpacked)
            #print('Received', repr(data))
            TransRotatmatrix=np.zeros([4,4])
            for i in range(4):
                TransRotatmatrix[i][:]=unpacked[0+(4*i):4+(4*i)]
            #print(TransRotatmatrix)
            rpy=rot2rpy(TransRotatmatrix[0:3,0:3],True)
            x=TransRotatmatrix[0][3]*39.3701  #convert to inches 
            y=TransRotatmatrix[1][3]*39.3701
            z=TransRotatmatrix[2][3]*39.3701
            roll=rpy[0]  #In degrees!
            pitch=rpy[1]
            yaw=rpy[2]
            print("x",x,"y",y,"z",z)
            print("roll",roll,"pitch",pitch,"yaw",yaw)

            """"""
            forcesamples=5
            FT_list=[]
            AVG_FT_list=[0]*6  #3 forces 3 torques
            for i in range(5):  #get 5 force samples
                FT_list.append(FTclient.get_wrist_force())
                sleep( 0.020 )            
            for i in range(len(FT_list)): 
                #print("i:",i)
                AVG_FT_list[i]=0
                for j in range(len(FT_list)):  
                    AVG_FT_list[i]+=FT_list[j][i]    
                AVG_FT_list[i]=AVG_FT_list[i]/(len(FT_list))
            print("Forces and Torques:")
            print(AVG_FT_list)
            #print( "Bias wrist force" )
            #FTclient.bias_wrist_force()
            #sleep( 3.0 )
            """"""
    
finally:
    
    print('closing DC socket in .py file')
    sock.close()