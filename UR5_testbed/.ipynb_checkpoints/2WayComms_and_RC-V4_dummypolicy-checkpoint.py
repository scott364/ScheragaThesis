import socket
import numpy as np
import struct
from scipy.spatial.transform import Rotation as R
from remote_FT_client import RemoteFTclient
from time import sleep
import math

HOST2 = '192.168.0.103'
PORT2= 65483

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
            inputstring!='home' or inputstring!='end' or inputstring!='obs' or inputstring != b'fetch' 
                or inputstring != b'return' or inputstring != b'reset' ):

            data1=inputstring.encode('ascii')    
            sock.sendall(data1)
            
            
        if (inputstring == 'fetch'  or inputstring == 'return' or inputstring == 'reset' ):
            print("waiting for done response")
            data2 = sock.recv(64) #receive the "done" command
            while data2== b'':
                data2 = sock.recv(64)  #48 bytes
            print(data2)

            
            
        if (inputstring=='end'):
            print('end episode')
            break
        if(inputstring=='bias'):
            FTclient.bias_wrist_force()
            print("biased wrist force ")
  
        """
        data = sock.recv(16)
            #amount_received += len(data)
        print('received {!r}'.format(data))
        """
        if (inputstring=='obs'):
            data2 = sock.recv(64) 
            while data2== b'':
                data2 = sock.recv(64)  #48 bytes
                #print(data2)
            #print(data2)
            unpacked = struct.unpack('ffffffffffffffff', data2)
            #if self.totalstepstaken>=410:
            #    print("unpacked data: ",unpacked)
            TransRotatmatrix=np.zeros([4,4])
            for i in range(4):
                TransRotatmatrix[i][:]=unpacked[0+(4*i):4+(4*i)]

                
                
            #print(TransRotatmatrix)
            rpy=rot2rpy(TransRotatmatrix[0:3,0:3],True)
            currentX=TransRotatmatrix[0][3]*39.3701  #convert to inches 
            currentY=TransRotatmatrix[1][3]*39.3701
            currentZ=TransRotatmatrix[2][3]*39.3701
            roll=rpy[0]  #In degrees!
            pitch=rpy[1]
            yaw=rpy[2]
            print("currentpose: x",currentX,"y",currentY,"z",currentZ)
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
            
            
            #Reward calculation--------------------------------------------------
            #Default initial Z is 0.06915259
            #half inserted peg Z is 0.044 ish
            
            """
            cylinder above target pos:
            """
            initial=np.array(
            [[-0.99934544,-0.00730877  ,.0354299 ,-0.19068683],
            [-0.00689006 ,0.99990515 ,0.01192549,-0.49560643],
            [-0.0355137  ,0.01167357,-0.99930101 ,0.10501393],
            [ 0.        , 0.         ,0.        , 1.        ]])
            
            initialX=initial[0][3]*39.3701  #convert to inches 
            initialY=initial[1][3]*39.3701
            initialZ=initial[2][3]*39.3701
            initialZ=2.72
            print(" ")
            print("Reward")
            #a=self._compute_observation()
            #[currentX,currentY,currentZ]=self.currentpose  #in inches 


            #Default initial Z is 2.72
            #half inserted peg Z is 0.044 ish

            #[initialX,initialY,initialZ]=self.episodeinitialpose #in inches 

            #self.currentreward=0 
            currentreward=-1* math.sqrt(pow(currentX-initialX,2)+pow(currentY-initialY,2))  #2D distance formula
            print("currentreward (XY dist to initial point, no bonus:",currentreward)
            print("InitialZ:",initialZ, "currentZ:",currentZ)
            #check for success condition, and if success, add bonus reward :)
            if initialZ-currentZ>0.72: #1 inch  #DOUBLE CHECK THAT  THIS IS IN INCHES NOT METERS OR MM!!!!
                print("success condition achieved  InitialZ:",initialZ, "currentZ:",currentZ)
                print("abs z dist=",abs(initialZ-currentZ))
                print("bonusreward=1-(self._envStepCounter/self.StepsPerEpisode)")
            
            """"""
    
finally:
    
    print('closing DC socket in .py file')
    sock.close()