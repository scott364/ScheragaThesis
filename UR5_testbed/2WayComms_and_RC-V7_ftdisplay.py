bot='red'  #'blue'
print(bot, "robot is being used. Please change the bot variable if this is incorrect")
import socket
import numpy as np
import struct
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import serial
import serial.tools.list_ports
from IPython.display import display, clear_output
from scipy.spatial.transform import Rotation as R
from time import sleep
import math

HOST2 = '192.168.0.103' #'128.138.224.236'
PORT2= 65480

if bot=='red':
    from remote_FT_client import RemoteFTclient
    FTclient = RemoteFTclient( '192.168.0.103', 10000 )
    print( FTclient.prxy.system.listMethods() )

#Arduino Button
ports = serial.tools.list_ports.comports()
#sudo chmod a+rw /dev/ttyACM0
#print(ports)
[port.manufacturer for port in ports]
def find_arduino(port=None):
    """Get the name of the port that is connected to Arduino."""
    if port is None:
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if p.manufacturer is not None and "SparkFun" in p.manufacturer:
                port = p.device
    return port
arduinoport = find_arduino()
print("port=",arduinoport)
arduinoserial = serial.Serial(arduinoport,9600)



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
FT_list_x=[]
FT_list_y=[]
FT_list_z=[]
FT_list_roll=[]
FT_list_pitch=[]
FT_list_yaw=[]
buttonoutputlist=[]
endflag=False

#print( "Bias wrist force" )
#FTclient.bias_wrist_force()
try:
        while True:
            # Send data

            inputstring=0
            #while True:
                #while inputstring!='end':
            inputstring=input("") #Press Enter to continue...
            if (inputstring=='datareset'):
                FT_list_x=[]
                FT_list_y=[]
                FT_list_z=[]
                FT_list_roll=[]
                FT_list_pitch=[]
                FT_list_yaw=[]
                buttonoutputlist=[]
            if "action" in inputstring:
                #actionbyte=struct.pack('ff',0.9887,-.5789) # action[0],action[1])
                actionbyte=struct.pack('dd',0.9887,-.57889) # action[0],action[1])
                sock.send(actionbyte) 

            elif (inputstring=='h' or inputstring=='j' or inputstring=='k' or inputstring=='l' or 
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
                 #request a 0 or 1 from the arduino button   
                arduinoserial.write(b'q\n')  
                arduinobuttonstatus = arduinoserial.readline()

                if arduinobuttonstatus== b'1\r\n':
                    print("BUTTON PRESSED! Episode over!")
                    buttonoutputlist.append(1)
                elif arduinobuttonstatus== b'0\r\n':  #If not
                    print("BUTTON NOT PRESSED")
                    buttonoutputlist.append(0) 

                if bot=='blue':
                    data2 = sock.recv(88)#(64) 
                    while data2== b'':
                        data2 = sock.recv(88)#(64)  #48 bytes
                        #print(data2)
                    #print(data2)

                    unpacked = struct.unpack('ffffffffffffffffffffff', data2)
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

                    forcetorque=[]
                    for j in range(6):
                        forcetorque.append(unpacked[16+j])
                    print("forcetorque:",forcetorque)

                    FT_list_x.append(forcetorque[0])
                    FT_list_y.append(forcetorque[1])
                    FT_list_z.append(forcetorque[2])
                    FT_list_roll.append(forcetorque[3])
                    FT_list_pitch.append(forcetorque[4])
                    FT_list_yaw.append(forcetorque[5])

                    fig = plt.figure()
                    plt.plot(range(len(FT_list_x)),FT_list_x, label="force:x")
                    plt.plot(range(len(FT_list_x)),FT_list_y, label="force:y")
                    plt.plot(range(len(FT_list_x)),FT_list_z, label="force:z")
                    plt.plot(range(len(FT_list_x)),FT_list_roll, label="torque:roll")
                    plt.plot(range(len(FT_list_x)),FT_list_pitch, label="torque:pitch")
                    plt.plot(range(len(FT_list_x)),buttonoutputlist, color='black',label="0/1 button output",linewidth=5.0)

                    plt.show()

                elif bot=='red':
                    data2 = sock.recv(64)#(64) 
                    while data2== b'':
                        data2 = sock.recv(64)#(64)  #48 bytes
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

                    FT_list_x.append(AVG_FT_list[0])
                    FT_list_y.append(AVG_FT_list[1])
                    FT_list_z.append(AVG_FT_list[2])
                    FT_list_roll.append(AVG_FT_list[3])
                    FT_list_pitch.append(AVG_FT_list[4])
                    FT_list_yaw.append(AVG_FT_list[5])

                    fig = plt.figure()
                    plt.plot(range(len(FT_list_x)),FT_list_x, label="force:x")
                    plt.plot(range(len(FT_list_x)),FT_list_y, label="force:y")
                    plt.plot(range(len(FT_list_x)),FT_list_z, label="force:z")
                    plt.plot(range(len(FT_list_x)),FT_list_roll, label="torque:roll")
                    plt.plot(range(len(FT_list_x)),FT_list_pitch, label="torque:pitch")
                    plt.plot(range(len(FT_list_x)),buttonoutputlist, color='black',label="0/1 button output",linewidth=5.0)




    
finally:
    
    print('closing DC socket in .py file')
    sock.close()