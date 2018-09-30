__author__ = 'srkiyengar'
import logging
import logging.handlers
from datetime import datetime
import tcp_client as tc
import time
import struct
import math
import numpy as np
import transform
import rotmath as rm


#labview_fname = "1140-2017-05-23-at-20-04-18 .txt"
labview_fname = "1408-2017-10-09-at-18-14-19 .txt"
#logger
LOG_LEVEL = logging.DEBUG
LOG_FILENAME = 'ur5' + datetime.now().strftime('%Y-%m-%d---%H:%M:%S')

PORT_SECONDARY_CLIENT = 30002
PORT_REALTIME_CLIENT = 30003
HOST = '192.168.10.2'

my_logger = logging.getLogger("UR5_Logger")

# get_UR5_tool_position() to get the pose,seem to require opening and closing the socket.
def get_UR5_tool_position(logger=my_logger):
    rt_connection = tc.ur5_connector(HOST,PORT_REALTIME_CLIENT)
    logger.info("reading packets from UR5")
    message_size = rt_connection.recv(4)                    # Read first 4 bytes only
    total_length = struct.unpack('!I',message_size)[0]      # Total length of the packets following the first 4 bytes
    logger.info("Message Size of RT_client response from UR5: {}".format(total_length))
    message = rt_connection.recv(total_length)
    message_len = len(message)
    rt_connection.close()
    if message_len != total_length:
        logger.info("Warning !!!! - {} Bytes read NOT equal to {} bytes expected from UR5".format(total_length,message_len))

    tool_pos = message[440:488]                         # 48 bytes consisting of 8 bytes of x,y,z,rx,ry,rz)
    my_tuple = struct.unpack('!dddddd',tool_pos)
    my_list = list(my_tuple)
    my_list[0] = 1000*my_list[0]                      # x*1000 meter to mm
    my_list[1] = 1000*my_list[1]                      # meter to mm
    my_list[2] = 1000*my_list[2]                      # meter to mm
    return(my_list)           # A list [x,y,z,rx,ry,rz]


def set_UR5_tool_position(tool_pose,acceleration=((math.pi*4)/9), velocity=(math.pi/3),logger=my_logger):

    my_connection = tc.ur5_connector(HOST,PORT_REALTIME_CLIENT)
    logger.info("Sending command to UR5")

    tool_str = tool_pose
    command_str = 'movej(' + tool_str + ',' + 'a=' + str(acceleration) + ',v='+ str(velocity) +')\n'
    my_connection.send(command_str)
    logger.info("Command Sent")
    my_connection.close()


def compose_command(x, y, z, Rx, Ry, Rz, a=((math.pi*4)/9),v=(math.pi/3)):
    # Rx,Ry,Rz are in radians, x, y ,z are in mm which are converted to meters
    x = x/1000
    y = y/1000
    z = z/1000

    # add verification based on a volume. Reject if outside the volume (result = 0)

    command_str = 'p['+ str(x)+','+ str(y)+','+ str(z)+','+ str(Rx)+','+ str(Ry)+','+ str(Rz)+']' #To strip spaces
    command_str = 'movej(' + command_str + ',' + 'a=' + str(a) + ',v='+ str(v) +')\n'
    result = 1
    return result, command_str

class UR5_commander:

    def __init__(self,host=HOST,port=PORT_REALTIME_CLIENT):
        self.commander = tc.ur5_connector(host,port)
        if self.commander.my_socket.link == 0:
            raise RuntimeError("No socket connection to UR5")
            self.connection = 0
        else:
            self.connection = 1
        return

    def send(self,command_str):
        self.commander.send(command_str)

    def recv(self,num_bytes):
        return self.commander.recv(num_bytes)

    def close(self):
        self.commander.close()


# Temporary testing Homogenous transform
def ht_from_object_to_gripper():
    R = np.zeros((3,3))
    R[0,1] = -1
    R[1,0] = -1
    R[2,2] = -1
    x = 0
    y = 0
    z = 100
    H = transform.homogenous_transform(R,[x,y,z])
    return H

if __name__ == '__main__':

    # Set up a logger with output level set to debug; Add the handler to the logger

    my_logger.setLevel(LOG_LEVEL)
    handler = logging.handlers.RotatingFileHandler(LOG_FILENAME, maxBytes=6000000, backupCount=5)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    my_logger.addHandler(handler)
    # end of logfile preparation Log levels are debug, info, warn, error, critical

    starting_pose = get_UR5_tool_position()
    remote_commander = UR5_commander(HOST)

    #Position of the TCP (close to) Object origin
    Rx = 2.1361
    Ry = 2.3107
    Rz = 0.0546
    x = 609.90
    y = 4.51
    z = 110.94

    # pass the axis angle of the tcp when it is at the object origin reference
    HT_base_to_object = transform.st_from_UR5_base_to_object_platform(x,y,z,Rx,Ry,Rz)

    HT_object_to_gripper = ht_from_object_to_gripper()      #

    H = np.dot(HT_base_to_object,HT_object_to_gripper)

    # Extracting Axis angle from the matrix representing the gripper wrt base of UR5
    R = H[0:3,0:3]
    x = H[0,3]
    y = H[1,3]
    z = H[2,3]
    Rx,Ry,Rz = rm.rotmat_to_axis_angle(R)

    success,command_str = compose_command(x, y, z, Rx, Ry, Rz)
    if success:
        print("Command String: {}".format(command_str))
        my_logger.info("Sending Command: {}".format(command_str))
        remote_commander.send(command_str)
        time.sleep(4.0)

    x, y, z, Rx, Ry, Rz = starting_pose
    success,command_str = compose_command(x, y, z, Rx, Ry, Rz)
    if success:
        print("Command String: {}".format(command_str))
        my_logger.info("Sending Command: {}".format(command_str))
        remote_commander.send(command_str)
        time.sleep(0.5)

    remote_commander.close()





