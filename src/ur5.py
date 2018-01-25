__author__ = 'srkiyengar'

import logging
import logging.handlers
from datetime import datetime
import tcp_client as tc
import time
import struct
import math
import sample as s
import sample
import transform as tf
import numpy as np


LOG_LEVEL = logging.DEBUG
LOG_FILENAME = 'ur5' + datetime.now().strftime('%Y-%m-%d---%H:%M:%S')

PORT_SECONDARY_CLIENT = 30002
PORT_REALTIME_CLIENT = 30003
HOST = '192.168.10.2'

# get_UR5_tool_position() to get the pose,seem to require opening and closing the socket.
def get_UR5_tool_position():
    rt_connection = tc.ur5_connector(HOST,PORT_REALTIME_CLIENT)
    my_logger.info("reading packets from UR5")
    message_size = rt_connection.recv(4)                    # Read first 4 bytes only
    total_length = struct.unpack('!I',message_size)[0]      # Total length of the packets following the first 4 bytes
    my_logger.info("Message Size of RT_client response from UR5: {}".format(total_length))
    message = rt_connection.recv(total_length)
    message_len = len(message)
    rt_connection.close()
    if message_len != total_length:
        my_logger.info("Warning !!!! - {} Bytes read NOT equal to {} bytes expected from UR5".format(total_length,message_len))

    tool_pos = message[440:488]                         # 48 bytes consisting of 8 bytes of x,y,z,rx,ry,rz)
    return(struct.unpack('!dddddd',tool_pos))         # A tuple (x,y,z,rx,ry,rz)


def set_UR5_tool_position(tool_pose):
    sc_connection = tc.ur5_connector(HOST,PORT_REALTIME_CLIENT)
    my_logger.info("Sending command to UR5")

    #tool_str = str(list(tool_pose))
    tool_str = tool_pose
    acceleration = (math.pi*4)/9
    velocity = math.pi/3
    command_str = 'movej(' + tool_str + ',' + 'a=' + str(acceleration) + ',v='+ str(velocity) +')\n'
    sc_connection.send(command_str)
    sc_connection.close()
    my_logger.info("Command Sent")



def move_to_pose_base_ref(P):

    x,y,z,rx,ry,rz = get_UR5_tool_position()
    my_logger.info("Current Position x = {}, y = {}, z = {}".format(x,y,z))
    my_logger.info("Current angle Rx = {}, Ry = {}, Rz = {}".format(rx,ry,rz))

    acceleration = (math.pi*4)/9
    velocity = math.pi/3

    x = P[0]
    y = P[1]
    z = P[2]
    rx = P[3]
    ry = P[4]
    rz = P[5]
    pose_str = 'p['+ str(x)+','+ str(y)+','+ str(z)+','+ str(rx)+','+ str(ry)+','+ str(rz)+']' #To strip spaces
    command_str = 'movej(' + pose_str + ',' + 'a=' + str(acceleration) + ',v='+ str(velocity) +')\n'
    return command_str


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




if __name__ == '__main__':

    # Set up a logger with output level set to debug; Add the handler to the logger
    my_logger = logging.getLogger("UR5_Logger")
    my_logger.setLevel(LOG_LEVEL)
    handler = logging.handlers.RotatingFileHandler(LOG_FILENAME, maxBytes=6000000, backupCount=5)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    my_logger.addHandler(handler)
    # end of logfile preparation Log levels are debug, info, warn, error, critical

    starting_pose = get_UR5_tool_position()

    remote_commander = UR5_commander(HOST)

    x =0.57890
    y =0.02322
    z =0.16084
    Rx = 2.1372
    Ry = 2.3716
    Rz = -0.0487
    command_str = move_to_pose_base_ref((x,y,z,Rx,Ry,Rz))
    #command_str = move_to_pose_base_ref(starting_pose)
    print("Command String: {}".format(command_str))
    my_logger.info("Sending Command: {}".format(command_str))
    remote_commander.send(command_str)

    F = s.axis_angle_to_rotmat(Rx,Ry,Rz)
    M = np.zeros((3,3))
    M[0,1] = -1
    M[1,0] = 1
    M[2,2] = 1

    G = np.dot(F, M)
    Rx,Ry,Rz = s.rotmat_to_axis_angle(G)

    command_str = move_to_pose_base_ref((x,y,z,Rx,Ry,Rz))
    #command_str = move_to_pose_base_ref(starting_pose)
    print("Command String: {}".format(command_str))
    my_logger.info("Sending Command: {}".format(command_str))
    remote_commander.send(command_str)


    my_logger.info("Completed")
    remote_commander.close()

    time.sleep(0.5)


    x =0.479
    y =0.07455
    z =0.1195
    Rx = 0.7181
    Ry = 2.5902
    Rz = -1.2104
    command_str = move_to_pose_base_ref((x,y,z,Rx,Ry,Rz))
    #command_str = move_to_pose_base_ref(starting_pose)
    print("Command String: {}".format(command_str))
    my_logger.info("Sending Command: {}".format(command_str))
    remote_commander.send(command_str)
    my_logger.info("Completed")
    remote_commander.close()