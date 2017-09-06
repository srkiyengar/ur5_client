__author__ = 'srkiyengar'
import logging
import logging.handlers
from datetime import datetime
import tcp_client as tc
import time
import struct
import math
import sample

labview_fname = "1140-2017-05-23-at-20-04-18 .txt"

#logger
LOG_LEVEL = logging.DEBUG
LOG_FILENAME = 'ur5' + datetime.now().strftime('%Y-%m-%d---%H:%M:%S')

PORT_SECONDARY_CLIENT = 30002
PORT_REALTIME_CLIENT = 30003
HOST = '192.168.10.2'

# get_UR5_tool_position() will open and close the socket everytime a get request is made.

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
    sc_connection = tc.ur5_connector(HOST,PORT_SECONDARY_CLIENT)
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

    def __init__(self,host=HOST):
        self.commander = tc.ur5_connector(HOST,PORT_SECONDARY_CLIENT)
        if self.commander.my_socket.link == 0:
            raise RuntimeError("No socket connection to UR5")
            self.connection = 0
        else:
            self.connection = 1
        return

    def send(self,command_str):
        self.commander.send(command_str)

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

    remote_commander = UR5_commander(HOST)
    
    #   move_to_pose_base_ref((0.1935,-0.2195, 0.77865,2.184,0.0083,2.2653))
    #   move_to_pose_base_ref((-0.22227,-0.19223, 0.77560,1.2048,-1.2046,1.2145))
    ur5x_origin = -0.26135
    ur5y_origin = -0.11846
    ur5z_origin = 0.70003
    Rx = -0.0928
    Ry = 0.0416
    Rz = -0.0025
    
    #Move to start position
    if (remote_commander.connection == 1):
        command_str = move_to_pose_base_ref((ur5x_origin,ur5y_origin,ur5z_origin,Rx,Ry,Rz))
        my_logger.info("Sending Command: {}".format(command_str))
        remote_commander.send(command_str)
        time.sleep(2)
    else:
        my_logger.info("No link to UR5 to send Command")
    
    with open(labview_fname) as f:
        lines = f.readlines()
		#r00,r01,r02,r03,r10,r11,r12,r13,r20,r21,r22,r23 = ([] for i in range(12))

    my_start =  lines[6][51:]
    my_start = my_start[:my_start.rfind("*")-4]
    x0,y0,z0,_,_,_,_ = map(float,my_start.split(","))

    for line in lines[6:]:
        vq_str = line[51:]
        vq_str = vq_str[:vq_str.rfind("*")-4]
        x,y,z,qr,qi,qj,qk = map(float,vq_str.split(","))
        ur5x = ur5x_origin + (x - x0)/1000
        ur5y = ur5y_origin + (z - z0)/1000
        ur5z = ur5z_origin + (y - y0)/1000
        Rx,Ry,Rz = sample.quat_to_axis_angle(qr,qi,qj,qk)
        print("Position x={}, y={}, z={},Axis Angles Rx = {}, Ry = {}, Rz = {}\n\n".format(ur5x,ur5y,ur5z,Rx,Ry,Rz))
        command_str = move_to_pose_base_ref((ur5x,ur5y,ur5z,Rx,Ry,Rz))
        my_logger.info("Sending Command: {}".format(command_str))
        remote_commander.send(command_str)
        time.sleep(1)
    

    remote_commander.close()
    #h = set_UR5_tool_position(g)
    time.sleep(1.00)
    my_logger.info("Completed")





