__author__ = 'srkiyengar'
import logging
import logging.handlers
from datetime import datetime
import tcp_client as tc
import time
import struct
import math



#logger
LOG_LEVEL = logging.DEBUG
LOG_FILENAME = 'ur5' + datetime.now().strftime('%Y-%m-%d---%H:%M:%S')

PORT_SECONDARY_CLIENT = 30002
PORT_REALTIME_CLIENT = 30003
HOST = '192.168.10.2'

# get_UR5_tool_position() will open and close the socket everytime a get request is made.

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

def morph_tool_position(my_position,flip):

    xyz_str = list(my_position[0:3])
    if flip == 1:
        Rxyz_str = [1.7380775, 0.65479857, -1.8207649]      #tcp 90 z-axis of tcp
    else:
        Rxyz_str = [-0.7586015, -1.679155, 1.778225]       #std
    return (xyz_str + Rxyz_str)

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

    x,y,z,rx,ry,rz = get_UR5_tool_position()

    acceleration = (math.pi*4)/9
    velocity = math.pi/3

    z = z + 0.20

    pose_str = 'p['+ str(x)+','+ str(y)+','+ str(z)+','+ str(rx)+','+ str(ry)+','+ str(rz)+']' #To strip spaces

    command_str = 'movej(' + pose_str + ',' + 'a=' + str(acceleration) + ',v='+ str(velocity) +')\n'
    #command_str = 'set_digital_out(0,TRUE)\n'

    remote_commander = UR5_commander(HOST)
    if (remote_commander.connection == 1):
        my_logger.info("Sending Command: {}".format(command_str))
        remote_commander.send(command_str)
    else:
        my_logger.info("No link to UR5 to send Command: {}".format(command_str))

    remote_commander.close()
    #h = set_UR5_tool_position(g)

    g = get_UR5_tool_position()

    time.sleep(1.00)
    my_logger.info("Completed")





