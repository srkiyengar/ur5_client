__author__ = 'srkiyengar'
import logging
import logging.handlers
from datetime import datetime
import tcp_client as tc
import time
import struct
import math
import sample
import transform as tf
import numpy as np

#labview_fname = "1140-2017-05-23-at-20-04-18 .txt"
labview_fname = "1408-2017-10-09-at-18-14-19 .txt"
#logger
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

    with open(labview_fname) as f:
        lines = f.readlines()

    v1 = [97.663788, -180.389755, -1895.446655]
    q1 = [0.416817, -0.806037, 0.028007, -0.419267]
    v2 = [78.019791, -26.525036, -1980.021118]
    q2 = [0.222542, 0.551251, 0.281243, 0.753326]

    H2 = tf.static_transform_449_top(q1,v1,q2,v2)

    ur5x_origin = 200.42
    ur5y_origin = -91.38
    ur5z_origin = 822.55

    my_start =  lines[6][51:]
    my_start = my_start[:my_start.rfind("*")-4]
    x_ndi0,y_ndi0,z_ndi0,qr0,qi0,qj0,qk0 = map(float,my_start.split(","))
    print("NDI frame raw data - x0={:.2f}, y0={:.2f}, z0={:.2f}".format(x_ndi0,y_ndi0,z_ndi0))

    R0 = np.zeros((3,3))
    R0[(1,2),(2,0)] = -1
    R0[0,1]=1
    H0 = tf.homogenous_transform(R0,[0.0,0.0,0.0])

    qv = (qr0,qi0,qj0,qk0)
    R1 = tf.rotation_matrix_from_quaternions(qv)
    H1 = tf.homogenous_transform(R1,[x_ndi0,y_ndi0,z_ndi0])
    H = H1.dot(H2)
    rH = H0.dot(H)

    R = rH[0:3,0:3]
    x0 = rH[0,3]
    y0 = rH[1,3]
    z0 = rH[2,3]


    for line in lines[7:]:
        vq_str = line[51:]
        vq_str = vq_str[:vq_str.rfind("*") - 4]
        x_ndi, y_ndi, z_ndi, qr, qi, qj, qk = map(float, vq_str.split(","))

        qv = (qr,qi,qj,qk)
        R1 = tf.rotation_matrix_from_quaternions(qv)
        H1 = tf.homogenous_transform(R1,[x_ndi,y_ndi,z_ndi])
        H = H1.dot(H2)

        rH = H0.dot(H)

        R = rH[0:3,0:3]
        x = rH[0,3] - x0
        y = rH[1,3] - y0
        z = rH[2,3] - z0

        ux = ur5x_origin + x
        uy = ur5y_origin + y
        uz = ur5z_origin + z

        print("Ux={:.2f},Uy={:.2f},Uz={:.2f},x={:.2f},y={:.2f},z={:.2f}".format(ux,uy,uz,x,y,z))
        Rx,Ry,Rz = sample.rotmat_to_axis_angle(R)
        # check
        #Angle = math.sqrt(Rx*Rx + Ry*Ry + Rz*Rz)
        #rx = Rx/Angle
        #ry = Ry/Angle
        #rz = Rz/Angle
        #unit_r = math.sqrt(rx*rx+ry*ry+rz*rz)
        # end of check
        #print("Angle x={}, Unit Vector={}, {}, {}\n".format(Angle,rx,ry,rz))
        #print("Position x={}, y={}, z={},Axis Angles for UR-5 Rx = {}, Ry = {}, Rz = {}".format(ux,uy,uz,Rx,Ry,Rz))
        # x,y,z for UR-5 in meters
        x = ux/1000
        y = uy/1000
        z = uz/1000
        command_str = move_to_pose_base_ref((x,y,z,Rx,Ry,Rz))
        print("Command String: {}".format(command_str))
        my_logger.info("Sending Command: {}".format(command_str))
        remote_commander.send(command_str)
        time.sleep(0.5)

    command_str = move_to_pose_base_ref(starting_pose)
    print("Command String: {}".format(command_str))
    my_logger.info("Sending Command: {}".format(command_str))
    remote_commander.send(command_str)
    my_logger.info("Completed")
    remote_commander.close()





