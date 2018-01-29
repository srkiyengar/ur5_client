import logging
import logging.handlers
import numpy as np
import math
import sample
import ur5_interface as ur5
import time
import newgripper as ng

__author__ = 'srkiyengar'

LOG_LEVEL = logging.DEBUG

# Set up a logger with output level set to debug; Add the handler to the logger
my_logger = logging.getLogger("UR5_Logger")

result_fname = "781530-modified"


def rotation_matrix_from_quaternions(q_vector):

    '''
    :param q_vector: array, containing 4 values representing a unit quaternion that encodes rotation about a frame
    :return: an array of shape 3x3 containing the rotation matrix.
    Takes in array as [qr, qx, qy, qz]
    https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation, s = 1
    '''

    qr, qi, qj, qk = q_vector
    first = [1-2*(qj*qj+qk*qk), 2*(qi*qj-qk*qr),   2*(qi*qk+qj*qr)]
    second= [2*(qi*qj+qk*qr),   1-2*(qi*qi+qk*qk), 2*(qj*qk-qi*qr)]
    third = [2*(qi*qk-qj*qr),   2*(qj*qk+qi*qr),   1-2*(qi*qi+qj*qj)]
    R = np.array([first,second,third])
    return R


def homogenous_transform(R,vect):

    '''
    :param R: 3x3 matrix
    :param vect: list x,y,z
    :return:Homogenous transformation 4x4 matrix using R and vect
    '''

    H = np.zeros((4,4))
    H[0:3,0:3] = R
    frame_displacement = vect + [1]
    D = np.array(frame_displacement)
    D.shape = (1,4)
    H[:,3] = D
    return H

def inverse_homogenous_transform(H):

    '''
    :param H: Homogenous Transform Matrix
    :return: Inverse Homegenous Transform Matrix
    '''


    R = H[0:3,0:3]
    origin = H[:-1,3]
    origin.shape = (3,1)

    R = R.T
    origin = -R.dot(origin)
    return homogenous_transform(R,list(origin.flatten()))


def st_from_UR5_base_to_object_platform(x,y,z,Rx,Ry,Rz):
    # The tool center frame and the object frame are at the same origin but the tool center z is in opposite direction
    # The object frame y direction has been set by the transformation used to convert the NDI referenced data to object
    # frame referenced data.
    # the following transformation rotates tool center point coordinate and frame to represent the object origin
    first = [-1,0,0]
    second= [0,1,0]
    third = [0,0,-1]
    R = np.array([first,second,third])
    H = homogenous_transform(R,[0,0,0])
    R1 = sample.axis_angle_to_rotmat(Rx,Ry,Rz)
    H1 = homogenous_transform(R1,[x,y,z])
    # H1 represents Homogenous transformation from UR5 base to UR5 tool center point.
    # H represents Homogenous transformation from tool center point to object frame
    # F is the homogenous transformation from base to object frame
    F = np.dot(H1,H)
    return F

# Takes in Axis Angle and build a Homogenous Transform
def ht_of_object_to_gripper(A):
    # A = [x,y,z,Rx,Ry,Rz]

    x = A[0]
    y = A[1]
    z = A[2]
    R = sample.axis_angle_to_rotmat(A[3], A[4], A[5])
    H = homogenous_transform(R, [x, y, z])
    return H

if __name__ == '__main__':

    my_gripper = ng.gripper()
    a = my_gripper.palm.get_palm_lower_limits()
    print a


    # Set up a logger with output level set to debug; Add the handler to the logger
    my_logger = logging.getLogger("UR5_Logger")
    my_logger.setLevel(LOG_LEVEL)
    handler = logging.handlers.RotatingFileHandler(ur5.LOG_FILENAME, maxBytes=6000000, backupCount=5)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    my_logger.addHandler(handler)
    # end of logfile preparation Log levels are debug, info, warn, error, critical

    starting_pose = ur5.get_UR5_tool_position()
    remote_commander = ur5.UR5_commander(ur5.HOST)

    with open(result_fname) as f:
        lines = f.readlines()

    # Position of the TCP (close to) Object origin
    Rx = 2.1361
    Ry = 2.3107
    Rz = 0.0546
    x = 609.90
    y = 4.51
    z = 103.94

    # pass the axis angle of the tcp to obtain the object origin reference wrt to base
    HT_base_to_object = st_from_UR5_base_to_object_platform(x,y,z,Rx,Ry,Rz)

    # starting point of the gripper
    t,x,y,z,Rx,Ry,Rz,f1,f2,f3,f4 = map(float,lines[0].split(','))
    HT_object_to_gripper = ht_of_object_to_gripper([x,y,z,Rx,Ry,Rz])
    H = np.dot(HT_base_to_object,HT_object_to_gripper)
    x = H[0,3]
    y = H[1,3]
    z = H[2,3]
    R = np.zeros((3,3))
    R = H[0:3,0:3]
    Rx,Ry,Rz = sample.rotmat_to_axis_angle(R)
    print("x={:.3f}, y={:.3f}, z={:.3f}, Rx={:.3f}, Ry={:.3f}, Rz={:.3f}".format(x,y,z,Rx,Ry,Rz))
    success,command_str = ur5.compose_command(x, y, z, Rx, Ry, Rz)
    if success:
        print("Command String: {}".format(command_str))
        my_logger.info("Sending Command: {}".format(command_str))
        remote_commander.send(command_str)

    #end point of the gripper
    t,x,y,z,Rx,Ry,Rz,f1,f2,f3,f4 = map(float,lines[(len(lines)-1)].split(','))
    time.sleep(t+5)
    HT_object_to_gripper = ht_of_object_to_gripper([x,y,z,Rx,Ry,Rz])
    H = np.dot(HT_base_to_object,HT_object_to_gripper)
    x = H[0,3]
    y = H[1,3]
    z = H[2,3]
    R = np.zeros((3,3))
    R = H[0:3,0:3]
    Rx,Ry,Rz = sample.rotmat_to_axis_angle(R)
    print("x={:.3f}, y={:.3f}, z={:.3f}, Rx={:.3f}, Ry={:.3f}, Rz={:.3f}".format(x,y,z,Rx,Ry,Rz))
    success,command_str = ur5.compose_command(x, y, z, Rx, Ry, Rz)
    if success:
        print("Command String: {}".format(command_str))
        my_logger.info("Sending Command: {}".format(command_str))
        remote_commander.send(command_str)
    time.sleep(t+3)

    # move to grip the object
    my_grip = [0,f1,f2,f3,f4]
    my_grip = map(int,my_grip)
    print my_grip
    my_gripper.palm.move_to_goal_position(my_grip)
    time.sleep(2.0)
    # release the object by going back to the lowest position.
    my_gripper.palm.move_to_goal_position(a)
    time.sleep(2.0)
    # Sending the tcp back to the start location
    x, y, z, Rx, Ry, Rz = starting_pose
    success,command_str = ur5.compose_command(x, y, z, Rx, Ry, Rz)
    if success:
        print("Command String: {}".format(command_str))
        my_logger.info("Sending Command: {}".format(command_str))
        remote_commander.send(command_str)
        time.sleep(3.0)
    remote_commander.close()