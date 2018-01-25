import logging
import numpy as np
import math
import sample



__author__ = 'srkiyengar'

LOG_LEVEL = logging.DEBUG

# Set up a logger with output level set to debug; Add the handler to the logger
my_logger = logging.getLogger("UR5_Logger")

#labview_fname = "1140-2017-05-23-at-20-04-18 .txt"
labview_fname = "1408-2017-10-09-at-18-14-19 .txt"


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

def center_tool_339_to_gripper_frame():

    '''
    The y-axis of 339 is aligned with the y axis of the gripper. The z-axis of the 339 will require a rotation of 90
    (counter clockwise with respect to y R (y,90) to get align gripper z axis to outward pointing. the origin of the
    339 needs to be moved in z-axis by + 40.45mm to get it to the origin of the gripper

    :return: homogenous transformation from 339 center to gripper center
    '''

    d =[0.0,0.0,40.45,1.0]
    H = np.zeros((4,4))
    H.shape = (4,4)
    H[:,3]= d
    H[(1,0),(1,2)]=1
    H[2,0]= -1
    return H

def static_transform_449_top(q1,v1,q2,v2):
    '''

    :param q1: unit quaternions representing the rotation of the frame of 449 tool at the top
    :param v1: vector representing the rotation of the frame of 449 tool at the top
    :param q2: unit quaternions representing the rotation of the frame of 339 tool at the center
    :param v2: vector representing the rotation of the frame of 339 tool at the center
    :return: homogenous tranformation
    '''
    # H1 -  Homogenous transform from reference NDI frame to front tool
    # H2 -  Homogenous transform from reference NDI frame to center tool
    # H3 -  Homogenous transformation from the center tool frame to center of the gripper with axis rotated where the y
    # is parallel and between the two fingers and z is pointing outward


    R1 = rotation_matrix_from_quaternions(q1)
    H1 = homogenous_transform(R1, v1)
    h1 = inverse_homogenous_transform(H1)

    R2 = rotation_matrix_from_quaternions(q2)
    H2 = homogenous_transform(R2, v2)

    H3 = center_tool_339_to_gripper_frame()
    H = (h1.dot(H2)).dot(H3)
    return H

def st_from_UR5_base_to_object_platform(Rx,Ry,Rz,x,y,z):

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


if __name__ == '__main__':
    with open(labview_fname) as f:
        lines = f.readlines()

    v1 = [97.663788, -180.389755, -1895.446655]
    q1 = [0.416817, -0.806037, 0.028007, -0.419267]
    v2 = [78.019791, -26.525036, -1980.021118]
    q2 = [0.222542, 0.551251, 0.281243, 0.753326]

    H2 = static_transform_449_top(q1,v1,q2,v2)

    ur5x_origin = 0.1136
    ur5y_origin = -0.121
    ur5z_origin = 0.600

    my_start =  lines[6][51:]
    my_start = my_start[:my_start.rfind("*")-4]
    x_ndi0,y_ndi0,z_ndi0,qr0,qi0,qj0,qk0 = map(float,my_start.split(","))
    print("x0={}, y0={}, z0={}".format(x_ndi0,y_ndi0,z_ndi0))



    R0 = np.zeros((3,3))
    R0[(1,2),(2,0)] = -1
    R0[0,1]=1
    H0 = homogenous_transform(R0,[0.0,0.0,0.0])

    qv = (qr0,qi0,qj0,qk0)
    R1 = rotation_matrix_from_quaternions(qv)
    H1 = homogenous_transform(R1,[x_ndi0,y_ndi0,z_ndi0])
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

        #x = x_ndi - x0
        #y = y_ndi - y0
        #z = z_ndi - z0

        qv = (qr,qi,qj,qk)
        R1 = rotation_matrix_from_quaternions(qv)
        H1 = homogenous_transform(R1,[x_ndi,y_ndi,z_ndi])
        H = H1.dot(H2)

        rH = H0.dot(H)

        R = rH[0:3,0:3]
        x = rH[0,3] - x0
        y = rH[1,3] - y0
        z = rH[2,3] - z0

        # For testing with UR5, aligning the y data of NDI frame to X-axis of the base of UR-5, z to -Y, x to -Z
        # x,y,z now are UR-5 coordinates



        #print("Ux={}, y={}, Uy={}, z={}, Uz={}, x={}".format(ux,uy,uz,y,z,x))
        print("x={:.3f}, y={:.3f}, z={:.3f}".format(x,y,z))
        #Rx,Ry,Rz = sample.quat_to_axis_angle(qr,qi,qj,qk)
        #Rx,Ry,Rz = sample.rotmat_to_axis_angle(R)
        # check
        #Angle = math.sqrt(Rx*Rx + Ry*Ry + Rz*Rz)
        #rx = Rx/Angle
        #ry = Ry/Angle
        #rz = Rz/Angle
        #unit_r = math.sqrt(rx*rx+ry*ry+rz*rz)
        # end of check
        #print("Angle x={}, Unit Vector={}, {}, {}\n".format(Angle,rx,ry,rz))
        #print("Position x={}, y={}, z={},Axis Angles for UR-5 Rx = {}, Ry = {}, Rz = {}".format(ux,uy,uz,Rx,Ry,Rz))
        #print("x={}, y={}, z={}".format(ux,uy,uz))
    #print("Xdiff={}, Ydiff={}, Zdiff={}".format(ux-ur5x_origin,uy-ur5y_origin,uz-ur5z_origin))