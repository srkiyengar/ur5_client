import logging
import logging.handlers
import numpy as np
import math
import rotmath as rm
import ur5_interface as ur5
import time
import newgripper as ng
import cv2
import sys

__author__ = 'srkiyengar'

LOG_LEVEL = logging.DEBUG

# Set up a logger with output level set to debug; Add the handler to the logger
my_logger = logging.getLogger("UR5_Logger")

result_fname = "781530-modified" # used for Victor's demo


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


def st_from_UR5_base_to_object_platform():
    # The tool center point is placed at the object(table) origin and the following measurements are obtained using the UR-5/
    # The tool center frame z is in opposite direction
    # The object frame y direction is aligned to the y-axis of the tcp (opposite to the stub)
    Rx = 2.1361
    Ry = 2.3107
    Rz = 0.0546
    x = 609.90
    y = 4.51
    z = 103.94

    # the following transformation rotates tool center point coordinate and frame to represent the object origin
    first = [-1,0,0]
    second= [0,1,0]
    third = [0,0,-1]
    R = np.array([first,second,third])
    H = homogenous_transform(R,[0,0,0])
    R1 = rm.axis_angle_to_rotmat(Rx,Ry,Rz)
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
    R = rm.axis_angle_to_rotmat(A[3], A[4], A[5])
    H = homogenous_transform(R, [x, y, z])
    return H

# A is the 6-tuple pose of the gripper w.r.t to the object origin and B is the rotation from the base to the object origin
def base_to_gripper(A,B):
    # A = [x,y,z,Rx,Ry,Rz]
    x = A[0]
    y = A[1]
    z = A[2]
    R = rm.axis_angle_to_rotmat(A[3], A[4], A[5])
    T = homogenous_transform(R, [x, y, z])  # T is A expressed as a homogenous transform
    H = np.dot(B, T)
    x = H[0, 3]
    y = H[1, 3]
    z = H[2, 3]
    R = H[0:3, 0:3]
    Rx, Ry, Rz = rm.rotmat_to_axis_angle(R)
    return[x,y,z,Rx,Ry,Rz]

def way_points_ur5(lines):

    result = 0
    p=[]
    for line in lines:
        p1 = line.strip().split(",")
        p1 = map(float, p1)
        t,x,y,z,Rx,Ry,Rz = p1[0:7]
        f1 = int(p1[7])
        f2 = int(p1[8])
        f3 = int(p1[9])
        f4 = int(p1[10])
        p1 = t,x,y,z,Rx,Ry,Rz,f1,f2,f3,f4
        p.append(p1)
        result +=1

    return result,p

if __name__ == '__main__':

    # This part of the code opens a CV window which overlays a RS image of a selected grasp onto the live
    # video feed of the RS camera. The purpose of this code is to recreate the grasp as accurately as possible
    # by aligning the selected object with the outline of the object depicted in the selected grasp image.
    # The CV window can be zoomed in (CTRL and (SHIFT) +), zoomed  out (CTRL and -), and the transparency can be increased (>) or decreased (<)
    # When alignment is complete, press SPACE bar to move onto the main code.

    cap = cv2.VideoCapture(0)  # RS camera

    '''Enter image number'''
    # num = 422104
    num = 187017
    # The npy files should be in the folder ../trials
    img = np.load('../trials/{}_RS_color.npy'.format(num))

    transparency = 0.6

    while cap.isOpened():

        ret, frame = cap.read()

        k = cv2.waitKey(1)

        # To adjust transparency
        if k % 256 == 62:
            # > pressed
            transparency = transparency + 0.1
        if k % 256 == 60:
            # < pressed
            transparency = transparency - 0.1

        blend = cv2.addWeighted(img, transparency, frame, 0.7, 0)

        cv2.imshow("RS", blend)  # opens CV window with image overlaid on frames

        # To exit image alignment code, press SPACE bar.
        if k % 256 == 32:
            # SPACE pressed
            break

    cap.release()
    cv2.destroyAllWindows()

    # This program and requires id-ur5 file should be in the folder ../trials

    move_file = '../trials/{}-ur5'.format(num)
    try:
        with open(move_file) as f:
            lines = f.readlines()
    except:
        print("unable to open file {}".format(move_file))
        sys.exit(1)




    if("Finger start position" != lines[0][0:21]):
        print("{}-ur5 line 0 should have Finger start position = [.....]".format(move_file))
        sys.exit(1)
    else:
        fing_low_pos_data_col = lines[0][25:51].strip().split(",")
        fing_low_pos_data_col = map(int, fing_low_pos_data_col)
        fing_low_pos_data_col.insert(0,0)
        print "Calibrated position of the gripper at data collection = {}".format(fing_low_pos_data_col)

    result,p = way_points_ur5(lines[1:])

    if result!=2:       #The current assumption is that we are providing only 2 waypoints in the move_file
        sys.exit(1)


    my_gripper = ng.gripper()
    current_finger_low_pos = my_gripper.palm.get_palm_lower_limits()
    print "Current calibrated position of the gripper = {}".format(current_finger_low_pos)


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

    # The function 'st_from_UR5_base_to_object_platform' uses the TCP to the object origin - obtained using the UR5
    # and one more rotation to create the static transformation from the base of ur5 to the object origin
    HT_base_to_object = st_from_UR5_base_to_object_platform()


    # intermediate point of the gripper187017
    #t, x, y, z, Rx, Ry, Rz, f1, f2, f3, f4 = 6.666211, 10.463093, -105.186267, 169.729363, -1.043381, -1.463836, -0.950437, 15640, 13834, 17446, 12866
    t, x, y, z, Rx, Ry, Rz, f1, f2, f3, f4 = p[0]
    x, y, z, Rx, Ry, Rz = base_to_gripper([x,y,z,Rx,Ry,Rz],HT_base_to_object)
    print("x={:.3f}, y={:.3f}, z={:.3f}, Rx={:.3f}, Ry={:.3f}, Rz={:.3f}".format(x, y, z, Rx, Ry, Rz))

    success,command_str1 = ur5.compose_command(x, y, z, Rx, Ry, Rz)
    if success:
        print("Command String: {}".format(command_str1))
        my_logger.info("Sending Command: {}".format(command_str1))
        remote_commander.send(command_str1)


    # Gripping point of the gripper187017
    #t, x, y, z, Rx, Ry, Rz, f1, f2, f3, f4 = 7.082235,5.332551,-81.456898,154.165987,-1.073694,-1.445108,-0.968983,16651,12835,18438,12946
    t, x, y, z, Rx, Ry, Rz, f1, f2, f3, f4 = p[1]
    x, y, z, Rx, Ry, Rz = base_to_gripper([x, y, z, Rx, Ry, Rz], HT_base_to_object)
    print("x={:.3f}, y={:.3f}, z={:.3f}, Rx={:.3f}, Ry={:.3f}, Rz={:.3f}".format(x, y, z, Rx, Ry, Rz))

    success,command_str2 = ur5.compose_command(x, y, z, Rx, Ry, Rz)
    time.sleep(5)
    if success:
        print("Command String: {}".format(command_str2))
        my_logger.info("Sending Command: {}".format(command_str2))
        remote_commander.send(command_str2)
    time.sleep(6)

    # move to grip the object after correcting adjusting the finger movement w.r.t. current calibration
    F_diff = [0,0,0,0,0]
    F_diff[1] = f1 - fing_low_pos_data_col[1]
    F_diff[2] = f2 - fing_low_pos_data_col[2]
    F_diff[3] = f3 - fing_low_pos_data_col[3]
    F_diff[4] = f4 - fing_low_pos_data_col[4]

    my_grip = [0,F_diff[1]+current_finger_low_pos[1],F_diff[2]+current_finger_low_pos[2],F_diff[3]+current_finger_low_pos[3],F_diff[4]+current_finger_low_pos[4]]
    my_grip[4] = 0

    print "Angle travelled {} in ticks".format(F_diff)
    print my_grip
    my_gripper.palm.move_to_goal_position(my_grip)
    time.sleep(2.0)

    #Lift the object
    print("Command String: {}".format(command_str1))
    my_logger.info("Sending Command: {}".format(command_str1))
    remote_commander.send(command_str1)
    time.sleep(6)

    #put it back
    print("Command String: {}".format(command_str2))
    my_logger.info("Sending Command: {}".format(command_str2))
    remote_commander.send(command_str2)
    time.sleep(6)

    #release the object by going back to the lowest position.
    my_gripper.palm.move_to_goal_position(current_finger_low_pos)
    time.sleep(5.0)
    # Sending the tcp back to the start location
    x, y, z, Rx, Ry, Rz = starting_pose
    success,command_str = ur5.compose_command(x, y, z, Rx, Ry, Rz)
    if success:
        print("Command String: {}".format(command_str))
        my_logger.info("Sending Command: {}".format(command_str))
        remote_commander.send(command_str)
        time.sleep(5.0)
    remote_commander.close()
    my_gripper.move_to_start()
