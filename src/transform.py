__author__ = 'srkiyengar'

import logging.handlers
import numpy as np
import rotmath as rm
import ur5_interface as ur5
import time
import newgripper as ng
import cv2
import os
import sys
import match
from datetime import datetime



scriptname = os.path.basename(__file__)
LOG_LEVEL = logging.INFO
LOG_FILENAME = 'ur5' + datetime.now().strftime('%Y-%m-%d---%H:%M:%S')
LOG_FILENAME2 = 'ur5-grasps-' + datetime.now().strftime('%Y-%m-%d---%H:%M:%S')

# Set up a logger with output level set to debug; Add the handler to the logger
my_logger = logging.getLogger("UR5_Logger")

result_fname = "781530-modified" # used for Victor's demo

# The rectangle parallelepiped or right rectangular prism is the volume through with the TCP (in our case plus 6 cm in +z-axis)
# can move with respect to the UR-5 base. The measurements are in mm.
x_min = 290
x_max = 700
y_min = -480
y_max = +480
z_min = 165
z_max = 800

# To sanity check the maximum finger movements
MLIMIT  = 0   #some arbitary reduction to the maximum allowed finger movement due to paranoia
MAX_FINGER_MOVEMENT = 2700
MAX_PRESHAPE_MOVEMENT = 800

my_dir = "../trials"


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
    # The following were measured in the old lab
    #Rx = 2.1361
    #Ry = 2.3107
    #Rz = 0.0546
    #x = 609.90
    #y = 4.51
    #z = 103.94
    #The following were measured in the new lab on 31 Oct. 2018
    Rx = 2.2949
    Ry = 2.1925
    Rz = 0.0041
    x = 590.74
    y = 9.52
    z = 88.85

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

    return len(p),p

def check_position(x,y,z):

    if z_min <= z <= z_max:
        if y_min <= y <= y_max:
            if x_min <= x <= x_max:
                return 1,"Good"
            else:
                return 0,"x = {} out of range, y and z good".format(x)
        else:
            return 0,"y = {} out of range, z good".format(y)
    else:
        return 0,"z = {} out of range".format(z)


def check_if_finger_movement_excessive(finger_angle):

    max_limit_f123 = MAX_FINGER_MOVEMENT-MLIMIT
    max_limit_f4 = MAX_PRESHAPE_MOVEMENT-MLIMIT

    f = map(abs,finger_angle)       #fingers f1 and f3 increase during gripping whereas f2 and f4 decrease

    if f[1] > max_limit_f123 or f[2] > max_limit_f123 or f[3] > max_limit_f123:
        return 0, "finger angels 1 or 2 or 3 {} exceeds safety limit".format(f)
    elif f[4] > max_limit_f4:
        return 0,"finger angel 4 exceeds safety limit"
    else:
        return 1, "Good"


if __name__ == '__main__':

    # Set up a logger with output level set to debug; Add the handler to the logger
    my_logger = logging.getLogger("UR5_Logger")
    my_logger.setLevel(LOG_LEVEL)
    handler = logging.handlers.RotatingFileHandler(LOG_FILENAME, maxBytes=6000000, backupCount=5)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    my_logger.addHandler(handler)
    # end of logfile preparation Log levels are debug, info, warn, error, critical

    # Results Logger
    result_logger = logging.getLogger("Results_Logger")
    result_logger.setLevel(LOG_LEVEL)
    handler = logging.handlers.RotatingFileHandler(LOG_FILENAME2, maxBytes=6000000, backupCount=5)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    result_logger.addHandler(handler)

    # This part of the code opens a CV window which overlays a RS image of a selected grasp onto the live
    # video feed of the RS camera. The purpose of this code is to recreate the grasp as accurately as possible
    # by aligning the selected object with the outline of the object depicted in the selected grasp image.
    # The CV window can be zoomed in (CTRL and (SHIFT) +), zoomed  out (CTRL and -), and the transparency can be increased (>) or decreased (<)
    # When alignment is complete, press SPACE bar to move onto the main code.

    my_objects = match.match(my_dir)
    if(my_objects.success == False):
        my_logger.info("{}:Exiting program".format(scriptname))
        sys.exit(1)
    my_objects.pickup_files()
    my_objects.pair_files()

    obj_id_list = map(int,my_objects.id)

    my_logger.info("{}:Total number of objects {} - List of objects = {}".format(scriptname,len(obj_id_list ),obj_id_list))
    result_logger.info("{}:Total number of objects {} - List of objects = {}".format(scriptname,len(obj_id_list ),obj_id_list))
    print(obj_id_list)

    my_gripper = ng.gripper()
    current_finger_low_pos = my_gripper.palm.get_palm_lower_limits()
    my_logger.info("{}:Current calibrated position of the gripper = {}".format(scriptname,current_finger_low_pos))
    print ("Current calibrated position of the gripper = {}".format(current_finger_low_pos))

    for num in obj_id_list:
        cap = cv2.VideoCapture(0)  # RS camera

        '''Enter image number'''
        # num = 422104
        #num = 187017
        #num = 422114
        #num = 756416
        # The npy files should be in the folder ../trials
        img_filename = '../trials/{}_RS_color.npy'.format(num)
        img = np.load(img_filename)
        transparency = 0.6

        my_logger.info("{}: Image File {} - object {}".format(scriptname,num, img_filename))
        print("Displaying Image File {} of object {}".format(num, img_filename))

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

        move_file = my_dir + '/{}-ur5'.format(num)
        try:
            with open(move_file) as f:
                lines = f.readlines()

        except:
            my_logger.info("{}:unable to open file {}, skipping object id {} ".format(scriptname,move_file, num))
            print("unable to open file {}, skipping object id {} ".format(move_file, num))
            result_logger.info("Object {} grasp replay: Failure:unable to open file {}".format(num,move_file))
            continue

        if ("Fingers at calibration" != lines[0][0:22]):
            my_logger("{}:Skipping object id {}: {}-ur5 line 0 should have Fingers at calibration = [.....]".
                      format(scriptname,num,move_file))
            result_logger.info("Object {} grasp replay: Failure:line 0 should have Fingers at calibration file {}".format(num, move_file))
            print("Skipping object id {}: {}-ur5 line 0 should have Fingers at calibration = [.....]".
                      format(num,move_file))
            continue
        else:
            fing_low_pos_data_col = lines[0][26:52].strip().split(",")
            fing_low_pos_data_col = map(int, fing_low_pos_data_col)
            fing_low_pos_data_col.insert(0, 0)
            my_logger.info("{}:Calibrated position of the gripper at data collection = {}".format(scriptname, fing_low_pos_data_col))
            print ("Calibrated position of the gripper at data collection = {}".format(fing_low_pos_data_col))

        num_waypoints, pose = way_points_ur5(lines[1:])

        # The function 'st_from_UR5_base_to_object_platform' uses the TCP to the object origin - obtained using the UR5
        # and one more rotation to create the static transformation from the base of ur5 to the object origin
        HT_base_to_object = st_from_UR5_base_to_object_platform()

        if num_waypoints != 2:  # The current assumption is that we are providing only 2 waypoints in the move_file
            my_logger.info("{}:Skipping object id {}: {}-ur5 number of waypoints is not equal to 2".format(scriptname,num,move_file))
            result_logger.info("Object {} grasp replay: Failure: number of waypoints is not equal to 2 {}".format(num,move_file))
            print("Skipping object id {}: {}-ur5 number of waypoints is not equal to 2".format(num,move_file))
            continue
        else:
            my_line = 1
            command_str = []
            finger_pos = []

            skip = 0
            for p in pose:
                t, x, y, z, Rx, Ry, Rz, f1, f2, f3, f4 = p
                x, y, z, Rx, Ry, Rz = base_to_gripper([x, y, z, Rx, Ry, Rz], HT_base_to_object)
                val, msg = check_position(x, y, z)
                if val == 0:
                    my_logger.info("{}:Skipping object id {}: Pose line {}: ".format(scriptname,num,my_line) +
                                   "coordinates outside the box: "+ msg)
                    result_logger.info("Object {} grasp replay: Skip:Pose line {} ".format(num,my_line)
                                       + "coordinates outside the box: "+ msg)
                    print("Skipping object id {}: Pose line {}: ".format(num,my_line) +
                                   "coordinates outside the box: "+ msg)
                    skip = 1
                    break
                success, c_str = ur5.compose_command(x, y, z, Rx, Ry, Rz)
                command_str.append(c_str)
                my_grip = [0, 0, 0, 0, 0]
                my_grip[1] = f1 - fing_low_pos_data_col[1]
                my_grip[2] = f2 - fing_low_pos_data_col[2]
                my_grip[3] = f3 - fing_low_pos_data_col[3]
                my_grip[4] = f4 - fing_low_pos_data_col[4]

                val, msg = check_if_finger_movement_excessive(my_grip)
                if val == 0:
                    my_logger.info("{}:Skipping object id {}: Pose {}: ".format(scriptname,num, my_line) +
                                   "finger angles outside limit: "+ msg)
                    result_logger.info("Object {} grasp replay: Skip:Pose {} ".format(num, my_line)
                                       + "finger angles outside limit: " + msg)
                    print("Skipping object id {}: Pose {}: ".format(num, my_line) +
                                   "finger angles outside limit: "+ msg)
                    skip = 1
                    break
                my_grip[1] = my_grip[1] + current_finger_low_pos[1]
                my_grip[2] = my_grip[2] + current_finger_low_pos[2]
                my_grip[3] = my_grip[3] + current_finger_low_pos[3]
                my_grip[4] = 0  # temp to restrict finger spreading
                finger_pos.append(my_grip)
                my_logger.info("{}:Position {}: x={:.3f}, y={:.3f}, z={:.3f}, Rx={:.3f}, Ry={:.3f}, Rz={:.3f}, Fingers {}".
                      format(scriptname,my_line, x, y, z, Rx, Ry, Rz, my_grip))
                print("Position {}: x={:.3f}, y={:.3f}, z={:.3f}, Rx={:.3f}, Ry={:.3f}, Rz={:.3f}, Fingers {}".
                      format(my_line, x, y, z, Rx, Ry, Rz,my_grip))
                my_line += 1
        if skip == 1:
            continue
        starting_pose = ur5.get_UR5_tool_position()
        remote_commander = ur5.UR5_commander(ur5.HOST)

        # intermediate point of the gripper187017
        # t, x, y, z, Rx, Ry, Rz, f1, f2, f3, f4 = 6.666211, 10.463093, -105.186267, 169.729363, -1.043381, -1.463836, -0.950437, 15640, 13834, 17446, 12866
        # grasp step 1
        remote_commander.send(command_str[0])
        time.sleep(5.0)
        my_gripper.palm.move_to_goal_position(finger_pos[0])
        # grasp
        remote_commander.send(command_str[1])
        time.sleep(2.0)
        my_gripper.palm.move_to_goal_position(finger_pos[1])
        time.sleep(2.0)
        # lift
        remote_commander.send(command_str[0])
        time.sleep(2.0)
        # put it back
        remote_commander.send(command_str[1])
        time.sleep(2.0)
        # release fingers
        my_gripper.palm.move_to_goal_position(finger_pos[0])
        time.sleep(2.0)
        # Sending the tcp back to the start location
        x, y, z, Rx, Ry, Rz = starting_pose
        success, command_str = ur5.compose_command(x, y, z, Rx, Ry, Rz)
        if success:
            print("Command String: {}".format(command_str))
            my_logger.info("{}:Sending Command: {}".format(scriptname,command_str))
            remote_commander.send(command_str)
            time.sleep(5.0)
        remote_commander.close()
        my_gripper.move_to_start()

        img_filename = '../trials/{}_RS_color.npy'.format(num)
        img = np.load(img_filename)
        cv2.imshow("RS", img)  # opens CV window with image overlaid on frames
        while True:

            ret, frame = cap.read()
            k = cv2.waitKey(1)
            k = k % 256
            if k == 115:
                result_logger.info("Object {} grasp replay: Success".format(num))
                my_logger.info("Object {} grasp replay: Success".format(num))
                break
            elif k == 102:
                result_logger.info("Object {} grasp replay: Failure".format(num))
                my_logger.info("Object {} grasp replay: Failure".format(num))
                break
            else:
                pass

        cv2.destroyAllWindows()


