import math
import numpy as np

	#From Wikipedia - the X computation uses q0 and q1 where the formula has q1 and q3 ???
	#Z uses q1 and q2 where as the formula uses q2 and q3???

def quaternion_to_euler_angle_wikipedia(q0, q1, q2, q3):
	q1squared = q1*q1
	
	t0 = +2.0 * (q3 * q0 + q1*q2)
	t1 = +1.0 - 2.0 * (q0*q0 + q1squared)
	X = math.degrees(math.atan2(t0, t1))
	
	t2 = +2.0 * (q3*q1 - q2*q0)
	t2 =  1 if t2 > 1 else t2
	t2 = -1 if t2 < -1 else t2
	Y = math.degrees(math.asin(t2))
	
	t3 = +2.0 * (q3 * q2 + q0*q1)
	t4 = +1.0 - 2.0 * (q1squared + q2*q2)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z

	# the following based on what was written in mathworks web-site

def quat_to_euler_mathworks(q0, q1, q2, q3):

	q0_sq = q0*q0
	q1_sq = q1*q1
	q2_sq = q2*q2
	q3_sq = q3*q3

	alpha = math.degrees(math.atan2(2*(q1*q2+q0*q3),(q0_sq + q1_sq - q2_sq - q3_sq)))
	beta = math.degrees(math.asin(2*(q0*q2-q1*q3)))
	gamma = math.degrees(math.atan2(2*(q2*q3+q0*q1),(q0_sq - q1_sq - q2_sq + q3_sq)))

	#alpha is the angle to rotate around z, beta about y and gamma is around x.
    #rotation order is z-y-x body is rotated with respect to z axis ,y-axis and x-axis by by alpha, beta, gamma
	return alpha, beta, gamma


def rotmat_to_RPY(R):

	r00 = R[0,0]
	r01 = R[0,1]
	r02 = R[0,2]
	r10 = R[1,0]
	r11 = R[1,1]
	r12 = R[1,2]
	r20 = R[2,0]
	r21 = R[2,1]
	r22 = R[2,2]

	alpha = math.degrees(math.atan2(r10,r00))
	temp = math.sqrt(r21*r21+r22*r22)
	beta = math.degrees(math.atan2(-r20,temp))
	gamma = math.degrees(math.atan2(r21,r22))
	return alpha,beta,gamma

def rot_mat_from_abg(a,b,g):
	a = math.radians(a)
	b = math.radians(b)
	g = math.radians(g)

	r00 = math.cos(a)*math.cos(b)
	r01 = (math.cos(a)*math.sin(b)*math.sin(g) - math.sin(a)*math.cos(g))
	r02 = (math.cos(a)*math.sin(b)*math.cos(g) + math.sin(a)*math.sin(g))

	r10 = math.sin(a)*math.cos(b)
	r11 = (math.sin(a)*math.sin(b)*math.sin(g) + math.cos(a)*math.cos(g))
	r12 = (math.sin(a)*math.sin(b)*math.cos(g) - math.cos(a)*math.sin(g))

	r20 = -(math.sin(b))
	r21 = math.cos(b)*math.sin(g)
	r22 = math.cos(b)*math.cos(g)

	R = np.zeros((3,3))

	R[0,0] = r00
	R[0,1] = r01
	R[0,2] = r02
	R[1,0] = r10
	R[1,1] = r11
	R[1,2] = r12
	R[2,0] = r20
	R[2,1] = r21
	R[2,2] = r22

	return R


if __name__ == '__main__':

	#q0 = 0.0044
	#q1 = -0.7342
	#q2 = 0.0015
	#q3 = -0.679

    #q0 = 0.4971
    #q1 = 0.5075
    #q2 = -0.5077
    #q3 = 0.4874

	#q0 = 0.2653
	#q1 = 0.6973
	#q2 = 0.248
	#q3 = 0.6179



	# Angles are Rz(alpha), Ry(beta), Rx(gamma) should be supplied.
	R1 = rot_mat_from_abg(90,0,90)
	R1.shape = (3,3)
	alpha,beta,gamma = rotmat_to_RPY(R1)
	for i, row in enumerate(R1):
		print i, ' '.join(str(row))

	print("Rz = {}, Ry = {}, Rx = {}".format(alpha,beta,gamma) )



