import math
import numpy as np

	#From Wikipedia - the X computation uses q0 and q1 where the formula has q1 and q3 ???
	#Z uses q1 and q2 where as the formula uses q2 and q3???

def quaternion_to_euler_angle_wikipedia(q0, q1, q2, q3):	#not using this due to the issue mentioned above
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

def quat_to_euler_mathworks(q0, q1, q2, q3): #This is from mathworks and not using it anymore

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
	'''

	:param a: Rotation about Z
	:param b: Roation about Y
	:param g: Rotation about X
	:return: Rotation Matrix
	'''
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

def quat_to_axis_angle(q0, q1, q2, q3):
	'''

	:param q0: w
	:param q1: qx
	:param q2: qy
	:param q3: qz
	:return:axis vector*angle expressed as Rx, Ry, and Rz components
	'''

	sqr_w = q0*q0
	sqr_qx = q1*q1
	sqr_qy = q2*q2
	sqr_qz = q3*q3

	invs = 1 / (sqr_w + sqr_qx + sqr_qy + sqr_qz)	#may not be required if quarternion is normalized.
	r00 = (sqr_qx - sqr_qy - sqr_qz + sqr_w)*invs
	r11 = (-sqr_qx + sqr_qy - sqr_qz + sqr_w)*invs
	r22 = (-sqr_qx - sqr_qy + sqr_qz + sqr_w)*invs

	theta = math.acos((r00+r11+r22-1)/2)
	sinetheta = math.sin(theta)
	v = (2*sinetheta)*theta

	qxy = q1*q2
	qwz = q0*q3
	r10 = 2.0 * (qxy + qwz)*invs
	r01 = 2.0 * (qxy - qwz)*invs



	cz = ((r10-r01)/(2*sinetheta))*theta

	qxz = q1*q3
	qwy = q2*q0
	r20 = 2.0 * (qxz - qwy)*invs
	r02 = 2.0 * (qxz + qwy)*invs

	by = ((r02-r20)/(2*sinetheta))*theta

	qyz = q2*q3
	qwx = q0*q1
	r21 = 2.0 * (qyz + qwx)*invs
	r12 = 2.0 * (qyz - qwx)*invs

	ax = ((r21-r12)/(2*sinetheta))*theta

	return ax,by,cz

def rotmat_to_quaternion(R):
	# This function can cause divide by zero errors!!!!!


	for (x,y), value in np.ndenumerate(R):
		if value > 0.0 and value <0.000001:
			R[x,y] = 0
		elif value < 0.0 and value > -0.000001:
			R[x,y] = 0
	r00 = R[0,0]
	r01 = R[0,1]
	r02 = R[0,2]
	r10 = R[1,0]
	r11 = R[1,1]
	r12 = R[1,2]
	r20 = R[2,0]
	r21 = R[2,1]
	r22 = R[2,2]



	qw= (math.sqrt(1 + r00 + r11 + r22))/2
	c = 4*qw
	qx = (r21 - r12)/c
	qy = (r02 - r20)/c
	qz = (r10 - r01)/c

	return qw,qx,qy,qz

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

	rpy = [(0,0,90),(90,0,90),(0,-90,90),(0,-90,-180),(-180,0,-90)]

	for gba in rpy:
		a = gba[2]
		b = gba[1]
		g = gba[0]
		print("Given Rx = {}, Ry = {}, Rz = {}".format(g,b,a) )
		R = rot_mat_from_abg(a,b,g)
		R.shape = (3,3)
		print("Computed rotation matrix")
		for i, row in enumerate(R):
			print ' '.join(str(row))
		alpha,beta,gamma = rotmat_to_RPY(R)
		print("Recomputed Rx = {}, Ry = {}, Rz = {}\n".format(gamma,beta,alpha) )

		#q0,q1,q2,q3 = rotmat_to_quaternion(R)
		#Rx,Ry,Rz = quat_to_axis_angle(q0,q1,q2,q3)
		#print("Axis Angles Rx = {}, Ry = {}, Rz = {}\n\n".format(Rx,Ry,Rz) )

	labview_fname = "1140-2017-05-23-at-20-04-18 .txt"

	with open(labview_fname) as f:
		lines = f.readlines()
		r00,r01,r02,r03,r10,r11,r12,r13,r20,r21,r22,r23 = ([] for i in range(12))

	for line in lines[6:]:
		vq_str = line[51:]
		vq_str = vq_str[:vq_str.rfind("*")-4]
		x,y,z,qr,qi,qj,qk = map(float,vq_str.split(","))
		Rx,Ry,Rz = quat_to_axis_angle(qr,qi,qj,qk)
		print("Axis Angles Rx = {}, Ry = {}, Rz = {}\n\n".format(Rx,Ry,Rz))





