import math

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

if __name__ == '__main__':

	#q0 = 0.0044
	#q1 = -0.7342
	#q2 = 0.0015
	#q3 = -0.679
    q0 = 0.4971
    q1 = 0.5075
    q2 = -0.5077
    q3 = 0.4874

    Rz,Ry,Rx = quat_to_euler_mathworks(q0,q1,q2,q3)

    print("About X {}, About Y {}, About Z {}".format(g,b,a))

