#!/usr/bin/env python3
from class_ekf import EKF
import rospy
# from rosgraph_msgs import Clock
from math import sin, cos, sqrt, atan2, acos
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from rospy.numpy_msg import numpy_msg
import numpy as np


def rotationMatrixToQuaternion1(m):
    #q0 = qw
    tt = np.matrix.trace(m)
    q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if(tt > 0):
        tt = np.sqrt(tt + 1)
        q[3] = 0.5 * tt
        tt = 0.5/tt
        q[0] = (m[2,1] - m[1,2]) * tt
        q[1] = (m[0,2] - m[2,0]) * tt
        q[2] = (m[1,0] - m[0,1]) * tt

    else:
        i = 0
        if (m[1,1] > m[0,0]):
            i = 1
        if (m[2,2] > m[i,i]):
            i = 2
        j = (i+1)%3
        k = (j+1)%3

        tt = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
        q[i] = 0.5 * tt
        tt = 0.5 / tt
        q[3] = (m[k,j] - m[j,k]) * tt
        q[j] = (m[j,i] + m[i,j]) * tt
        q[k] = (m[k,i] + m[i,k]) * tt

    return q

def imucallback1(msg1):
    global Px, Py, t, imu1counter, prev_time_1, imu1_x, imu1_y, imu1_theta, v1x, v1y
    if imu1counter ==0:
        prev_time_1 = rospy.get_time()
        imu1counter +=1
    else:
        
        curr_time_1 = rospy.get_time()
        dt1 = curr_time_1 - prev_time_1
        imu1_x = ekf.x[0][0,0]
        imu1_y = ekf.x[1][0,0]
        imu1_theta = ekf.x[2][0,0]

        prevx = imu1_x 
        prevy = imu1_y
        prevt = imu1_theta
        ax = msg1.linear_acceleration.x
        ay = msg1.linear_acceleration.y
        ww = msg1.angular_velocity.z

        
        
        
        # imu1_x += v1x*dt1 + 0.5*ax*dt1*dt1
        # imu1_y += v1y*dt1 + 0.5*ay*dt1*dt1
        # imu1_theta += ww*dt1

        # v1x = (imu1_x - prevx)/dt1
        # v1y = (imu1_y-prevy)/dt1
        ekf.recompute_HR(dt1,ax,ay,v1x,v1y,imu1_x,imu1_y,prevx,prevy,prevt)
        #z = np.matrix([imu1_x, imu1_y, imu1_theta]).T 
        z = np.matrix([ax, ay, ww]).T           
        y = z - ekf.H_imu*ekf.x
        # while(y[1]>np.pi):
        #     y[1] -= 2.0*np.pi
        # while(y[1]<np.pi):
        #     y[1] += 2.0*np.pi    
        
        Si = ekf.H_imu *ekf.p*ekf.H_imu.T + ekf.R_imu1
        K = (ekf.p * ekf.H_imu.T)*Si.I
        ekf.x += (K*y)
        ekf.p = (ekf.I - K*ekf.H_imu)*ekf.p
        Px = ekf.x[0][0,0]
        Py = ekf.x[1][0,0]
        t = ekf.x[2][0,0]
        prev_time_1 = curr_time_1

    
def imucallback2(msg2):
    global Px, Py, t, imu2counter, prev_time_2, imu2_x, imu2_y, imu2_theta, v2x, v2y
    if imu2counter ==0:
        prev_time_2 = rospy.get_time()
        imu2counter +=1
    else:
        curr_time_2 = rospy.get_time()
        dt2 = curr_time_2 - prev_time_2
        
        imu2_x = ekf.x[0][0,0]
        imu2_y = ekf.x[1][0,0]
        imu2_theta = ekf.x[2][0,0]

        prevx2 = imu2_x 
        prevy2 = imu2_y
        prevt2 = imu2_theta
        ax2 = msg2.linear_acceleration.x
        ay2 = msg2.linear_acceleration.y
        ww2 = msg2.angular_velocity.z
        
        

        imu2_x += v2x*dt2 + 0.5*ax2*dt2*dt2
        imu2_y += v2y*dt2 + 0.5*ay2*dt2*dt2
        imu2_theta += ww2*dt2

        v2x = (imu2_x - prevx2)/dt2
        v2y = (imu2_y-prevy2)/dt2
        ekf.recompute_HR(dt2,ax2,ay2,v2x,v2y,imu2_x,imu2_y,prevx2,prevy2,prevt2)
        #z2 = np.matrix([imu2_x, imu2_y, imu2_theta]).T 
        z2 = np.matrix([ax2, ay2, ww2]).T           
        y2 = z2 - ekf.H_imu*ekf.x
        
        # while(y2[1]>np.pi):
        #     y2[1] -= 2.0*np.pi
        # while(y2[1]<np.pi):
        #     y2[1] += 2.0*np.pi 
        Si2 = ekf.H_imu *ekf.p*ekf.H_imu.T + ekf.R_imu2
        K2 = (ekf.p * ekf.H_imu.T)*Si2.I
        ekf.x += (K2*y2)
        ekf.p = (ekf.I - K2*ekf.H_imu)*ekf.p
        Px = ekf.x[0][0,0]
        Py = ekf.x[1][0,0]
        t = ekf.x[2][0,0]
        prev_time_2 = curr_time_2
        
def velcallback(vel):
      
    global Vx, Vy, w, velcounter, prev_time_v
    if velcounter == 0:
        prev_time_v = rospy.get_time()
        velcounter +=1
    else:    
        curr_time_v = rospy.get_time()
        dtv = curr_time_v - prev_time_v
        velx = ekf.x[0][0,0]
        vely = ekf.x[1][0,0]
        veltheta = ekf.x[2][0,0]
        prevvx = velx 
        prevvy = vely
        prev_v_theta = veltheta

        
        Vx = vel.linear.x
        Vy = vel.linear.y
        w = vel.angular.z
        # rospy.Subscriber('tf', TFMessage, tfcallback)
        ekf.recompute_F_and_Q(dtv,Vx,Vy,w,prev_v_theta)
        ekf.predict()
        Px = ekf.x[0][0,0]
        Py = ekf.x[1][0,0]
        t = ekf.x[2][0,0]
        
        tf1 = p
        tf1.transforms[0].transform.translation.x = Px
        tf1.transforms[0].transform.translation.y = Py
        theta = t
        q_array = rotationMatrixToQuaternion1(np.matrix([[cos(theta),-sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]]))
        tf1.transforms[0].transform.rotation.x = q_array[0]
        tf1.transforms[0].transform.rotation.y = q_array[1]
        tf1.transforms[0].transform.rotation.z = q_array[2]
        tf1.transforms[0].transform.rotation.w = q_array[3]
        pub.publish(tf1) 
        
        prev_time_v = curr_time_v

def tfcallback(tf):
    
    global p, prev_x , prev_y , prev_theta , prev_time_v
    
    if tf.transforms[0].header.frame_id == 'odom' and tf.transforms[0].child_frame_id == 'base_footprint':
        # prev_time_v = rospy.get_time()
        p = tf        
        prev_x = tf.transforms[0].transform.translation.x
        prev_y = tf.transforms[0].transform.translation.y 
        angle_value = tf.transforms[0].transform.rotation.w
        prev_theta = 2*acos(angle_value)


def state_vector_to_scalars(state_vector):
    return (state_vector[0][0,0],state_vector[1][0,0],state_vector[2][0,0])

def cartesian_to_polar(state_vector):
    px,py,vx,vy = state_vector_to_scalars(state_vector)
    ro= sqrt(px**2 + py**2)
    phi     = atan2(py,px)
    ro_dot  = (px*vx + py*vy)/ro
    return np.matrix([ro, phi, ro_dot]).T     
    
class EKF:
    def __init__(self):
        self.x = None         
        self.I = np.matrix([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        self.p = np.matrix ([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        self.H_imu = None
        # self.H_imu = np.matrix([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        
        self.R_imu1 = np.matrix([[0.017,0,0],[0,0.017,0],[0,0,0.0002]]) #
        self.R_imu2 = np.matrix([[0.00024,0,0],[0,0.00024,0],[0,0,0.00001]]) #

    def init_state_vector(self, x,y, theta):
        self.x = np.matrix([[x,y,theta]]).T
    def current_estimate(self):
        return (self.x, self.p)
    def recompute_F_and_Q(self,dt,xx,yy,www, prr):
        V = (xx**2 + yy**2)**0.5
        if www != 0:
            f13 = (V/www)*(-cos(prr)+cos(prr + w*dt))
            f23 = (V/www)*(-sin(prr)+sin(prr + w*dt))
        else:
            f13 = (V)*(cos(prr)*dt)
            f23 = (V)*(sin(prr)*dt)
        self.F = np.matrix([[1.0, 0.0, f13],[0.0,1.0,f23],[0.0,0.0,1.0]])
        sigma2_a = 2.43
        sigma2_alpha = 1.57
        dt2 = dt**2
        dt3 = dt**3
        dt4 = dt**4

        e11 = dt4 * np.cos(prr)**2 * sigma2_a / 4
        e12 = dt4 * np.cos(prr) * np.sin(prr) * sigma2_a / 4
        e21 = dt4 * np.cos(prr) * np.sin(prr) * sigma2_a / 4
        e22 = dt4 * np.sin(prr)**2 * sigma2_a/ 4
        e33 = dt4 * sigma2_alpha/ 4
        self.Q = np.matrix([[e11,e12,0.0],[e21,e22,0.0],[0.0,0.0,e33]])        

    def recompute_HR(self,dt,aax,aay,vvx,vvy,xx,yy,a,b,tt):

        px,py,t = state_vector_to_scalars(self.x)
        # pxpy_squared = px**2+py**2
        # pxpy_squared_sqrt = sqrt(pxpy_squared)
        # pxpy_cubed = (pxpy_squared*pxpy_squared_sqrt)
        e11 = Vx*aax*dt/(xx-a)**2
        e22 = Vy*aay*dt/(yy-b)**2
        e33 = -(tt)/dt
        # if pxpy_squared < 1e-4:
        #     self.H_radar = np.matlib.zeros((3,4))
        #     return
        # e11 = px/pxpy_squared_sqrt
        # e12 = py/pxpy_squared_sqrt
        # e21 = -py/pxpy_squared
        # e22 = px/pxpy_squared
        # e31 = py*(vx*py - vy*px)/pxpy_cubed
        # e32 = px*(px*vy - py*vx)/pxpy_cubed
        self.H_imu = np.matrix([[e11, 0.0, 0.0],
                            [ 0.0,e22, 0.0],
                            [0.0,0.0, 1]])
    
    def predict(self):
        self.x = self.F * self.x
        self.p = (self.F * self.p * self.F.T) + self.Q       



if __name__ == '__main__':

    rospy.init_node('extended_kalman_filter', anonymous=True)
    global tf

    # Defining the initial states of the robot
    Px = 0
    Py = 0
    t = 0

    velcounter = 0
    imu1counter = 0
    imu2counter = 0
    # Defining the initial variables of imu1 reading
    imu1_x = 0
    imu1_y = 0
    imu1_theta = 0

    v1x=0
    v1y = 0
    v2x = 0
    v2y = 0

    # Defining the initial variables of imu2 reading
    imu2_x = 0
    imu2_y = 0
    imu2_theta = 0
    rospy.Subscriber('/imu1', Imu, imucallback1)
    rospy.Subscriber('/imu2', Imu, imucallback2)
    rospy.Subscriber('/cmd_vel',Twist, velcallback)
    pub = rospy.Publisher('/tf',TFMessage, queue_size = 10)
    rospy.Subscriber('tf', TFMessage, tfcallback)
    rate = rospy.Rate(10) 
    rospy.sleep(0.1)
    ekf = EKF()
    ekf.x = np.matrix([p.transforms[0].transform.translation.x,p.transforms[0].transform.translation.y,2*acos(p.transforms[0].transform.rotation.w)]).T
    initial_time = rospy.get_time()
    while not rospy.is_shutdown():
        tf1 = p
        tf1.transforms[0].transform.translation.x = Px
        tf1.transforms[0].transform.translation.y = Py
        theta = t
        q_array = rotationMatrixToQuaternion1(np.matrix([[cos(theta),-sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]]))
        tf1.transforms[0].transform.rotation.x = q_array[0]
        tf1.transforms[0].transform.rotation.y = q_array[1]
        tf1.transforms[0].transform.rotation.z = q_array[2]
        tf1.transforms[0].transform.rotation.w = q_array[3]
        pub.publish(tf1)    
        rospy.loginfo(tf1)
        
        rate.sleep()






