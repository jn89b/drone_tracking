#!/usr/bin/env python

import rospy 
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import scipy
from scipy.integrate import odeint
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64

from mavros_msgs.msg import AttitudeTarget
from drone_tracking.msg import LQRGain

""" 
LQR control for position tracking and referencing 
of fiducial tags

Drone in sim is AR 2.0 
weight is 13.4 oz 
Ix = 8.1 * 1e-3 kg/m^2
Iy = 8.1 * 1e-3 kg/m^2
Iz = 14.2 * 1e-3 kg/m^2

LQR procedures
    #get act state done by callbacks already
    #get desired state done by callbacks already
    #compute state error 
    #do lqr and get my gains
    #update state space model done by callbacks already
"""

Ix = 8.1 * 1E-3
Iy = 8.1 * 1E-3
g = 9.81 #m^2/s
m = 1.0 #kg

class LQR():
    def __init__(self, A = None, B = None, Q = None, R = None, x0 = None,
                 rate_val = None):     
        if(A is None or B is None):
            raise ValueError("Set proper system dynamics.")

        # 
        self.n = A.shape[0]
        self.m = B.shape[1]
        
        self.A = A
        self.B = 0 if B is None else B
        
        #self.high_Q = 2.5    
        self.Q = np.diag(np.full(4,0.1E-1)) #np.diag(np.array(Q)) #if Q is None else Q
        #1.9 is high gain, set to 0.9 after close to target
        self.Q[0,0] = 1.8#2.0#2.5#1.85Q = 1.4 or 0.93 for apriltag , Q=1.9 Q = 3.25 for position 
        self.low_Q = 1.0#0.5
        
        self.R = np.diag([20]) #25, 14, 75 30 for apriltag, 9.1 for regular position
        self.low_R = np.diag([25]) #25 50
        self.close = 0.075
        self.x = np.zeros((self.n, 1)) if x0 is None else x0
        
        #gains
        self.K = [] 
        self.low_K = []
        
        #desired states
        self.z = [0] * len(Q)
        self.error = [0] * len(Q)
        self.lqr_output = [0] * len(Q)

        #rate values
        self.rate_val = 30 if rate_val is None else rate_val
        self.dt = 1/self.rate_val
        
        #feedforward inputs
        self.old_u = np.array([0,0,0,0])
        
    def current_state(self, x_state):
        """update position estimate """
        self.x[0] = x_state[0]
        self.x[1] = x_state[1]
        self.x[2] = x_state[2]
        self.x[3] = x_state[3]
                
    def desired_state(self, desired_val):
        """get desired position from current position"""
        #desired_x = msg.pose.position.x # - self.x[0]
        #des_y = msg.pose.position.y
        #des_vel_x = (desired_x - self.z[0])/self.dt 
        self.z[0] = desired_val[0] 
        self.z[1] = desired_val[1]
        self.z[2] = desired_val[2] #self.x[2]#desired_val[2]
        self.z[3] = desired_val[3]

    def compute_error(self):
        """compute error of state
        for fiducial tags desired already accounts for the error 
        since its relative"""
        #self.error[0] = self.z[0] # - self.x[0] if not using apriltag use this
        self.error[0] = self.z[0]
        self.error[1] = self.z[0]/ self.dt
        self.error[2] = self.z[2] - self.x[2]
        self.error[3] = self.z[3] - self.x[3]
        
    def lqr(self, A, B, Q, R):
        """Solve the continuous time lqr controller.
        dx/dt = A x + B u
        cost = integral x.T*Q*x + u.T*R*u
        """
        # http://www.mwm.im/lqr-controllers-with-python/
        # ref Bertsekas, p.151

        # first, try to solve the ricatti equation, X is n x n
        X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))

        # compute the LQR gain Compute $K=R^-1B^TS$
        K = np.matrix(scipy.linalg.inv(R) * (B.T * X))
        print("K is", K )
        print("/n")
        
        #gives solution that yields stable system, negative values
        eigVals, eigVecs = scipy.linalg.eig(A - B * K)
        print("Eigens", eigVals )
        return np.asarray(K), np.asarray(X), np.asarray(eigVals)

    def compute_K(self):
        """get gains from LQR"""
        #print(self.Q)
        K, _, _ = self.lqr(self.A, self.B, self.Q, self.R)
        self.K = K
        
    def compute_lowK(self):
        """switch to low Q"""
        self.Q[0,0] = self.low_Q
        K, _, _ = self.lqr(self.A, self.B, self.Q, self.low_R)
        self.low_K = K
        
    def update_state(self): 
        """update state space"""
        self.lqr_output = np.dot(self.B.T, self.u)
        self.x = np.dot(self.A, self.x)  + self.lqr_output
        
    def get_u(self,K_val):
        """compute controller input"""
        self.u = np.multiply(K_val, self.error)[0]
        print("u is", self.u) 
        #self.u = self.u #+ self.old_u
        
        #threshold command for pitch rate
        max_pitch_rate = 0.45 #0.25, is the moveabout 20 degrees
        att_rate_idx = 2#2
        if abs(self.u[att_rate_idx])>= max_pitch_rate:
            if self.u[att_rate_idx] > 0:              
                self.u[att_rate_idx] = max_pitch_rate
            else:
                self.u[att_rate_idx] = -max_pitch_rate
        
        #threshold command for body velocity              
        max_vel = 15.0
        vel_idx = 0#0
        if abs(self.u[vel_idx])>= max_vel:
            if self.u[vel_idx] > 0:              
                self.u[vel_idx] = max_vel
            else:
                self.u[vel_idx] = -max_vel

        #update u after done
        self.old_u = self.u

    def main(self):         
        """update values to LQR"""
        self.compute_error()
        #gain scheduling, if close to target reduce gains
        if self.error[0] <= self.close:
            self.get_u(self.low_K)
        else:         
            self.get_u(self.K)
            
        self.update_state()
        
class DroneLQR():
    def __init__(self, Ax, Bx, Ay, By, Q, R, rate_val):
        """drone LQR controller"""
        #quad position callback
        self.quad_sub = rospy.Subscriber("mavros/odometry/in",
                                         Odometry,
                                         self.current_state)
        
        # """should refactor this to say if I want to websling or not"""
        # self.track_sub = rospy.Subscriber("uav0/mavros/vision_pose/pose", 
        #                                          PoseStamped,
        #                                          self.desired_state)
        
        self.track_sub = rospy.Subscriber("mavros", 
                                         PoseStamped,
                                         self.desired_state)

        self.k_pub = rospy.Publisher("K_gain", LQRGain, queue_size=5)
        
        self.x_state = np.array([0.0,0.0,0.0,0.0])
        self.y_state = np.array([0.0,0.0,0.0,0.0])      
        
        #subsystem lqr X
        self.x_lqr = LQR(A = Ax, B = Bx, Q = Q, R = R, x0 = self.x_state,
                    rate_val = rate_val) #import matrices into class
        #subsystem lqr Y
        self.y_lqr = LQR(A = Ay, B = By, Q = Q, R = R, x0 = None,
                    rate_val = rate_val) #import matrices into class

        #intialize desired states
        self.desired_x = [0.0, 0.0, 0.0, 0.0]
        self.desired_y = [0.0, 0.0, 0.0, 0.0]
        self.dt = 1/rate_val
        
    def current_state(self, msg):
        """update current estimates"""
        px = msg.pose.pose.position.x 
        py = msg.pose.pose.position.y
        
        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                             orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        pitch_rate = pitch - self.x_state[2]/self.dt
        roll_rate = roll - self.y_state[2]/self.dt
         
        self.x_state[0] = px
        self.x_state[1] = vel_x
        self.x_state[2] = pitch
        self.x_state[3] = pitch_rate

        self.y_state[0] = py
        self.y_state[1] = vel_y
        self.y_state[2] = roll
        self.y_state[3] = roll_rate

    def desired_state(self, msg):
        """get desired position from current position"""
        desired_x = msg.pose.position.x # - self.x[0]
        desired_y = msg.pose.position.y
        # desired_x = 5.0 - self.x_state[0]
        # desired_y =  5.0 - self.y_state[0] + 10.0
        # print("desired x and desired y", desired_x, desired_y)
        self.desired_x[0] = desired_x 
        self.desired_y[0] = desired_y 
        
        self.desired_x[2] = desired_x - self.x_state[2]
        self.desired_y[2] = desired_y - self.y_state[2]

    def publish_input(self):
        """publish body rate commands"""
        # gains = LQRGain()        
        tol = 0.075    #0.075 for regular tracking , 0.5 for apriltag
        zero_vals = [0,0,0,0]
        input_x = [float(xu) for xu in self.x_lqr.u]
        input_y = [-float(yu) for yu in self.y_lqr.u]
        
        if abs(self.x_lqr.error[0]) <= tol and abs(self.y_lqr.error[0]) >= tol:
            pub_vals = zero_vals
            pub_vals.extend(input_y)
            self.k_pub.publish(pub_vals)
            #self.k_pub.publish([0.0, -self.y_lqr.u])
            
        elif abs(self.x_lqr.error[0]) >= tol and abs(self.y_lqr.error[0]) <= tol:
            pub_vals = input_x
            pub_vals.extend(zero_vals)
            self.k_pub.publish(pub_vals)
            #self.k_pub.publish([self.x_lqr.u, 0.0])
        
        elif abs(self.x_lqr.error[0]) <= tol and abs(self.y_lqr.error[0]) <= tol:
            pub_vals = zero_vals
            pub_vals.extend(zero_vals)
            self.k_pub.publish(pub_vals)
            #self.k_pub.publish([0.0, 0.0])
        else:
            pub_vals = input_x
            pub_vals.extend(input_y)   
            self.k_pub.publish(pub_vals)

    def compute_gains(self):
        """get gains from ricatti equation"""
        self.x_lqr.compute_K()
        self.y_lqr.compute_K()
        
        self.x_lqr.compute_lowK()
        self.y_lqr.compute_lowK()
        
    def main(self):
        """main implementation"""
        self.x_lqr.current_state(self.x_state)
        self.x_lqr.desired_state(self.desired_x)
        
        self.y_lqr.current_state(self.y_state)
        self.y_lqr.desired_state(self.desired_y)
        
        self.x_lqr.main()
        self.y_lqr.main()
        
        self.publish_input()
        
        
if __name__ == "__main__":
    
    rospy.init_node("lqr_controller", anonymous=False)
    rate_val = 30

    ############ Set up X and Y #####################
    # X-subsystem
    # The state variables are x, dot_x, pitch, dot_pitch
    Ax = np.array(
        [[0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, g, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0]])
    
    Bx = np.array(
        [[0.0],
        [0.0],
        [0.0],
        [1 / Ix]])
    
    # Y-subsystem
    # The state variables are y, dot_y, roll, dot_roll
    Ay = np.array(
        [[0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, -g, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0]])
    
    By = np.array(
        [[0.0],
        [0.0],
        [0.0],
        [1 / Iy]])
    
    ## Q penalty
    Q_fact =  1.5 #penalizes performance rating 
    Q = np.array([[Q_fact, 0, 0], 
                [0, Q_fact, 0, 0], 
                [0, 0, Q_fact/2, 0], 
                [0, 0 , 0, Q_fact/2]])
    
    ## R penalty for input
    R = np.array([[Q_fact, 0, 0], 
                [0, Q_fact, 0, 0], 
                [0, 0, Q_fact/2, 0], 
                [0, 0 , 0, Q_fact/2]])
    
    drone_lqr = DroneLQR(Ax, Bx, Ay, By, Q, R, rate_val)
    drone_lqr.compute_gains()
    rate = rospy.Rate(rate_val)
    while not rospy.is_shutdown():
        drone_lqr.main()
        rate.sleep()
    
    
    
    