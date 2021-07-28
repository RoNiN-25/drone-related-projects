#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from fly_bot_cpp.msg import kwad_input
from fly_bot_cpp.msg import kwad_state
import numpy as np

# /Kwad/twelve_state
# /Kwad/control_cmd

class DroneFly():
    ''' class for flight control of the quad'''
    def __init__(self):
        ''' Initalizer '''
        # Variable to publish the velocity
        self.cmd = kwad_input()

        self.gravity = 9.81
        self.mass = 3

        # Position - x, y and z axes
        self.currentPosition = np.array([0,0,0])
        self.currentVelocity = np.array([0, 0, 0])

        self.currentAttitude = np.array([0, 0, 0])
        self.currentAngularVelocity = np.array([0, 0, 0])

        self.desiredPosition = np.array([0,0,0])
        self.desiredVelocity = np.array([0, 0, 0])

        # Subscibers and publishers
        self.currentStateSubscriber = rospy.Subscriber("/Kwad/twelve_state", kwad_state, self.currentStateCB)
        self.desiredPositionSubscriber = rospy.Subscriber("/drone/desired_position", Point, self.desiredPositionCB)
        self.desiredVelocitySubscriber = rospy.Subscriber("/drone/desired_velocity", Point, self.desiredVelocityCB)
        self.inputPublisher = rospy.Publisher("/Kwad/control_cmd", kwad_input, queue_size = 10)


        # k1 and k2 constants for [Ft, Tx, Ty, Tz]
        self.k2 = np.array([4, 4, 4])
        self.k1 = np.array([6.25, 6.25, 6.25])

        self.k2_att = np.array([20, 20, 20])
        self.k1_att = np.array([100, 100, 100])

        self.phi_des = 0
        self.theta_des = 0
        self.psi_des = 0

        # PD calculated [Ft,Tx,Ty,Tz]
        self.input = np.array([0, 0, 0, 0])

        #current state of the drone
        self.currentState = kwad_state()

        self.i = 0

        self.Ixx = 6.5e-5
        self.Iyy =  6.21e-5
        self.Izz = 1.18e-4

        rospy.sleep(0.1)
    
    def isThere(self):
        ''' Function to check if the drone has reached set point '''
        if (np.linalg.norm(self.desiredPosition - self.currentPosition) < 0.01):
            return True
        else:
            return False
    
    def positionHold(self):
        ''' function for position holding '''
       
        while not rospy.is_shutdown():
            try:
                self.outerLoop()

                self.innerLoop()

                self.cmd.thrust = self.clamp(self.input[0], 0, 2*self.mass)
                self.cmd.tau_x = self.input[1]
                self.cmd.tau_y = self.input[2]
                self.cmd.tau_z = self.input[3]

                # print('Applied velocity =',self.cmd.linear)
                print('Reached =', self.isThere())
                print('Thrust =',self.cmd.thrust)

                #publish the velocities
                self.inputPublisher.publish(self.cmd)
                rospy.sleep(0.005)
            except rospy.ROSInterruptException:
                rospy.logwarn('Stopping controller....')
        
        
    
    def outerLoop(self):
        ''' function to perform PID calculations '''
        e = self.desiredPosition - self.currentPositon
        e_dot = self.desiredVelocity - self.currentVelocity

        print(e)

        T = self.k1*e + self.k2*e_dot
        g = self.gravity


        self.phi_des = -1/g * T[1]
        self.theta_des = 1/g * T[0]
        self.psi_des = 0
        

        Ft = (g + T[2]) * self.mass
        self.input[0] = Ft

    def innerLoop(self):
        ''' function for innerLoop '''
        desAtt = np.array([self.phi_des, self.theta_des, self.psi_des])
        e = desAtt - self.currentAttitude
        e_dot = - self.currentAngularVelocity
        T = np.array([self.Ixx, self.Iyy, self.Izz]) * (self.k1_att * e + self.k2_att * e_dot)
        self.input[1] = T[0]
        self.input[2] = T[1]
        self.input[3] = T[2]

        

    
    
    def clamp(self, x, lb, ub):
        ''' function to clamp x to range [lb,ub] '''
        return min(max(x, lb), ub)


    def currentStateCB(self, data):
        ''' Current position callback function '''
        self.currentState = data

        self.currentPositon = np.array([data.x, data.y, data.z])
        self.currentVelocity = np.array([data.x_dot, data.y_dot, data.z_dot])

        self.currentAttitude = np.array([data.phi, data.theta, data.psi])
        self.currentAngularVelocity = np.array([data.p, data.q, data.r])
    
    def desiredPositionCB(self, data):
        ''' desired position callback function '''
        self.desiredPosition = np.array([data.x, data.y, data.z])
    
    def desiredVelocityCB(self, data):
        ''' desired velocity callback function '''
        self.desiredVelocity = np.array([data.x, data.y, data.z])

