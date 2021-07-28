#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from nav_msgs.msg import Odometry
import numpy as np


class DroneFly():
    ''' class for flight control of the quad'''
    def __init__(self):
        ''' Initalizer '''
        # Variable to publish the velocity
        self.cmd = Twist()

        # Position - x, y and z axes
        self.currentPosition = np.array([0,0,0])
        self.desiredPosition = np.array([0,0,0])

        # Subscibers and publishers
        self.currentPositionSubscriber = rospy.Subscriber("/mavros/global_position/local", Odometry, self.currentPositionCB)
        self.desiredPositionSubscriber = rospy.Subscriber("/drone/desired_position", Point, self.desiredPositionCB)
        self.stateSubscriber = rospy.Subscriber("/mavros/state", State, self.stateCB)
        self.velocityPublisher = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size = 10)


        # kp, ki and kd of PID controller - x, y and z axes
        self.kp = np.array([0.25, 0.25, 0.5])
        self.ki = np.array([0, 0, 0])
        self.kd = np.array([15, 15, 15])
        

        self.prevError = np.array([0,0,0])
        self.totalError = np.array([0,0,0])

        # PID calculated x, y and z axes' velocity
        self.input = np.array([0,0,0])

        #current state of the drone
        self.currentState = State()

        # Wait for the services
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')

        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.setModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.sleep(0.1)
    
    def isThere(self):
        ''' Function to check if the drone has reached set point '''
        if (np.linalg.norm(self.desiredPosition - self.currentPosition) < 0.25).all():
            self.totalError = np.array([0,0,0])
            return True
        else:
            return False
    
    def positionHold(self):
        ''' function for position holding '''
       
        while not rospy.is_shutdown():
            try:
                self.calcPID()

                self.cmd.linear.x = self.clamp(self.input[0], -0.5, 0.5)
                self.cmd.linear.y = self.clamp(self.input[1], -0.5, 0.5)
                self.cmd.linear.z = self.clamp(self.input[2], -1, 1)

                if self.currentPosition[2] < 0.2:
                    self.cmd.linear.x = 0
                    self.cmd.linear.y = 0

                # print('Applied velocity =',self.cmd.linear)
                print('vel =',self.input)
                print('Reached =', self.isThere())

                #publish the velocities
                self.velocityPublisher.publish(self.cmd)
                rospy.sleep(0.05)
            except rospy.ROSInterruptException:
                rospy.logwarn('Stopping controller....')
        
        
    
    def calcPID(self):
        ''' function to perform PID calculations '''
        error = self.desiredPosition - self.currentPosition
        self.totalError = error + self.totalError
        self.input = self.kp*error + self.kd*(self.prevError - error) + self.ki*self.totalError
        self.prevError = error
    
    def arm(self):
        ''' function to arm the drone '''
        for i in range(100):
            self.velocityPublisher.publish(self.cmd)
            rospy.sleep(0.01)
        if self.currentState.mode != "OFFBOARD":
            if self.setModeService(0,'OFFBOARD'):
                rospy.loginfo("Set mode to OFFBOARD!")
                rospy.sleep(0.1)
            else:
                rospy.logwarn("Unable to set mode")
                return False
        
        if not self.currentState.armed:
            if self.armService(True):
                rospy.loginfo("Armed successfully!")
                rospy.sleep(0.1)
                return True
            else:
                rospy.logwarn("Unable to arm")
                return False
    
    def disarm(self):
        ''' function to disarm drone '''
        if self.currentState.armed:
            if self.armService(False):
                rospy.loginfo("Disarmed drone")
                return True
            else:
                return False
        return False
    
    def clamp(self, x, lb, ub):
        ''' function to clamp x to range [lb,ub] '''
        return min(max(x, lb), ub)


    def currentPositionCB(self, data):
        ''' Current position callback function '''
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        self.currentPosition = np.array([x, y, z])
    
    def desiredPositionCB(self, data):
        ''' desired position callback function '''
        self.desiredPosition = np.array([data.x, data.y, data.z])
    
    def stateCB(self, data):
        ''' state callback '''
        self.currentState = data

