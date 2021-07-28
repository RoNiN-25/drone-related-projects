#!/usr/bin/env python
import rospy
import numpy as np
from DroneFly import DroneFly
from geometry_msgs.msg import Point
import threading

desiredPositionPublisher = rospy.Publisher('/drone/desired_position', Point, queue_size = 10)
desiredVelocityPublisher = rospy.Publisher('/drone/desired_velocity', Point, queue_size = 10)

def populateTrajectory():
    ''' Function to create the trajectory points '''
    traj = [(2,2,2)]
    val = np.linspace(2,-2,11)
    for i in val:
        traj.append((2,i,2))
    for i in val:
        traj.append((i,-2,2))
    val = np.linspace(-2,2,11)
    for i in val:
        traj.append((-2,i,2))
    for i in val:
        traj.append((i,2,2))
    # return [(2,2,2), (2,-2,2), (-2,-2,2), (-2,2,2)]
    return traj

def trajFollow():
    ''' function to run the trajectory following '''
    rospy.loginfo('Initializing Drone.....')
    drone = DroneFly()
    rospy.sleep(1.0)

    posHoldThread = threading.Thread(target=drone.positionHold)
    posHoldThread.start()
    
    desPos = Point()
    desVel = Point()

    trajectory = populateTrajectory()
    p = trajectory[0]

    desPos.x = p[0]
    desPos.y = p[1]
    desPos.z = p[2]

    while not rospy.is_shutdown():
        if drone.isThere():
            p = trajectory[0]
            trajectory.append(trajectory.pop(0))
            desPos.x = p[0]
            desPos.y = p[1]
            desPos.z = p[2]
        
        desPos.x = 1
        desPos.y = 1
        desPos.z = 1

        desVel.x = 0
        desVel.y = 0
        desVel.z = 0

        desiredPositionPublisher.publish(desPos)
        desiredVelocityPublisher.publish(desVel)
        rospy.sleep(0.1)




if __name__ == '__main__':
    try:
        rospy.init_node('drone_traj_follow', anonymous=True)
        rospy.loginfo('Starting....')
        trajFollow()
    except rospy.ROSInterruptException:
        rospy.logwarn('Exiting....')