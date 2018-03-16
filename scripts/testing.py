#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Bool
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from chicken_turtlebot.msg import DetectedObject
from matplotlib import pyplot as plt
import numpy as np
import pdb
import tf

def wrapToPi(a):
    if isinstance(a, list):
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi

class Testing:
    """ the state machine of the turtlebot """

    def __init__(self):
        rospy.init_node('turtlebot_tester', anonymous=True)
        self.start = rospy.Publisher('/explore_yn', Bool, queue_size=10)

    def startExplorer(self):
        object_msg = Bool()
        object_msg.data = True
        self.start.publish(object_msg)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.startExplorer()
            rate.sleep()
        

if __name__ == '__main__':
    sup = Testing()
    sup.run()