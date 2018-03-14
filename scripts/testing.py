#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
from matplotlib import pyplot as plt
import numpy as np
import pdb


class Testing:
    """ the state machine of the turtlebot """

    def __init__(self):
        rospy.init_node('turtlebot_tester', anonymous=True)
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        corners = msg.corners
        dx = corners[3] - corners[1]
        dy = corners[2] - corners[0]

        r = dx/dy

        rdist = np.array([.15, .20, .25, .30,.35, .40, .45, .50])
        pixelheight = np.array([139, 102, 82, 64, 56, 50, 44, 40])
        if dy > pixelheight[-1] and dy < pixelheight[0]:
            dist = np.interp(dy, pixelheight[::-1], rdist[::-1])
        else:
            dist = 0

        print dist


    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    sup = Testing()
    sup.run()