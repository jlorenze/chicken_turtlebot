#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
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
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        self.trans_listener = tf.TransformListener()

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        corners = msg.corners
        dx = corners[3] - corners[1]
        dy = corners[2] - corners[0]

        r = dx/dy # aspect ratio

        rdist = np.array([.15, .20, .25, .30,.35, .40, .45, .50])
        pixelheight = np.array([139, 102, 82, 64, 56, 50, 44, 40])
        if dy > pixelheight[-1] and dy < pixelheight[0]:
            dist = np.interp(dy, pixelheight[::-1], rdist[::-1])
        else:
            dist = 0

        # Get location of camera with respect to the map
        try:
            (translation,rotation) = self.trans_listener.lookupTransform('/map', '/camera', rospy.Time(0))
            xcam = translation[0]
            ycam = translation[1]
            zcam = translation[2]
            euler = tf.transformations.euler_from_quaternion(rotation)
            thetacam = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # Now we have pose of robot, we want to determine stop sign angle relative
        # to camera frame
        thstopsign = (wrapToPi(msg.thetaright) + wrapToPi(msg.thetaleft))/2.
        zstopsign = dist*np.cos(-thstopsign)
        xstopsign = dist*np.sin(-thstopsign)

        x = xcam + xstopsign*np.cos(thetacam) - zstopsign*np.sin(thetacam) 
        y = ycam + xstopsign*np.sin(thetacam) + zstopsign*np.cos(thetacam)
        print x,y

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    sup = Testing()
    sup.run()