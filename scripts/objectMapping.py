#!/usr/bin/env python

import rospy
import tf
import numpy as np
from chicken_turtlebot.msg import DetectedObject, DetectedStopSign

def wrapToPi(a):
    if isinstance(a, list):
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi

class ObjectMapping:

    def __init__(self):
        rospy.init_node('turtlebot_object_mapping', anonymous=True)

        # Stop Signs
        self.stopSigns = [[],[],[]]
        self.stopSignCounts = []
        self.stopSignPublisher = rospy.Publisher('/stopSigns', DetectedStopSign, queue_size=10)

        self.tf_listener = tf.TransformListener()
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)

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
            return

        # Get location of camera with respect to the map
        try:
            (translation,rotation) = self.tf_listener.lookupTransform('/map', '/camera', rospy.Time(0))
            xcam = translation[0]
            ycam = translation[1]
            zcam = translation[2]
            euler = tf.transformations.euler_from_quaternion(rotation)
            thetacam = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # Get angle of robot with respect to the map
        try:
            (translation,rotation) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rotation)
            thetarobot = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # Now we have pose of robot, we want to determine stop sign angle relative
        # to camera frame
        thstopsign = (wrapToPi(msg.thetaright) + wrapToPi(msg.thetaleft))/2.
        zstopsign = dist*np.cos(-thstopsign)
        xstopsign = dist*np.sin(-thstopsign)

        x = xcam + xstopsign*np.cos(thetacam) - zstopsign*np.sin(thetacam) 
        y = ycam + xstopsign*np.sin(thetacam) + zstopsign*np.cos(thetacam)

        # Now that we have x and y coord of stop sign in world frame, append coord
        found = False
        for i in range(len(self.stopSigns[0])):
            xcur = self.stopSigns[0][i]
            ycur = self.stopSigns[1][i]
            thetarobotcur = self.stopSigns[2][i]
            distance = np.sqrt((x - xcur)**2 + (y - ycur)**2)
            n = self.stopSignCounts[i]
            if distance < .2:
                if n < 100:
                    # We have found the same stop sign as before
                    xnew = (n/(n+1.))*xcur + (1./(n+1))*x
                    ynew = (n/(n+1.))*ycur + (1./(n+1))*y
                    thetarobotnew = (n/(n+1.))*thetarobotcur + (1./(n+1))*thetarobot
                    self.stopSigns[0][i] = xnew
                    self.stopSigns[1][i] = ynew
                    self.stopSigns[2][i] = thetarobotnew
                    self.stopSignCounts[i] += 1
                found = True
        
        if not found:
            # Found a new one, append it
            self.stopSigns[0].append(x)
            self.stopSigns[1].append(y)
            self.stopSigns[2].append(thetarobot)
            self.stopSignCounts.append(1)

        #  Publishes the detected object and its location
        object_msg = DetectedStopSign()
        object_msg.x = self.stopSigns[0]
        object_msg.y = self.stopSigns[1]
        object_msg.theta = self.stopSigns[2]
        self.stopSignPublisher.publish(object_msg)

    def run(self):
        rospy.spin()


if __name__=='__main__':
    o = ObjectMapping()
    o.run()