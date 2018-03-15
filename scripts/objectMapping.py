#!/usr/bin/env python

import rospy
import tf
import numpy as np
from chicken_turtlebot.msg import DetectedObject, DetectedStopSign, DetectedAnimal
from geometry_msgs.msg import PoseStamped
import pdb

def wrapToPi(a):
    if isinstance(a, list):
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi

class ObjectMapping:

    def __init__(self):
        rospy.init_node('turtlebot_object_mapping', anonymous=True)

        # Store topic names for detected objects
        self.topicNames = []

        # Stop Signs
        self.stopSigns = [[],[],[]]
        self.stopSignCounts = []
        self.stopSignPublisher = rospy.Publisher('/stopSigns', DetectedStopSign, queue_size=10)

        # Animals
        self.animals = [[],[],[]]
        self.animalCounts = []
        self.animalPublisher = rospy.Publisher('/animals', DetectedAnimal, queue_size=10)

        self.animal_pos_pub = rospy.Publisher('/animal_pos', PoseStamped, queue_size=10)

        self.tf_listener = tf.TransformListener()
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)

        self.tf_broadcaster = tf.TransformBroadcaster()

    def searchForTopics(self):
        topics = rospy.get_published_topics(namespace='/detector')
        for topic in topics:
            if topic[0] not in self.topicNames and topic[0] != '/detector/stop_sign':
                self.topicNames.append(topic[0])
                rospy.Subscriber(topic[0], DetectedObject, self.animal_detected_callback)

    def animal_detected_callback(self, msg):
        # Get animal name
        animal = msg.name

        # Compute bounding box width and height in pixels
        corners = msg.corners
        dx = corners[3] - corners[1]
        dy = corners[2] - corners[0]
        r = dx/dy # aspect ratio

        # Use linear interpolation from test data
        rdist = np.array([.26, .31, .36, .41, .46, .51, .56])
        pixelheight = np.array([120, 108, 95, 86, 71, 67, 65])
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

        # Now we have pose of robot, we want to determine stop sign angle relative
        # to camera frame
        thanimal = (wrapToPi(msg.thetaright) + wrapToPi(msg.thetaleft))/2.
        zanimal = dist*np.cos(-thanimal)
        xanimal = dist*np.sin(-thanimal)

        x = xcam + xanimal*np.cos(thetacam) - zanimal*np.sin(thetacam) 
        y = ycam + xanimal*np.sin(thetacam) + zanimal*np.cos(thetacam)

        # Now that we have x and y coord of animal in world frame, append coord
        found = False
        for i in range(len(self.animals[0])):
            xcur = self.animals[0][i]
            ycur = self.animals[1][i]
            distance = np.sqrt((x - xcur)**2 + (y - ycur)**2)
            n = self.animalCounts[i]
            if distance < .2 and self.animals[2][i] == animal:
                if n < 100:
                    # We have found the same stop sign as before
                    xnew = (n/(n+1.))*xcur + (1./(n+1))*x
                    ynew = (n/(n+1.))*ycur + (1./(n+1))*y
                    self.animals[0][i] = xnew
                    self.animals[1][i] = ynew
                    self.animalCounts[i] += 1
                found = True
        
        if not found:
            # Found a new one, append it
            self.animals[0].append(x)
            self.animals[1].append(y)
            self.animals[2].append(animal)
            self.animalCounts.append(1)


        # Publish locations to TF tree
        for i in range(len(self.animals[0])):
            x = self.animals[0][i]
            y = self.animals[1][i]
            z = 0.
            animal = self.animals[2][i]
            self.tf_broadcaster.sendTransform((x,y,z), tf.transformations.quaternion_from_euler(0,0,0),
                                    rospy.Time.now(),animal,'map')

        # #  Publishes the detected object and its location
        # object_msg = DetectedAnimal()
        # object_msg.x = self.animals[0]
        # object_msg.y = self.animals[1]
        # object_msg.animal = self.animals[2]
        # self.animalPublisher.publish(object_msg)

        # # publish current desired x and y for visualization only
        # pathsp_msg = PoseStamped()
        # pathsp_msg.header.frame_id = 'map'
        # pathsp_msg.pose.position.x = self.animals[0][0]
        # pathsp_msg.pose.position.y = self.animals[1][0]
        # theta_d = 0.0
        # quat_d = tf.transformations.quaternion_from_euler(0, 0, theta_d)
        # pathsp_msg.pose.orientation.x = quat_d[0]
        # pathsp_msg.pose.orientation.y = quat_d[1]
        # pathsp_msg.pose.orientation.z = quat_d[2]
        # pathsp_msg.pose.orientation.w = quat_d[3]
        # self.animal_pos_pub.publish(pathsp_msg)

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
        rate = rospy.Rate(100) # 10 Hz
        while not rospy.is_shutdown():
            self.searchForTopics()
            rate.sleep()


if __name__=='__main__':
    o = ObjectMapping()
    o.run()