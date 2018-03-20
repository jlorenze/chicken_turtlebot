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
        self.lookFor = ['/detector/cat','/detector/dog','/detector/bear','/detector/elephant', '/detector/teddybear']
        self.animals = []
        self.actualAnimals = []
        self.animalPublisher = rospy.Publisher('/animals', DetectedAnimal, queue_size=10)

        self.animal_pos_pub = rospy.Publisher('/animal_pos', PoseStamped, queue_size=10)

        self.tf_listener = tf.TransformListener()
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)

        self.tf_broadcaster = tf.TransformBroadcaster()

    def searchForTopics(self):
        topics = rospy.get_published_topics(namespace='/detector')
        for topic in topics:
            if topic[0] not in self.topicNames and topic[0] in self.lookFor:
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
            dist = rdist[-1]

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

        try:
            (translation,rotation) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            xrobot = translation[0]
            yrobot = translation[1]
            zrobot = translation[2]
            euler = tf.transformations.euler_from_quaternion(rotation)
            thetarobot = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        # Now we have pose of robot, we want to determine stop sign angle relative
        # to camera frame
        thanimal = (wrapToPi(msg.thetaright) + wrapToPi(msg.thetaleft))/2.
        zanimal = dist*np.cos(-thanimal)
        xanimal = dist*np.sin(-thanimal)

        x = xrobot
        y = yrobot
        # Now that we have x and y coord of animal in world frame, append coord
        found = False
        for i in range(len(self.animals)):
            xcur = self.animals[i][0]
            ycur = self.animals[i][1]
            distance = np.sqrt((x - xcur)**2 + (y - ycur)**2)
            if distance < .4:
                # We have found something here before
                foundAnimal = False
                for j in range(len(self.animals[i][2])):
                    n = self.animals[i][3][j]
                    
                    # We are in close proximity to old animal found
                    if self.animals[i][2][j] == animal:
                        # Probably same animal, just update
                        # We have found the same animal as before
                        xnew = (n/(n+1.))*xcur + (1./(n+1))*x
                        ynew = (n/(n+1.))*ycur + (1./(n+1))*y
                        self.animals[i][0] = xnew
                        self.animals[i][1] = ynew
                        self.animals[i][3][j] += 1
                        foundAnimal = True

                if not foundAnimal:
                    # New animal in this location
                    self.animals[i][2].append(animal)
                    self.animals[i][3].append(1)

                found = True
        
        if not found:
            # Found a new one, append it
            newanimal = [x, y, [animal], [1]]
            self.animals.append(newanimal)


        x = xcam + xanimal*np.cos(thetacam) - zanimal*np.sin(thetacam) 
        y = ycam + xanimal*np.sin(thetacam) + zanimal*np.cos(thetacam)
        found = False
        for i in range(len(self.actualAnimals)):
            xcur = self.actualAnimals[i][0]
            ycur = self.actualAnimals[i][1]
            distance = np.sqrt((x - xcur)**2 + (y - ycur)**2)
            if distance < .4:
                # We have found something here before
                foundAnimal = False
                for j in range(len(self.actualAnimals[i][2])):
                    n = self.actualAnimals[i][3][j]
                    
                    # We are in close proximity to old animal found
                    if self.actualAnimals[i][2][j] == animal:
                        # Probably same animal, just update
                        # We have found the same animal as before
                        xnew = (n/(n+1.))*xcur + (1./(n+1))*x
                        ynew = (n/(n+1.))*ycur + (1./(n+1))*y
                        self.actualAnimals[i][0] = xnew
                        self.actualAnimals[i][1] = ynew
                        self.actualAnimals[i][3][j] += 1
                        foundAnimal = True

                if not foundAnimal:
                    # New animal in this location
                    self.actualAnimals[i][2].append(animal)
                    self.actualAnimals[i][3].append(1)

                found = True
        
        if not found:
            # Found a new one, append it
            newanimal = [x, y, [animal], [1]]
            self.actualAnimals.append(newanimal)

    def publishToTree(self):
        # Publish locations of animals to TF tree
        animals_published = []
        animal_threadnames = []
        for i in range(len(self.animals)):
            counts = np.array(self.animals[i][3], 'float64')
            ratios = counts/np.sum(counts)
            for j in range(len(self.animals[i][2])):
                if ratios[j] > 0:
                    x = self.animals[i][0]
                    y = self.animals[i][1]
                    z = 0.
                    animal = self.animals[i][2][j]
                    if animal in animals_published:
                        N = animals_published.count(animal)
                        threadname = animal + str(int(N))
                    else:
                        threadname = animal + str(0)
                    self.tf_broadcaster.sendTransform((x,y,z), tf.transformations.quaternion_from_euler(0,0,0),
                                            rospy.Time.now(), threadname, 'map')
                    animals_published.append(animal)
                    animal_threadnames.append(threadname)

        object_msg = DetectedAnimal()
        object_msg.threadnames = animal_threadnames
        self.animalPublisher.publish(object_msg)

        # Publish locations of actual animals to TF tree
        actualAnimals_published = []
        for i in range(len(self.actualAnimals)):
            counts = np.array(self.actualAnimals[i][3], 'float64')
            ratios = counts/np.sum(counts)
            for j in range(len(self.actualAnimals[i][2])):
                if ratios[j] > 0:
                    x = self.actualAnimals[i][0]
                    y = self.actualAnimals[i][1]
                    z = 0.
                    animal = self.actualAnimals[i][2][j]
                    if animal in actualAnimals_published:
                        N = actualAnimals_published.count(animal)
                        threadname = 'actual_' + animal + str(int(N))
                    else:
                        threadname = 'actual_' + animal + str(0)
                    self.tf_broadcaster.sendTransform((x,y,z), tf.transformations.quaternion_from_euler(0,0,0),
                                            rospy.Time.now(), threadname, 'map')
                    actualAnimals_published.append(animal)

        # Publis locations of stop signs to TF tree
        stopsign_threadnames = []
        for i in range(len(self.stopSigns[0])):
            x = self.stopSigns[0][i]
            y = self.stopSigns[1][i]
            z = 0.
            theta = self.stopSigns[2][i]
            threadname = 'StopSign' + str(i)
            self.tf_broadcaster.sendTransform((x,y,z), tf.transformations.quaternion_from_euler(0,0,theta),
                                                rospy.Time.now(), threadname, 'map')
            stopsign_threadnames.append(threadname)

        #  Publishes the stop sign threadnames
        object_msg = DetectedStopSign()
        object_msg.threadnames = stopsign_threadnames
        self.stopSignPublisher.publish(object_msg)

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

    def run(self):
        rate = rospy.Rate(100) # 10 Hz
        while not rospy.is_shutdown():
            self.searchForTopics()
            self.publishToTree()
            rate.sleep()


if __name__=='__main__':
    o = ObjectMapping()
    o.run()
