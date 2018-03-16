#!/usr/bin/env python
# license removed for brevity
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Bool
from geometry_msgs.msg import Pose2D, PoseStamped
import numpy as np
from enum import Enum
#from asl_turtlebot.msg import DetectedObject
import tf

from chicken_turtlebot.msg import DetectedAnimal

class Phase(Enum):
    EXPLORE = 1
    RETURN = 2
    RESCUE = 3

class Phase_Supervisor():
    def __init__(self):
        rospy.init_node('Publish_Goals', anonymous=True)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.ready_pub = rospy.Publisher('/ready_to_rescue', Bool, queue_size=10)
        self.listener = tf.TransformListener()
        rospy.Subscriber('/animals', DetectedAnimal , self.Track_Animals_Callback) # INCOMPLETE
        rospy.Subscriber('/Return_Ready', Bool, self.Return_Ready_Callback)
        rospy.Subscriber('/rescue_on', Bool, self.Rescue_Ready_Callback)
        self.phase = Phase.EXPLORE

        self.startx = None
        self.starty = None

        initial_pose = PoseStamped()
        for i in range(10): # Hacky shit - try to get the location of the turtle bot 10 times since it may fail the first few times.
            try:
                (bot_loc,bot_ang) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                self.startx = bot_loc[0]
                self.starty = bot_loc[1]
            except:
                pass

    def Navigate(self, goal_string = None):
        # pub = rospy.Publisher('Goals', PoseStamped, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        pose_g_msg = PoseStamped()

        # while not rospy.is_shutdown():
	    #I Added this to a working node
        goal_dist = 9000
        while (goal_dist > 0.2):
            try:
                (bot_loc,bot_ang) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                if (goal_string != None):
                    (translation_Cat,rotation_Cat) = self.listener.lookupTransform('/map', '/'+goal_string, rospy.Time(0))
                else:
                    translation_Cat = [self.startx, self.starty,0]


                # build the message
                pose_g_msg.pose.position.x = translation_Cat[0]
                pose_g_msg.pose.position.y = translation_Cat[1]
                # pose_g_msg.theta = 0
                pose_g_msg.pose.orientation.x = 0
                pose_g_msg.pose.orientation.y = 0
                pose_g_msg.pose.orientation.z = 0
                pose_g_msg.pose.orientation.w = 0

                state = np.asarray(bot_loc)
                goal = np.asarray(translation_Cat)

                goal_dist = np.sum((goal - state)**2)**0.5
                print('dist to goal: %s' % goal_dist)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print('Fail')
                pass

            self.pub.publish(pose_g_msg)
            rate.sleep()
        print('Arrived')

    def Track_Animals_Callback(self, msg):
        self.animal_list = msg.threadnames # this may change somewhat

    def Return_Ready_Callback(self, msg):
        if (msg.data == True):
            self.phase = Phase.RETURN

    def Rescue_Ready_Callback(self, msg):
        if (msg.data == True):
            self.phase = Phase.RESCUE

    def Rescue(self):

        # Do some smart sorting here

        for animal in self.animal_list:
            self.Navigate(animal)

    def loop(self):
        if self.phase == Phase.EXPLORE:
            pass

        elif self.phase == Phase.RETURN:
            self.Navigate()
            ready_msg = Bool()
            ready_msg.data = True
            self.ready_pub.publish(ready_msg)

        elif self.phase == Phase.RESCUE:
            self.Rescue() # get all animals
            self.Navigate() # Bring them home!
            self.phase = Phase.EXPLORE # give control back to us.

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if (self.startx == None or self.starty == None):
                try:
                    (bot_loc,bot_ang) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                    self.startx = bot_loc[0]
                    self.starty = bot_loc[1]
                    print 'FINALLY!'
                except:
                    print 'Where am I?'
                    pass
            else:
                self.loop()
            print self.phase
            rate.sleep()

if __name__ == '__main__':
    try:
        phase_sub = Phase_Supervisor()
        phase_sub.run()
    except rospy.ROSInterruptException:
        pass
