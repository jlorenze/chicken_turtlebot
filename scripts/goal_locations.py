#!/usr/bin/env python
# license removed for brevity
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Pose2D, PoseStamped
import numpy as np
#from asl_turtlebot.msg import DetectedObject
import tf

def Publish_Goals():
    # pub = rospy.Publisher('Goals', PoseStamped, queue_size=10)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('Publish_Goals', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    listener = tf.TransformListener()
    pose_g_msg = PoseStamped()

    num_animals = 3


    while not rospy.is_shutdown():
	#I Added this to a working node
        for i in range(num_animals):
            goal_dist = 9000
            while (goal_dist > 0.2):
                try:
                    (bot_loc,bot_ang) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                    (translation_Cat,rotation_Cat) = listener.lookupTransform('/map', '/Animal0'+str(i), rospy.Time(0))

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
                    print('dist to goal %s/%s: %s' % (i+1, num_animals, goal_dist))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    print('Fail')
                    pass

                pub.publish(pose_g_msg)
                rate.sleep()

if __name__ == '__main__':
    try:
        Publish_Goals()
    except rospy.ROSInterruptException:
        pass
