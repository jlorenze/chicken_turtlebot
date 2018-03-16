#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum
from geometry_msgs.msg import Pose2D
from chicken_turtlebot.msg import DetectedAnimal
#from asl_turtlebot.msg import DetectedObject
import tf

def Publish_to_tf():
    #pub = rospy.Publisher('tf_pub', Pose2D, queue_size=10)
    pub = rospy.Publisher('Animal_List', DetectedAnimal, queue_size=10)
    rospy.init_node('Publish_tf_frames', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()
        br.sendTransform((2.1, 2.5, 0.), tf.transformations.quaternion_from_euler(0, 0, 3.14), rospy.Time.now(), 'Stop00', 'map')
        br.sendTransform((2.6, 0.6, 0.), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 'Stop01', 'map')
        br.sendTransform((0.6, 0.6, 0.), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 'Stop02', 'map')
        br.sendTransform((0.6, 1.8, 0.), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 'Stop03', 'map')
        br.sendTransform((2.0, 0.7, 0.), tf.transformations.quaternion_from_euler(0, 0, 1.57), rospy.Time.now(), 'Stop04', 'map')
        br.sendTransform((2.6, 2.5, 0.), tf.transformations.quaternion_from_euler(0, 0, -1.57), rospy.Time.now(), 'Stop05', 'map')
        br.sendTransform((2.0, 1.8, 0.), tf.transformations.quaternion_from_euler(0, 0, 1.57), rospy.Time.now(), 'Stop06', 'map')

        br.sendTransform((3.2, 1.5, 0.), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 'BaseStation', 'map')
        br.sendTransform((2, 0.5, 0.), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 'Animal00', 'map')
        br.sendTransform((1.0, 1.5, 0.), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 'Animal01', 'map')
        br.sendTransform((0.5, 0.5, 0.), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), 'Animal02', 'map')

        msg = DetectedAnimal()
        msg.threadnames = ['Animal00', 'Animal01', 'Animal02']

        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        Publish_to_tf()
    except rospy.ROSInterruptException:
        pass
