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

class Demo_Node():
    def __init__(self):
        rospy.init_node('Demo_Node', anonymous=True)
        #rospy.Subscriber('/animals', DetectedAnimal , self.Track_Animals_Callback) # INCOMPLETE?  Detected Animals
        #rospy.Subscriber('/stopSigns', DetectedStopSign, self.stopsign_callback) # Detected Stopsigns
        #rospy.Subscriber('/Return_Ready', Bool, self.Return_Ready_Callback)   #Ready for rescue
        #rospy.Subscriber('/rescue_on', Bool, self.Rescue_Ready_Callback)
        #rospy.Subscriber('/rescue_on', Bool, self.Rescue_Ready_Callback)
        rospy.Subscriber('/Phase_Sup_Status', String, self.Phase_Sup_Callback)
        rospy.Subscriber('/sup_status',  String, self.Sup_Callback)
        self.msg_Phase_Sup='Starting Phase Sup'

    def Track_Animals_Callback(self, msg):
        self.animal_list = msg.threadnames # this may change somewhat

    def Phase_Sup_Callback(self, msg):
        if msg.data != self.msg_Phase_Sup:
            print(msg.data)
            self.msg_Phase_Sup=msg.data

    def Sup_Callback(self, msg):
        print(msg.data)
  

    def loop(self):
        rospy.spin()


    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    d_node = Demo_Node()
    d_node.run()

