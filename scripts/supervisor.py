#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum
import numpy as np
import pdb

# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 3

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .5

# time taken to cross an intersection
CROSSING_TIME = 3

def wrapToPi(a):
    if isinstance(a, list):
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6

class Supervisor:
    """ the state machine of the turtlebot """

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)

        # current pose
        self.x = 0
        self.y = 0
        self.theta = 0

        # pose goal
        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0

        # current mode
        self.mode = Mode.IDLE
        self.last_mode_printed = None
        self.stop_thresh = 0.7

        # stop signs
        self.stopSigns = []
        self.stopSignCounts = []

        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

        self.trans_listener = tf.TransformListener()

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """

        self.x_g = msg.pose.position.x
        self.y_g = msg.pose.position.y
        rotation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(rotation)
        self.theta_g = euler[2]

        self.mode = Mode.NAV

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

        # Now that we have x and y coord of stop sign in world frame, append coord
        found = False
        for i, stopSign in enumerate(self.stopSigns):
            distance = np.sqrt((x - stopSign[0])**2 + (y - stopSign[1])**2)
            n = self.stopSignCounts[i]
            if distance < .2:
                if n < 100:
                    # We have found the same stop sign as before
                    xnew = (n/(n+1.))*stopSign[0] + (1./(n+1))*x
                    ynew = (n/(n+1.))*stopSign[1] + (1./(n+1))*y
                    self.stopSigns[i] = np.array([xnew, ynew])
                    self.stopSignCounts[i] += 1
                found = True
        
        if not found:
            # Found a new one, append it
            self.stopSigns.append(np.array([x,y]))
            self.stopSignCounts.append(1)

        # # If close enough and in nav mode, stop
        # if dist > 0 and r > self.stop_thresh and self.mode == Mode.NAV:
        #     self.init_stop_sign()

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.nav_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return (self.mode == Mode.CROSS and (rospy.get_rostime()-self.cross_start)>rospy.Duration.from_sec(CROSSING_TIME))

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        try:
            (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.x = translation[0]
            self.y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.theta = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass



        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            self.stay_idle()

        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                pass

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.has_crossed():
                self.mode = Mode.NAV
            else:
                self.nav_to_pose()

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.nav_to_pose()

        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            # self.loop()
            rate.sleep()
        data = np.asarray(self.stopSigns)
        print data
        print self.stopSignCounts

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
