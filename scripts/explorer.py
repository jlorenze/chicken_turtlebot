#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from std_msgs.msg import Float32MultiArray, String, Bool
import tf
import numpy as np
from numpy import linalg
from utils import wrapToPi
from astar import AStar
from grids import StochOccupancyGrid2D
import scipy.interpolate
import matplotlib.pyplot as plt


# threshold at which navigator switches
# from trajectory to pose control
END_POS_THRESH = .2

MAX_SEARCH_INT = 100


class Explorer:

    def __init__(self):
        rospy.init_node('turtlebot_explorer', anonymous=True)

        # current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # goal state
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.0

	self.active = False

        self.current_plan = []

        # map parameters
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0,0]
        self.map_probs = []
        self.occupancy = None
        self.occupancy_updated = False

	self.has_started = False

	self.nav_resolution = 0.1

        self.nav_pos_pub = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)

        self.trans_listener = tf.TransformListener()

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
        rospy.Subscriber('/explore_yn', Bool, self.activation_callback)

    def activation_callback(self, msg):
        self.active = msg.data

    def map_md_callback(self, msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
        self.map_probs = msg.data
        if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  8,
                                                  self.map_probs)
            self.occupancy_updated = True

    def close_to_end_location(self):
        return (abs(self.x-self.x_g)<END_POS_THRESH and abs(self.y-self.y_g)<END_POS_THRESH and abs(self.theta-self.theta_g)<END_POS_THRESH)

    def snap_to_grid(self, x):
        return (self.map_resolution*round(x[0]/self.map_resolution), self.map_resolution*round(x[1]/self.map_resolution))

    def close_to_start_location(self):
        if len(self.current_plan)>0:
            snapped_current = self.snap_to_grid([self.x, self.y])
            snapped_start = self.snap_to_grid(self.current_plan_start_loc)
            return (abs(snapped_current[0]-snapped_start[0])<START_POS_THRESH and abs(snapped_current[1]-snapped_start[1])<START_POS_THRESH)
        return False

    def get_neighbors(self, pos):
	step = self.map_resolution
	return [self.snap_to_grid((pos[0] - step, pos[1] - step)), self.snap_to_grid((pos[0] + step, pos[1] - step)), self.snap_to_grid((pos[0] - step, pos[1] + step)), self.snap_to_grid((pos[0] + step, pos[1] + step))]

    def get_prob(self, pos):
        # combine the probabilities of each cell by assuming independence
        # of each estimation
	p_val = 0
        lower = -int(round((self.occupancy.window_size-1)/2))
        upper = int(round((self.occupancy.window_size-1)/2))
        for dx in range(lower,upper+1):
            for dy in range(lower,upper+1):
                x, y = self.occupancy.snap_to_grid([pos[0] + dx * self.occupancy.resolution, pos[1] + dy * self.occupancy.resolution])
                grid_x = int((x - self.occupancy.origin_x) / self.occupancy.resolution)
                grid_y = int((y - self.occupancy.origin_y) / self.occupancy.resolution)
                if grid_y>0 and grid_x>0 and grid_x<self.occupancy.width and grid_y<self.occupancy.height:
                    p_val = self.occupancy.probs[grid_y * self.occupancy.width + grid_x]
        return p_val

    def grid_search(self):
	i = 0
	searched_list = {}
	searched_list[self.snap_to_grid((self.x, self.y))] = 0

	while i < MAX_SEARCH_INT:
		i += 1
		for point in searched_list.keys():
			if (searched_list[point] == (i-1)):
				for neigh in self.get_neighbors(point):
					if neigh not in searched_list.keys():
						neigh_val = self.get_prob(neigh)
						if self.occupancy.is_free(neigh):
							searched_list[neigh] = i
						elif (neigh_val is -1) and (i>10):
							return neigh
	return None

    def run_explorer(self):
        """ computes a path from current state to goal state using A* and sends it to the path controller """

        # makes sure we have a location
        try:
            (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.x = translation[0]
            self.y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.theta = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.current_plan = []
            return

        # makes sure we have a map
        if not self.occupancy:
            # self.current_plan = []
            return

        # if there is no plan, we are far from the start of the plan,
        # or the occupancy grid has been updated, update the current plan
        if self.close_to_end_location() or (not self.has_started) or (self.get_prob((self.x_g, self.y_g)) != -1): # or self.occupancy_updated:
		self.occupancy_updated = False
		self.has_started = True

		nav_point = self.grid_search()
		print("pos: ", (self.x, self.y))
		print("goal: ", nav_point)
		if nav_point:
			self.x_g = nav_point[0]
			self.y_g = nav_point[1]
			self.theta_g = 0.0

			nav_msg = Pose2D()
			nav_msg.x = self.x_g
			nav_msg.y = self.y_g
			nav_msg.theta = self.theta_g
			self.nav_pos_pub.publish(nav_msg)


if __name__ == '__main__':
    exp = Explorer()
    while True:
	if exp.active:
		exp.run_explorer()
	rospy.Rate(2)
    rospy.spin()
