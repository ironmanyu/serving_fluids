#!/usr/bin/env python

import argparse

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
import math
import tf
from tf.transformations import quaternion_from_euler
import numpy as np
import fetch_api

def pose_from_xyY(x, y, Y):
    # returns a pose given x and y position coordinates (m) and a Y yaw value (rad)
    pose = Pose()
    # position
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    # orientation
    (x, y, z, w) = quaternion_from_euler(0.0, 0.0, Y)
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w
    return pose

class NavGoalClient():
    def __init__(self):
        '''create a NavGoalClient'''
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def send_goal(self, x, y, yaw, frame='map'):
        '''go to goal with x and y coordinates (m) and yaw orientation (rad)
         in the specified frame'''
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        # set position
        pose = pose_from_xyY(x, y, yaw)
        goal.target_pose.pose = pose
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        return result

if __name__ == '__main__':
    # parse command line arguments
    print('parsing command line arguments')
    parser = argparse.ArgumentParser(description="navigate to a series of poses")
    parser.add_argument('-d', '--doors', action='store_true') # default False
    args, unknown = parser.parse_known_args()
    # print(args.doors)

    # start the node
    print('starting the node')
    rospy.init_node('navigate_irl')

    # start the NavGoalClient
    print('creating the NavGoalClient')
    nav_client = NavGoalClient()

    # TODO: navigate to kitchen counter
    def go_to_kitchen():
        # drive to kitchen doorway
        print('driving to kitchen doorway')
        result = nav_client.send_goal(-1.8, 0.5, math.pi/2)
        if args.doors:
            # drive through kitchen doorway
            print('driving through kitchen doorway')
            result = nav_client.send_goal(-1.8, 2.5, 0.0)
    go_to_kitchen()

    # TODO: navigate to dining table
    # drive to bench area
    print('driving to bench area')
    result = nav_client.send_goal(-4.1, 0.5, math.pi/2)
    if args.doors:
        # drive up to dining table
        print('driving to dining table')
        result = nav_client.send_goal(-4.1, 2.0, math.pi/2)

    # TODO: navigate to kitchen counter
    go_to_kitchen()

    # TODO: navigate to starting point in hallway
    # drive to start of hallway
    print('driving to start of hallway')
    result = nav_client.send_goal(1.4, 0.5, math.pi/2)
    if args.doors:
        # drive down hallway
        print('driving down hallway')
        result = nav_client.send_goal(1.4, 5.0, -math.pi/2)

    rospy.signal_shutdown("Goal sequence completed!")