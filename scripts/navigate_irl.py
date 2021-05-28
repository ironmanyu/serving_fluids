#!/usr/bin/env python

# import sys, getopt # for command line arguments
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

def set_pose_estimate(x, y, yaw):
    pose_est_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    while pose_est_pub.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscriber to connect")
        rospy.sleep(1)
    pose_estimate = PoseWithCovarianceStamped()
    pose_estimate.header.frame_id = "map"
    pose_estimate.header.stamp = rospy.Time.now()
    pose_estimate.pose.pose = pose_from_xyY(x, y, yaw)
    # print(pose_estimate)
    pose_est_pub.publish(pose_estimate)

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
    parser = argparse.ArgumentParser(description="set 2D Pose Estimate and navigate to some goal poses")
    parser.add_argument('-x', default=0.0, type=float)
    parser.add_argument('-y', default=0.0, type=float)
    parser.add_argument('-Y', default=0.0, type=float)
    args = parser.parse_args()

    # start the node
    rospy.init_node('GoalSequence')

    # set the 2D Pose Estimate
    set_pose_estimate(args.x, args.y, args.Y)

    # start the NavGoalClient
    nav_client = NavGoalClient()
    
    # start the base
    base = fetch_api.Base()

    # TODO: navigate to kitchen counter
    # drive to kitchen doorway
    result = nav_client.send_goal(-2.1, 1.0, math.pi/2)
    # drive through kitchen doorway
    result = nav_client.send_goal()

    # TODO: navigate to dining table

    # TODO: navigate to kitchen conter

    # TODO: navigate to starting point in hallway

    rospy.signal_shutdown("Goal sequence completed!")