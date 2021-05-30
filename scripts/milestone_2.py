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

def drive_straight_through_narrow_space(distance, speed=0.1):
    '''drive straight forward by distance (meters), with a speed of speed (m/s)
    The robot will stop if it sees an obstacle less than 1 second away at current speed.'''
    base = fetch_api.Base()
    tf_listener = tf.TransformListener()
    rate = rospy.Rate(10) # 10 Hz
    start_trans = None
    # drive forward until we have travelled distance
    while not rospy.is_shutdown():
        # stop if too close to a wall
        laser_scan = rospy.wait_for_message("base_scan", LaserScan)
        if (np.min(laser_scan.ranges) < speed):
            # stop if the robot is within 1 second of hitting an obstacle
            # distance traveled in 1 second = speed (m/s) * 1 (s) = distance (m)
            continue # don't move forward
        # figure out how far the robot has driven already
        (trans, rot) = tf_listener.lookupTransform("/odom", "/base_link", rospy.Time(0))
        trans = np.array(trans)
        if start_trans is None: # initialize the beginning transform
            start_trans = trans
        if np.linalg.norm(trans - start_trans) >= distance:
            # we reached the goal distance
            break
        # otherwise keep driving forward
        base.move(speed, 0.0)

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
    # drive forward down the hallway
    drive_straight_through_narrow_space(4.0, speed=0.5) # 4 meters
    # drive to kitchen doorway
    result = nav_client.send_goal(-2.1, 1.0, math.pi/2)
    # drive through kitchen doorway
    drive_straight_through_narrow_space(1.5)
    # turn to face table
    base.turn(-90) # 90 deg clockwise

    # TODO: navigate to dining table

    # TODO: navigate to kitchen conter

    # TODO: navigate to starting point in hallway

<<<<<<< HEAD
    # Initialize class
    sequence = GoalSequence()
    # Declare your relevant waypoints' positions and orientations as a list.
    # They should be the same LENGTH

    waypoints_pos = list()
    #waypoints_pos.append([-2.0, 1.0, 0.000]) # Booth
    #waypoints_pos.append([-2.0, 3.0, 0.000]) # Kitchen Doorway
    #waypoints_pos.append([-0.4, 3.1, 0.000]) # Near the desk
    #waypoints_pos.append([-1.7, 3.0, 0.000]) # Kitchen Doorway
    #waypoints_pos.append([-1.7, 1.0, 0.000]) # Booth
    waypoints_pos.append([-4.4, 2.0, 0.000])
    waypoints_pos.append([-4.4, -1.0, 0.000])
    waypoints_pos.append([-7.0, -1.0, 0.000]) # Canteen Doorway
    waypoints_pos.append([-8.0, 0.1, 0.000]) # Near the table
    waypoints_pos.append([-7.0, -1.0, 0.000])
    waypoints_pos.append([-4.4, -1.0, 0.000])
    waypoints_pos.append([-4.4, 2.0, 0.000])
    waypoints_orient = list()
    #waypoints_orient.append([0.000, 0.000, 0.707, 0.707])
    #waypoints_orient.append([0.000, 0.000, 0.707, 0.707])
    #waypoints_orient.append([0.000, 0.000, -0.707, -0.707])
    #waypoints_orient.append([0.000, 0.000, -0.707, -0.707])
    #waypoints_orient.append([0.000, 0.000, -0.707, -0.707])
    waypoints_orient.append([0.000, 0.000, 0.707, 0.707])
    waypoints_orient.append([0.000, 0.000, -0.707, 0.707])
    waypoints_orient.append([0.000, 0.000, 0.707, 0.707])
    waypoints_orient.append([0.000, 0.000, 0.707, 0.707])
    waypoints_orient.append([0.000, 0.000, -0.707, 0.707])
    waypoints_orient.append([0.000, 0.000, 0.707, 0.707])
    waypoints_orient.append([0.000, 0.000, 0.707, 0.707])

    # Cycle through your waypoints and send the goal to the action server.
    for point in range(0,len(waypoints_pos)):
        sequence.set_goal(position = waypoints_pos[point], orientation =
        waypoints_orient[point])
        print('Moving to Point No.',point,waypoints_pos[point])
        result = sequence.execute()
        # <CHECK_FOR_SUCCESS>
        rospy.loginfo("Goal execution done!")
    # Terminate after loop
=======
>>>>>>> james-muir/milestone-3
    rospy.signal_shutdown("Goal sequence completed!")