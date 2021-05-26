#!/usr/bin/env python

# import sys, getopt # for command line arguments
import argparse

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import math
import tf
from tf.transformations import quaternion_from_euler
import numpy as np
import fetch_api

def drive_straight_through_narrow_space(distance):
    base = fetch_api.Base()
    tf_listener = tf.TransformListener()
    rate = rospy.Rate(10) # 10 Hz
    start_trans = None
    # drive forward until we have travelled distance
    while not rospy.is_shutdown():
        # stop if too close to a wall
        laser_scan = rospy.wait_for_message("base_scan", LaserScan)
        if (np.min(laser_scan.ranges) < 0.07): # if less than 7 cm away from object
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
        base.move(0.1, 0.0)

class GoalSequence():
    def __init__(self):
        # Get an action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # Add a goal to the class
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = 0.0
        self.goal.target_pose.pose.position.y = 0.0
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = 0.0
        self.goal.target_pose.pose.orientation.y = 0.0
        self.goal.target_pose.pose.orientation.z = 0.0
        self.goal.target_pose.pose.orientation.w = 1.0

    def set_goal(self, position, orientation, frame='map'):
        goal = self.goal
        goal.target_pose.header.frame_id = frame
        # set position
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        # set orientation
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]

    def execute(self):
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        # Returns the result of this goal
        result = self.client.get_result()
        return result

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="set 2D Pose Estimate and navigate to some goal poses")
    parser.add_argument('-x', default=0.0, type=float)
    parser.add_argument('-y', default=0.0, type=float)
    parser.add_argument('-Y', default=0.0, type=float)
    args = parser.parse_args()

    rospy.init_node('GoalSequence')

    # set the 2D Pose Estimate
    pose_est_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    while pose_est_pub.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscriber to connect")
        rospy.sleep(1)
    pose_estimate = PoseWithCovarianceStamped()

    pose_estimate.header.frame_id = "map"
    pose_estimate.header.stamp = rospy.Time.now()
    # position
    pose_estimate.pose.pose.position.x = args.x
    pose_estimate.pose.pose.position.y = args.y
    # orientation
    q = quaternion_from_euler(0.0, 0.0, args.Y)
    [x, y, z, w] = q
    # print(q)
    pose_estimate.pose.pose.orientation.x = x
    pose_estimate.pose.pose.orientation.y = y
    pose_estimate.pose.pose.orientation.z = z
    pose_estimate.pose.pose.orientation.w = w
    print(pose_estimate)
    pose_est_pub.publish(pose_estimate)

    # drive forward 3 meters down the hallway
    drive_straight_through_narrow_space(3)

    # Initialize class
    sequence = GoalSequence()
    # Declare your relevant waypoints' positions and orientations as a list.
    # They should be the same LENGTH

    waypoints_pos = list()
    waypoints_pos.append([-4.1, 1.0, 0.000]) # Booth
    waypoints_pos.append([-1.88, 1.0, 0.000]) # Kitchen Doorway
    waypoints_orient = list()
    waypoints_orient.append([0.000, 0.000, 0.707, 0.707])
    waypoints_orient.append([0.000, 0.000, 0.707, 0.707])

    # Cycle through your waypoints and send the goal to the action server.
    for point in range(0,len(waypoints_pos)):
        sequence.set_goal(position = waypoints_pos[point], orientation =
        waypoints_orient[point])
        result = sequence.execute()
        # <CHECK_FOR_SUCCESS>
        rospy.loginfo("Goal execution done!")
    # Terminate after loop
    rospy.signal_shutdown("Goal sequence completed!")