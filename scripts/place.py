# place.py
# author: James Muir - jdmuir@uw.edu
# place.py makes the robot place the object on a table
# assumptions: the robot is holding an object, the object should be held upright

# rospy for the subscriber
import rospy
# ROS messages
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point

# fetch_api
import fetch_api

import math

# Fetch + MoveIt!
# see https://docs.fetchrobotics.com/manipulation
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import copy

from pick import go_to_pose

def main(x=0.80, y=0.20, z=0.85):
    # TODO: The robot scans the environment and adds it to the planning scene as an obstacle

    # create new placing pose
    target_frame = "base_link"
    position = Point(x, y, z)
    # pointing right (along negative y axis), gripper horizontal
    orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    pose = Pose(position, orientation)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = target_frame
    pose_stamped.pose = pose
    gripper_frame = 'gripper_link'
    move_group = MoveGroupInterface("arm", target_frame)
    # set up the shutdown callback, so incomplete goals get cancelled on shutdown
    rospy.on_shutdown(move_group.get_move_action().cancel_all_goals)
    
    # move the arm to the place position
    print("moving the arm to the place position")
    go_to_pose(pose_stamped, move_group, gripper_frame)

    # drop the can
    gripper = fetch_api.Gripper()
    print("dropping the can")
    gripper.open()

    # lift the arm
    lift_pose = copy.deepcopy(pose_stamped)
    lift_pose.pose.position.z += 0.10 # up 10 cm
    print("lifting arm")
    go_to_pose(lift_pose, move_group, gripper_frame)


if __name__ == '__main__':
    # start the place node
    rospy.init_node('place')
    main()
    rospy.signal_shutdown('Done!')