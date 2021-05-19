# stow.py
# author: James Muir - jdmuir@uw.edu
# hold.py makes the robot move the object its grasping to a good position for moving around
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
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import copy

def go_to_pose(pose, move_group, gripper_frame):
    pose.header.stamp = rospy.Time.now()

    move_group.moveToPose(pose, gripper_frame)

    result = move_group.get_move_action().get_result()

    if result:
        # Checking the MoveItErrorCode
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Hello there!")
        else:
            # If you get to this point please search for:
            # moveit_msgs/MoveItErrorCodes.msg
            rospy.logerr("Arm goal in state: %s",
                            move_group.get_move_action().get_state())
    else:
        rospy.logerr("MoveIt! failure no result returned.")

    return result

def main():
    # TODO: The robot scans the environment and adds it to the planning scene as an obstacle, except the can

    # create new holding pose
    target_frame = "base_link"
    position = Point(0.173315, -0.171359, 0.444913)
    # pointing right (along negative y axis), gripper horizontal
    orientation = Quaternion(0.0, 0.0, -math.sqrt(2)/2, math.sqrt(2)/2)
    pose = Pose(position, orientation)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = target_frame
    pose_stamped.pose = pose
    gripper_frame = 'gripper_link'
    move_group = MoveGroupInterface("arm", target_frame)
    # set up the shutdown callback, so incomplete goals get cancelled on shutdown
    rospy.on_shutdown(move_group.get_move_action().cancel_all_goals)
    
    # move the arm to the stowed position
    print("stowing the can")
    go_to_pose(pose_stamped, move_group, gripper_frame)

if __name__ == '__main__':
    # start the stow node
    rospy.init_node('stow')
    main()
    rospy.signal_shutdown('Done!')