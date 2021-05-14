# hold.py
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

def clear_planning_scene(PSI):
    # clears all of the objects in a given Planning Scene Interface (PSI)
    old_collision_objects = PSI.getKnownCollisionObjects()
    for collision_object in old_collision_objects:
        PSI.removeCollisionObject(collision_object)

    old_attached_objects = PSI.getKnownAttachedObjects()
    for attached_object in old_attached_objects:
        PSI.removeAttachedObject(attached_object)

def main():
    # start the pick node
    rospy.init_node('pick')

    # TODO: The robot scans the environment and adds it to the planning scene as an obstacle, except the can

    # set up collsion objects in MoveIt

    # remove old then add new collsion objects

    # planning scene relative to robot
    planning_scene_base_link = PlanningSceneInterface("base_link")
    # get rid of old objects
    clear_planning_scene(planning_scene_base_link)

    # add ground
    # planning_scene.addCube("ground", 2, 1.1, 0.0, -1.0)
    ground_thickness = .1
    planning_scene_base_link.addBox("ground", 3, 3, ground_thickness, 0, 0, -ground_thickness/2)

    # TODO: make sure this actually places the table relative to the world
    # clear_planning_scene seems to clear all collision objects
    # regardless of which PlanningSceneObject you pass it
    # planning scene relative to the world
    planning_scene_odom = PlanningSceneInterface("odom")
    
    # add table
    table_height = 1.0
    planning_scene_odom.addBox("table", 0.8, 2.0, table_height, 0.9, 0.115788, table_height/2.0)

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
    go_to_pose(pose_stamped, move_group, gripper_frame)

    rospy.signal_shutdown('Done!')

if __name__ == '__main__':
    main()