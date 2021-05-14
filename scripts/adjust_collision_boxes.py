# rospy for the subscriber
import rospy
# ROS messages
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for processing and saving an image
import cv2
# Numpy to work with arrays
import numpy as np
import struct
# helper function for TF transformations
from tf_listener import get_transformation, transform_point

# fetch_api
import fetch_api

# Fetch + MoveIt!
# see https://docs.fetchrobotics.com/manipulation
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import copy

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
    # MoveIt!

    # so the robot doesn't smash the ground
    # remove old then add new collsion objects

    # planning scene relative to robot
    planning_scene = PlanningSceneInterface("base_link")
    # get rid of old objects
    clear_planning_scene(planning_scene)

    # add ground
    # planning_scene.addCube("ground", 2, 1.1, 0.0, -1.0)
    ground_thickness = .1
    # planning_scene.addBox("ground", 3, 3, ground_thickness, 0, 0, -ground_thickness/2)

    # planning scene relative to the world
    planning_scene_2 = PlanningSceneInterface("odom")
    # clear_planning_scene(planning_scene_2)
    
    # add table
    table_height = 1.0
    # planning_scene_2.addBox("table", 0.8, 2.0, table_height, 0.9, 0.115788, table_height/2.0)

    rospy.signal_shutdown('Done!')

if __name__ == '__main__':
    main()