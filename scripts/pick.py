#!/usr/bin/env python

# pick.py
# author: James Muir - jdmuir@uw.edu
# pick.py makes the robot pick up a can
# assumptions: the can is a red coke can, the can is vertical,
# the robot can see the can, the robot can reach the can


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

import math

# Fetch + MoveIt!
# see https://docs.fetchrobotics.com/manipulation
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import copy



# Instantiate CvBridge
bridge = CvBridge()

def segment_image(img, simulation=False):
    # find red
    # see https://stackoverflow.com/questions/30331944/finding-red-color-in-image-using-python-opencv

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # HSV range: [0-179, 0-255, 0-255]
    if simulation:
        min_sat = 51
        max_sat = 255
        min_val = 51
        max_val = 255
    else:
        min_sat = int(math.floor(50/100.0 * 255))
        max_sat = 255
        min_val = int(math.floor(50/100.0 * 255))
        max_val = 255

    lower_red_1 = np.array([0, min_sat, min_val])
    upper_red_1 = np.array([9, max_sat, max_val])
    mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    # cv2.imshow("Mask 1", mask_1)

    lower_red_2 = np.array([167, min_sat, min_val])
    upper_red_2 = np.array([179, max_sat, max_val])
    mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    # cv2.imshow("Mask 2", mask_2)

    mask = cv2.bitwise_or(mask_1, mask_2)
    # cv2.imshow("Mask", mask)

    obj_seg = cv2.bitwise_and(img,img, mask=mask)

    # Find the moments of the image
    M = cv2.moments(mask)
    print M

    if M['m00'] != 0:
        cX = int(M['m10'] / M['m00'])
        cY = int(M['m01'] / M['m00'])
    else:
        cX, cY = 0, 0

    if not simulation:
        cv2.circle(obj_seg, (cX, cY), 5, (0, 0, 255), -1)
        cv2.imshow("Result",obj_seg)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return cX, cY, M['m00']

def get_XYZ(msg_pcl,cX,cY):
    array_position = cY*msg_pcl.row_step + cX*msg_pcl.point_step
    camPoint = Point()
    (camPoint.x, camPoint.y, camPoint.z) = struct.unpack_from('fff', msg_pcl.data, offset=array_position)

    return camPoint

def go_to_pose(pose, move_group, gripper_frame):
    pose.header.stamp = rospy.Time.now()

    move_group.moveToPose(pose, gripper_frame)

    result = move_group.get_move_action().get_result()

    if result:
        # Checking the MoveItErrorCode
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Reached Goal")
        else:
            # If you get to this point please search for:
            # moveit_msgs/MoveItErrorCodes.msg
            error_code = move_group.get_move_action().get_state()
            rospy.logerr("Arm goal in state: %s", error_code)
            raise RuntimeError("Move It Error:", error_code)
    else:
        rospy.logerr("MoveIt! failure no result returned.")

    return result

def main(simulation=False):
    # make sure gripper is open for grasping
    gripper = fetch_api.Gripper()
    gripper.open()

    # The robot finds the coke can in the camera image using OpenCV

    # Define your image topics
    color_topic = "/head_camera/rgb/image_raw"
    pcl_topic = "/head_camera/depth_registered/points"
    # Set up your color subscriber and define its callback
    #rospy.Subscriber(color_topic, Image, color_image_callback)
    # Set up your depth subscriber and define its callback
    #rospy.Subscriber(pcl_topic, PointCloud2, ptcloud_callback)

    for i in range(3):
        msg = rospy.wait_for_message(color_topic,Image)
    
    print("Received a color image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_color_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        if not simulation:
            cv2.imshow("Image",cv2_color_img)
            cv2.waitKey(0) 
            #closing all open windows 
            cv2.destroyAllWindows()
        pass

    cX,cY,area = segment_image(cv2_color_img, simulation=simulation)

    print "Centroid, x: ", cX, "y: ", cY
    print "Area: ", area
    
    # convert from 2D image coordinates to 3D spatial coordinates
    # using the camera depth cloud and transforms

    msg_pcl = rospy.wait_for_message(pcl_topic,PointCloud2)
    print("Received pointcloud")

    camP = get_XYZ(msg_pcl,cX,cY)

    print camP

    # define source and target frame
    source_frame = 'head_camera_rgb_optical_frame'
    target_frame = 'base_link'

    transformation = get_transformation(source_frame, target_frame)
    point_wrt_target = transform_point(transformation, camP)

    print point_wrt_target

    #Options: 
    # 1) publish point_wrt_target in a Publisher for another node to move the arm
    # 2) import fetch_api and use arm class to provide Pose stamped as goal to the gripper
    # 3) draw from helper function from first course to move fetch to a given pose

    # TODO: The robot points the camera towards the coke can to get a better image

    # TODO: The robot scans the environment and adds it to the planning scene as an obstacle, except the can
    # can use octomap for this, but I believe it involves editing the pointcloud before sending it to octomap
    # since the map is static, just using fixed collision objects in areas where we are using the arm is probably fine

    # planning scene relative to robot
    planning_scene_base_link = PlanningSceneInterface("base_link")

    # add can to planning scene as collision object
    # the can coordinates are relative to the base link
    # the dimensions are those of a standard coke can
    print("adding coke can as collision object")
    can_height = 12.2/100 # m
    can_diameter = 3.25/100 #m
    planning_scene_base_link.addCylinder("can", can_height, can_diameter, point_wrt_target[0] + can_diameter/2, point_wrt_target[1], point_wrt_target[2])


    # turn the can position into a gripping pose
    position = Point(point_wrt_target[0], point_wrt_target[1], point_wrt_target[2])
    orientation = Quaternion(0.0, 0.0, 0.0, 1.0) # straight forward, gripper horizontal
    pose = Pose(position, orientation)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = target_frame
    pose_stamped.pose = pose
    gripper_frame = 'gripper_link'
    move_group = MoveGroupInterface("arm", target_frame)
    # set up the shutdown callback, so incomplete goals get cancelled on shutdown
    rospy.on_shutdown(move_group.get_move_action().cancel_all_goals)
    
    # The robot reaches out to pick up the can, with the arm facing forwards
    in_front = copy.deepcopy(pose_stamped)
    in_front.pose.position.x += -0.05 # backward 5 cm
    print("reaching in front of can")
    go_to_pose(in_front, move_group, gripper_frame)

    # TODO: attach the can to the arm in the planning scene

    # The robot moves its hand around the can
    print("removing can as collision object")
    planning_scene_base_link.removeCollisionObject("can")
    at_can = copy.deepcopy(in_front)
    at_can.pose.position.x += 0.06 # forward 6 cm
    print("moving gripper around can")
    go_to_pose(at_can, move_group, gripper_frame)

    # The robot grabs the can
    print("closing the gripper")
    gripper.close()

    # The robot lifts the can off of the table
    above_table = copy.deepcopy(at_can)
    above_table.pose.position.z += 0.05 # 5 cm
    print("lifting the can")
    go_to_pose(above_table, move_group, gripper_frame)
    

if __name__ == '__main__':
    # start the pick node
    rospy.init_node('pick')
    main()
    rospy.signal_shutdown('Done!')