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



# Instantiate CvBridge
bridge = CvBridge()

def segment_image(img):
    # find red
    # see https://stackoverflow.com/questions/30331944/finding-red-color-in-image-using-python-opencv

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # HSV range: [0-179, 0-255, 0-255]
    lower_red_1 = np.array([0,51,51])
    upper_red_1 = np.array([9,255,255])
    mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    # cv2.imshow("Mask 1", mask_1)

    lower_red_2 = np.array([167,51,51])
    upper_red_2 = np.array([179,255,255])
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

    cv2.circle(obj_seg, (cX, cY), 5, (0, 0, 255), -1)

    # cv2.imshow("Result",obj_seg)
    # cv2.waitKey(0)
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
    
    # make sure arm is out of the way of the camera
    arm = fetch_api.Arm()
    arm.tuck()

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
    msg = rospy.wait_for_message(color_topic,Image)
    print("Received a color image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_color_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # cv2.imshow("Image",cv2_color_img)
        # cv2.waitKey(0) 

        #closing all open windows 
        cv2.destroyAllWindows()

    cX,cY,area = segment_image(cv2_color_img)

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

    # add can
    planning_scene_odom.addCylinder("can", 12.2/100, 3.25/100, point_wrt_target[0], point_wrt_target[1], point_wrt_target[2])


    # turn the can position into a gripping pose
    position = Point(point_wrt_target[0], point_wrt_target[1], point_wrt_target[2])
    orientation = Quaternion(0.0, 0.0, 0.0, 1.0) # straight forward, gripper horizontal
    pose = Pose(position, orientation)
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = target_frame
    pose_stamped.pose = pose
    gripper_frame = 'gripper_link'
    move_group = MoveGroupInterface("arm", "base_link")
    
    # TODO: The robot reaches out to pick up the can, with the arm facing forwards
    in_front = copy.deepcopy(pose_stamped)
    in_front.pose.position.x += -0.05 # backward 5 cm
    go_to_pose(in_front, move_group, gripper_frame)

    # TODO: The robot moves its hand around the can
    planning_scene_odom.removeCollisionObject("can")
    at_can = copy.deepcopy(in_front)
    at_can.pose.position.x += 0.05 # forward 3 cm
    go_to_pose(at_can, move_group, gripper_frame)

    # TODO: The robot grabs the can
    gripper.close()

    # TODO: The robot lifts the can off of the table
    above_table = copy.deepcopy(at_can)
    above_table.pose.position.z += 0.05 # 5 cm
    go_to_pose(above_table, move_group, gripper_frame)

    # TODO: The robot moves the can and arm into a configuration for moving around
    move_pose = copy.deepcopy(above_table)
    move_pose.pose.position.x += -0.10 # backwards 10 cm
    go_to_pose(move_pose, move_group, gripper_frame)

    # tuck the arm
    arm.tuck()

    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    move_group.get_move_action().cancel_all_goals()

    rospy.signal_shutdown('Done!')

if __name__ == '__main__':
    main()