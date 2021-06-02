#!/usr/bin/env python
import argparse

import rospy

from moveit_python import PlanningSceneInterface, MoveGroupInterface

import math

import fetch_api

# my nodes
import pick, stow, place
from navigate_irl import NavGoalClient
from milestone_1 import clear_planning_scene

import time

from std_srvs.srv import Empty

# the node code
if __name__ == '__main__':
    # parse command line arguments
    print('parsing command line arguments')
    parser = argparse.ArgumentParser(description="navigate to a series of poses")
    parser.add_argument('-s', '--simulation', action='store_true') # default False
    args, unknown = parser.parse_known_args()

    # start the node
    print('starting the node')
    rospy.init_node('demo_irl')

    # initialize the fetch_api components
    arm = fetch_api.Arm()
    base = fetch_api.Base()
    gripper = fetch_api.Gripper()
    head = fetch_api.Head()
    torso = fetch_api.Torso()

    # TODO: place can in hand
    gripper.open()
    raw_input("Place the can in the robot's gripper. KEEP FINGERS AWAY FROM GRIPPER! Press the ENTER key to close the gripper.")
    gripper.close()

    # wait for user to start
    raw_input("Press the ENTER key to start the robot.")

    # clear the costmaps
    # print("clearing the costmaps")
    # rospy.wait_for_service('move_base/clear_costmaps')
    # clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
    # clear_costmaps()

    # TODO: navigate to the table
    print('navigating to the dining table')
    nav_client = NavGoalClient()
    # table_x = -4.5
    # table_y = 0.5
    # table_theta = math.pi/2
    table_x = -2.949
    table_y = 0.645
    table_theta = 0.374

    nav_client.send_goal(table_x, table_y, table_theta)
    time.sleep(2)

    # drive towards the table
    drive_forward_distance = 1.1
    base.go_forward(drive_forward_distance)
    # TODO: place the can
    # set up system to cancel MoveIt actions if the program is terminated early
    print("setting up move group interface")
    move_group = MoveGroupInterface("arm", 'base_link')
    move_group.setPlanningTime(30)
    # set up the shutdown callback, so incomplete goals get cancelled on shutdown
    rospy.on_shutdown(move_group.get_move_action().cancel_all_goals)

    # set up the collision objects for MoveIt
    print("setting up collision objects")

    # planning scene relative to robot
    planning_scene = PlanningSceneInterface('base_link')

    # get rid of old objects
    clear_planning_scene(planning_scene)
   # add table
    table_height = 0.85
    # planning_scene.addBox("table", 0.8, 2.0, table_height, 2.4 - table_y - drive_forward_distance, 0.115788, table_height/2.0)

    # add ground
    # centered on robot base_link
    # all collision objects are stored relative to robot so they follow the robot around by default
    ground_thickness = .1
    planning_scene.addBox("ground", 3, 3, ground_thickness, 0, 0, -ground_thickness/2)

    # scan environment with octomap
    # wait for octomap to catch up
    print("waiting for octomap to refresh")
    rospy.wait_for_service('/clear_octomap') #this will stop your code until the clear octomap service starts running
    clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
    clear_octomap()
    # swing the head around to build the octomap
    pan_angles = (head.MAX_PAN, math.pi/4, 0, -math.pi/4, head.MIN_PAN)
    tilt_angles = (0, math.pi/4)
    for pa in pan_angles:
        for ta in tilt_angles:
            head.pan_tilt(pa, ta)
            time.sleep(1)
    # move the head to the right angle
    print("moving the head")
    head.pan_tilt(0, 0.3)
    
    # run the place routine
    print("placing")
    place.main(x=0.85, y=0.0, z=0.85)

    print("tucking the arm")
    arm.tuck()

    # finish up
    clear_planning_scene(planning_scene)
    rospy.signal_shutdown('Demo Complete!')