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

    # TODO: navigate to the stool
    print('navigating to the dining table')
    nav_client = NavGoalClient()
    # table_x = -2.0
    # table_y = 1.1
    # table_theta = math.pi/2
    table_x = -4.339
    table_y = -0.470
    table_theta = -math.pi/2

    nav_client.send_goal(table_x, table_y, table_theta)

    # TODO: pick up and tuck the can
    def pick_and_place_configuration():
        # raise the robot up
        # print("raising torso")
        # torso.set_height(torso.MAX_HEIGHT)
        # point head at table
        print("tilting head")
        # head.pan_tilt(0, math.pi/5)
        head.pan_tilt(0.0, 0.6)
    pick_and_place_configuration()

    # set up system to cancel MoveIt actions if the program is terminated early
    move_group = MoveGroupInterface("arm", 'base_link')
    # set up the shutdown callback, so incomplete goals get cancelled on shutdown
    rospy.on_shutdown(move_group.get_move_action().cancel_all_goals)

    # set up the collision objects for MoveIt

    # planning scene relative to robot
    planning_scene = PlanningSceneInterface('base_link')
    # get rid of old objects
    clear_planning_scene(planning_scene)

    def set_up_planning_scene():
        # add table
        # table_height = 0.75
        table_height = 0.695 # 69.5 cm
        planning_scene.addBox("table", 0.8, 2.0, table_height, 1.2, 0.115788, table_height/2.0)

        # add ground
        # centered on robot base_link
        # all collision objects are stored relative to robot so they follow the robot around by default
        ground_thickness = .1
        planning_scene.addBox("ground", 3, 3, ground_thickness, 0, 0, -ground_thickness/2)
    set_up_planning_scene()

    # run the pick routine
    print("picking")
    pick.main(simulation=args.simulation)
    # TODO: tuck the can
    # run the stow routine
    print("stowing")
    arm.tuck()

    clear_planning_scene(planning_scene)

    # TODO: drive away and come back
    # lower the robot down
    # print("lowering torso")
    # torso.set_height(torso.MIN_HEIGHT)
    # print('leaving the table')
    # nav_client.send_goal(0.0, 0.0, 0.0)
    # print('coming back to the table')
    # nav_client.send_goal(table_x, table_y, table_theta)

    # TODO: go to new table
    print('going to second table')
    nav_client.send_goal(-1.196, table_y, -math.pi/2)

    # TODO: place the can
    pick_and_place_configuration()
    set_up_planning_scene()
    # run the place routine
    print("placing")
    place.main(x=1.25, y=0.0, z=0.75)

    arm.tuck()

    # finish up
    clear_planning_scene(planning_scene)
    rospy.signal_shutdown('Demo Complete!')