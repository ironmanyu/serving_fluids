#!/usr/bin/env python

import rospy
import fetch_api
from fetch_api import Arm, Base, Gripper, Head, Torso, ArmJoints
import math

import ros

import threading

from moveit_python import PlanningSceneInterface, MoveGroupInterface

# my nodes
import pick, stow, place

def clear_planning_scene(PSI):
    # clears all of the objects in a given Planning Scene Interface (PSI)
    old_collision_objects = PSI.getKnownCollisionObjects()
    for collision_object in old_collision_objects:
        PSI.removeCollisionObject(collision_object)

    old_attached_objects = PSI.getKnownAttachedObjects()
    for attached_object in old_attached_objects:
        PSI.removeAttachedObject(attached_object)

# start the node
rospy.init_node("milestone_1")

# initialize the fetch_api components
arm = Arm()
base = Base()
gripper = Gripper()
head = Head()
torso = Torso()

# drive forward to get to table
# we have to spawn the robot at a distance from the table
# because its arm starts pointing straight ahead
print("driving to table")
start_distance = 1.1 # 1.1 m
base.go_forward(start_distance)

# raise the robot up
print("raising torso")
torso.set_height(torso.MAX_HEIGHT)

# point head at table
print("tilting head")
head.pan_tilt(0, math.pi/5)

# set up system to cancel MoveIt actions if the program is terminated early
move_group = MoveGroupInterface("arm", 'base_link')
# set up the shutdown callback, so incomplete goals get cancelled on shutdown
rospy.on_shutdown(move_group.get_move_action().cancel_all_goals)

# set up the collision objects for MoveIt

# planning scene relative to robot
planning_scene = PlanningSceneInterface('base_link')
# get rid of old objects
clear_planning_scene(planning_scene)

# add table
table_height = 0.75
planning_scene.addBox("table", 0.8, 2.0, table_height, 1.05, 0.115788, table_height/2.0)

# add ground
# centered on robot base_link
# all collision objects are stored relative to robot so they follow the robot around by default
ground_thickness = .1
planning_scene.addBox("ground", 3, 3, ground_thickness, 0, 0, -ground_thickness/2)

# run the pick routine
print("picking")
pick.main()

# run the stow routine
print("stowing")
stow.main()

# run the place routine
print("placing")
place.main()

# make sure the arm is tucked
arm.tuck()

# lower the robot
print("lowering torso")
torso.set_height(torso.MIN_HEIGHT)

# drive back to starting positoin
print("driving back to starting position")
base.go_forward(-start_distance)

rospy.signal_shutdown('Done!')