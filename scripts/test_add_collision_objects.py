#!/usr/bin/env python

import rospy

from moveit_python import PlanningSceneInterface

from milestone_1 import clear_planning_scene

# start the node
rospy.init_node("add_planning_objects")

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

rospy.signal_shutdown('Done!')