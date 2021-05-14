import rospy
import fetch_api
from fetch_api import Arm, Base, Gripper, Head, Torso, ArmJoints
import math

import ros

import threading

# my nodes
import pick, stow, place

# start the node
rospy.init_node("milestone_1")

# initialize the fetch_api components
arm = Arm()
base = Base()
gripper = Gripper()
head = Head()
torso = Torso()

# raise the robot up
torso.set_height(torso.MAX_HEIGHT)

# point head at table
head.pan_tilt(0, math.pi/5)

# TODO: run the pick routine
pick.main()

# TODO: run the stow routine
stow.main()

# TODO: run the place routine
place.main()

# TODO: make sure the arm is tucked
# TODO: make sure that this is aware of obstacles
# arm = fetch_api.Arm()
# arm.tuck()

# lower the robot
torso.set_height(torso.MIN_HEIGHT)

rospy.signal_shutdown('Done!')