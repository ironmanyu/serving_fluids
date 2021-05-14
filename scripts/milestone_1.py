import rospy
import fetch_api
from fetch_api import Arm, Base, Gripper, Head, Torso, ArmJoints

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

# TODO: run the pick routine

# TODO: run the place routine

# TODO: make sure the arm is tucked
# TODO: make sure that this is aware of obstacles
# arm = fetch_api.Arm()
# arm.tuck()

# lower the robot
torso.set_height(torso.MIN_HEIGHT)