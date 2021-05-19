# stow.py
# author: James Muir - jdmuir@uw.edu
# hold.py makes the robot move the object its grasping to a good position for moving around
# assumptions: the robot is holding an object, the object should be held upright

# rospy for the subscriber
import rospy

# fetch_api
import fetch_api

import math

def main():
    # TODO: The robot scans the environment and adds it to the planning scene as an obstacle, except the can
    
    # move the arm to the stowed position
    print("stowing the can")
    arm = fetch_api.Arm()
    theta = 0.15
    joints_list = [1.32, 1.4, -0.2, 1.72 + theta, -math.pi/2, 1.66, -math.pi/2 + theta]
    js = fetch_api.ArmJoints.from_list(joints_list)
    a = zip(js.names(), js.values())
    arm.move_to_joint_goal(a)
    # arm.move_to_joints(js)

if __name__ == '__main__':
    # start the stow node
    rospy.init_node('stow')
    main()
    rospy.signal_shutdown('Done!')