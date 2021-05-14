# rospy for the subscriber
import rospy
# fetch_api
import fetch_api
import math

def main():
    # start the pick node
    rospy.init_node('quick_setup_milestone_1')
    torso = fetch_api.Torso()
    torso.set_height(torso.MAX_HEIGHT)

    head = fetch_api.Head()
    head.pan_tilt(0, math.pi/5)

    rospy.signal_shutdown('Done!')

if __name__ == '__main__':
    main()