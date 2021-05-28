#!/usr/bin/env python

import argparse

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose

from tf.transformations import quaternion_from_euler


def set_pose_estimate(x, y, yaw):
    pose_est_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    while pose_est_pub.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscriber to connect")
        rospy.sleep(1)
    pose_estimate = PoseWithCovarianceStamped()
    pose_estimate.header.frame_id = "map"
    pose_estimate.header.stamp = rospy.Time.now()
    pose_estimate.pose.pose = pose_from_xyY(x, y, yaw)
    # print(pose_estimate)
    pose_est_pub.publish(pose_estimate)

def pose_from_xyY(x, y, Y):
    # returns a pose given x and y position coordinates (m) and a Y yaw value (rad)
    pose = Pose()
    # position
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    # orientation
    (x, y, z, w) = quaternion_from_euler(0.0, 0.0, Y)
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w
    return pose

if __name__ == '__main__':
    # parse command line arguments
    parser = argparse.ArgumentParser(description="set 2D Pose Estimate and navigate to some goal poses")
    # parser.add_argument('__name', default='set_pose_estimate')
    # parser.add_argument('__log', default=None)
    parser.add_argument('-x', default=0.0, type=float)
    parser.add_argument('-y', default=0.0, type=float)
    parser.add_argument('-Y', default=0.0, type=float)
    args, unknown = parser.parse_known_args()

    # start the node
    rospy.init_node('set_pose_estimate')

    # set the 2D Pose Estimate
    set_pose_estimate(args.x, args.y, args.Y)

    rospy.signal_shutdown("Goal sequence completed!")