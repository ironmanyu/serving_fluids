import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult

class GoalSequence():
    def __init__(self):
        # Get an action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # Add a goal to the class
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = 0.0
        self.goal.target_pose.pose.position.y = 0.0
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = 0.0
        self.goal.target_pose.pose.orientation.y = 0.0
        self.goal.target_pose.pose.orientation.z = 0.0
        self.goal.target_pose.pose.orientation.w = 1.0
    def set_goal(self, position, orientation, frame='map'):
        goal = self.goal
        goal.target_pose.header.frame_id = frame
        # set position
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = position[2]
        # set orientation
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]
    def execute(self):
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        # Returns the result of this goal
        result = self.client.get_result()
        return result
if __name__ == '__main__':
    rospy.init_node('GoalSequence')
    # Initialize class
    sequence = GoalSequence()
    # Declare your relevant waypoints' positions and orientations as a list.
    # They should be the same LENGTH

    waypoints_pos = list()
    waypoints_pos.append([2.473, 2.812, 0.000]) # Open Space Table
    waypoints_pos.append([-3.117, 3.656, 0.000]) # Inner Room Table
    waypoints_orient = list()
    waypoints_orient.append([0.000, 0.000, 0.000, 1.000])
    waypoints_orient.append([0.000, 0.000, 0.694, 0.720])

    # Cycle through your waypoints and send the goal to the action server.
    for point in range(0,len(waypoints_pos)):
        sequence.set_goal(position = waypoints_pos[point], orientation =
        waypoints_orient[point])
        result = sequence.execute()
        # <CHECK_FOR_SUCCESS>
        rospy.loginfo("Goal execution done!")
    # Terminate after loop
    rospy.signal_shutdown("Goal sequence completed!")