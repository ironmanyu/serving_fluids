# TECHIN 517 Robotics Lab5 (Milestone1)


## Intro

In this lab, I drove the robot using teleop inforot of the can then let the robot grasp the can and then place it back on the table.

## Explanation

The following sections explain how the robot is to grasp and place the can.

## Procedure

- Move the robot using teleop in front of the can
- Run the pick_and_palce node
    - This node will call three nodes (grasp.py, place.py, stow.py)
    ## grasp.py
        - Take an image of the can
        - Threshold the can to find red color
        - Find centroid of the can
        - Get pose (x, y, z) of the can 
        - Set pose using go_to_pose() function
        - Open the gripper using Fetch API
        - Move the aram to reach the can
        - Set the gripper orientation to horizontal 
        - Move the gripper forward to the can
        - Close the gripper
        - And finally lift the can off.
    ## place.py
        - move the arm to the target pose
        - open the gripper
        - lift the arm
    ## stow.py
        - move the arm to the stowed position
## How To Run

Make sure the required Python packages are installed, then follow the launch instructions. Vizualization Options contains various ways of viewing the robot's operation other than just looking at the Gazebo window.
Required Python Packages

    numpy
    cv2
    imutils
    rospy
    fetch_api

## Launch Instructions

Run each command in a seperate terminal tab.

    Launch Gazebo: roslaunch $HOME/catkin_ws/src/Serving_fluids/Milestone1/milestone1.launch
    Run python script: python $HOME/catkin_ws/src/Serving_fluids/pick_and_place.py

## Vizualization Options

    Camera Image Viewer: rosrun rqt_image_view rqt_image_view /head_camera/rgb/image_raw
    Launch RViz: rosrun rviz rviz -d $HOME/catkin_ws/src/Serving_fluids/milestone1.rviz



