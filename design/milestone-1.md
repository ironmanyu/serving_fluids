# Milestone 1 Design

## TODO
### General
- [x] add stow behavior
- [x] add place behavior
- [x] add pick behavior
- [x] add actual furniture to GIX world
    - [x] change GIX world to include built in furniture
    - [x] spawn cup in milestone-1.launch
### Strech Goals
- [ ] create a node to publish the position of the can as a transform
- [ ] attach can to arm in MoveIt so the motion planning doesn't crash the can into anything
- [ ] add octomap support so the robot can dynamically plan around obstructions

## Pick
The robot will pick up a red coke can.
### Approach
- [ ] The robot finds the coke can in the camera image using OpenCV
- [ ] The robot points the camera towards the coke can to get a better image
- [ ] The robot scans the environment and adds it to the planning scene as an obstacle, except the can
- [ ] The robot reaches out to pick up the can, with the arm facing forwards
- [ ] The robot moves its hand around the can
- [ ] The robot grabs the can
- [ ] The robot lifts the can off of the table
- [ ] The robot moves the can and arm into a configuration for moving around

## Stow
The robot will move the can to a good position for moving.
### Approach
- [ ] The robot scans the environment and adds it to the planning scene as an obstacle
- [ ] The robot moves the can to the stowed position

## Place
The robot will place the coke can on the table
### Approach
- [ ] The robot scans the environment and adds it to the planning scene as an obstacle
- [ ] The robot identifies the surface of the table
- [ ] The robot moves the can to a point just above the table
- [ ] The robot lowers the can onto the table
- [ ] The robot releases the can
- [ ] The robot moves its gripper away from the can
- [ ] The robot scans the environment and adds it to the planning scene as an obstacle, including the can
- [ ] The robot tucks its arm