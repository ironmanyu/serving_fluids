# This repo is for the project 'Serving fluids'
This project is a demo of the Fetch Mobile Manipulator acting as a waiter by serving beverages.

## TODO

- [x] update package.xml
- [ ] add tuck behavior
- [ ] add place behavior
- [ ] add pick behavior
- [ ] add actual furniture to GIX world
- [ ] update CMakeLists.txt

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

## Running Instructions
1. Navigate to your Catkin workspace directory using `cd`
2. Run `source devel/setup.bash`
3. Run `roscd Serving_fluids`
4. Run `roslaunch launch/milestone_1.launch`

## Installation Instructions
1. Install Unbuntu 18.04: https://releases.ubuntu.com/18.04.5/?_ga=2.106856195.1054129880.1620949916-1656213416.1620949916
2. Install ROS Melodic: http://wiki.ros.org/melodic/Installation/Ubuntu
3. Install Fetch Gazebo simulation packages: https://docs.fetchrobotics.com/gazebo.html
4. Set up a Catkin workspace: https://docs.fetchrobotics.com/gazebo.html
5. Clone this repository in to the /src directory in the Catkin workspace
6. Build the Catkin workspace: run `catkin_make' in your Catkin workspace directory
7. 

## Working in this repo (pretty meta)

This is a Git repository (repo). Git is a distributed version control system. Basically, Git is a tool to keep track of different versions of our code.
* [Learn how to use Git](https://guides.github.com/introduction/git-handbook/)
* [Git command cheatsheet](https://training.github.com/downloads/github-git-cheat-sheet/)

This Git repo is hosted on GitHub. GitHub is a website for hosting Git repos.
* [Learn how to use GitHub](https://guides.github.com/introduction/git-handbook/)

In this repository, we follow the GitHub flow workflow. GitHub flow is development workflow that involves creating branches for new features. The golden rule of the workflow is **NEVER BREAK MAIN**. Never breaking main means that the code in the main branch should always work. In order to avoid breaking main, we create new branches when we want to add new features. We do all of the development work in the new branch. Once we have finished the new feature and made sure it works, we can merge our changes to the main branch.
* [Learn more about GitHub flow](https://guides.github.com/introduction/flow/)

## Editing this README (very meta)
This README file is written in [GitHub Flavored Markdown](https://github.github.com/gfm/). You can tell that it is a markdown file by the .md file extension at the end of the filename README.md. Markdown is simple markup format for formatting documents, written in plaintext and meant to be human readable in raw form. GitHub Flavored Markdown is the Markdown dialect used on GitHub.com.
* [Learn how to write GitHub Flavored Markdown](https://guides.github.com/features/mastering-markdown/)

