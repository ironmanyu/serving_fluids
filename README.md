# This repo is for the project 'Serving fluids'
This project is a demo of the Fetch Mobile Manipulator acting as a waiter by serving beverages.

## Installation Instructions
1. Install Unbuntu 18.04: https://releases.ubuntu.com/18.04.5/?_ga=2.106856195.1054129880.1620949916-1656213416.1620949916
2. Install ROS Melodic: http://wiki.ros.org/melodic/Installation/Ubuntu
3. Install Fetch Gazebo simulation package: https://docs.fetchrobotics.com/gazebo.html
4. Set up a Catkin workspace: https://docs.fetchrobotics.com/gazebo.html
5. Clone this repository in to the /src directory in the Catkin workspace: `git clone git@github.com:ironmanyu/serving_fluids.git`
6. Install the fetch_api:
    1. Clone the fetch_api repository into the /src directory in the Catkin workspace: `git clone git@github.com:jamesdarrenmuir/fetch_api.git`
    2. Install pip: `sudo apt install python-pip`
    3. Install IPython and Traitlets: `pip install IPython traitlets`
7. Build the Catkin workspace: run `catkin_make` in your Catkin workspace directory

## Launch Instructions
### General
Before starting any of the demos, do the following:
1. Navigate to your Catkin workspace directory using `cd`
2. Run `source devel/setup.bash`
### Milestone 1
1. Run `roslaunch serving_fluids milestone_1.launch`
### Milestone 2
Note: Milestone 2 in simulation is a WIP
TODO: update this with final instructions for milestone 2
1. Run `roslaunch serving_fluids milestone_2.launch`
### Milestone 3
Note: Milestone 3 in simulation is a WIP
TODO: update this with final instructions for milestone 3
1. Run `roslaunch serving_fluids milestone_3.launch`
### Final Demo IRL
1. Set up the real robot
2. Run `roslaunch serving_fluids demo_irl.launch`
3. Set the robot's position using "2D Pose Estimate" in RViz
4. Run `rosrun serving_fluids demo_irl.py`

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

