# Milestone 1 Design

## TODO
### General
- [ ] add items to milestone_2.launch
    - [x] navigation stack
    - [x] milestone_2.rviz
    - [ ] milestone_2.py
- [ ] finish milestone_2.py
    - [ ] add proper waypoints
    - [x] automatically send 2D pose estimate at the start
- [ ] get through doors and hallways
    - [ ] tune costmap parameters so robot can get through doors
    - [ ] add seperate behavior to get through doors and hallways
- [x] add finite state machine diagram
- [x] add sequence diagram
- [x] add evaluation metrics
### Stretch Goals
- [ ] add vision system to get through doors
- [ ] add keepout map so the robot doesn't try to drive through furniture

## Notes
How to create a map: `roslaunch fetch_navigation build_map.launch`
How to save a map: https://wiki.ros.org/map_server#map_saver
