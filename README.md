# CSC591-Project: Integrated Navigation and Control for Differential-Drive Robots using A* Path Planning and LQR Control

## Video
[![Project Video](https://img.youtube.com/vi/pPGtuJ7y9dQ/0.jpg)](https://www.youtube.com/watch?v=pPGtuJ7y9dQ)

## Details
- This project intends to implement Simultaneous Localization and Mapping, along with A* path planning algorithm and a Lienar Quadratic Optimal Controller(LQR) to control the robot path.
- It uses the slam-toolbox package, nav2 package for slam and navigation and eigen3-cmake package for the math calculation for the LQR process.
- There are a total of 5 packages for this project:
  - __ros_gz_wheelchair_bringup__: contains all the configuration and launch files.
  - __ros_gz_wheelchair_description__: contains the robot description file in SDF.
  - __ros_gz_wheelchair_gazebo__: contains the world description.
  - __ros_gz_wheelchair_application__: no significance for now.
  - __control__: contains the LQR controller launch file.
- As descibed in the __Usage__ below, each new goal will require a full restart of the system as the navigation and controller aren't fully integrated yet.
- __Unusual behavior__: If the robot ever stops and the controller specifies Goal reached, please stop the controller by __Ctrl + C__ and rerun it, the robot should start moving towards the target.I am not sure why this error occurs, but I'm guessing it is due to sync issues between odom and local_path topics.

## Installation
1) Clone this repository in a folder.
2) Create a workspace folder and a __"src"__ source folder inside it, and copy all the content from the cloned repo into the source folder.
3) Move to the workspace folder in terminal/cmd and source ROS (For linux: __"source /opt/ros/iron/setup.bash"__).
4) Run the command: __"colcon build"__ to build all the packages.
5) Being in the workspace, source the project using __"source ./install/setup.bash"__.
6) Open another terminal and source it as well.

## Usage
1) Launch the simulation, SLAM toolbox, and Nav2 packages using __"ros2 launch ros_gz_wheelchair_bringup wheelchair.launch.py use_sim_time:=true"__.
2) Once Gazebo comes up, start the simulation time using the Play button on the bottom left of the application.
3) Once all the nodes are running (nothing is printing in the terminal), set the initial pose and the goal pose using RViz's tools at the top of RViz screen. This should simulate the path to the goal position.
4) Open the other terminal and run the controller using __"ros2 run control lqr"__. This should be run as soon as possible after setting the goal pose.
5) Each new goal needs a complete rerun and relaunch of all system, rerun steps 1-4.

## Known behaviors
1) If the controller isn't run soon after the goal pose is set and the path is created, the Nav2 node might shutdown. This requires to set the goal pose again.
2) If the controller shuts down in the middle of the operation, rerunning the controller node will solve the issue. This is due to the fact that the controller isn't written as per the Nav2 controller paradigm, and is a future work item.
3) If nothing seems to be working as expected, please rerun everything and make sure the controller node is run after the goal pose is set as soon as possible.

## Future Work
1) Integrate the controller and Nav2 using the programming provide by Nav2 package.
2) Solve the sync issue in odom and local_plan topics, which can sometimes cause a very large error in the state_error variable in the LQR controller node.


