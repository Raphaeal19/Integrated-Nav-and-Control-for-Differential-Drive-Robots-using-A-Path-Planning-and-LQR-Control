# CSC591-Project

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



