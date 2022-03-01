# FISS Planner: A General Trajectory Planning Framework using Fast Iterative Search and Sampling Strategy for Autonomous Driving

[![CodeFactor](https://www.codefactor.io/repository/github/ss47816/lgsvl_utils/badge)](https://www.codefactor.io/repository/github/ss47816/lgsvl_utils)
![Code Grade](https://api.codiga.io/project/30669/status/svg)
![Code Quality Score](https://api.codiga.io/project/30669/score/svg)
![GitHub Repo stars](https://img.shields.io/github/stars/ss47816/lgsvl_utils?color=FFE333)
![GitHub Repo forks](https://img.shields.io/github/forks/ss47816/lgsvl_utils?color=FFE333)

![Ubuntu](https://img.shields.io/badge/OS-Ubuntu-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)
![ROS](https://img.shields.io/badge/Tools-ROS-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)
![C++](https://img.shields.io/badge/Code-C++-informational?style=flat&logo=c%2B%2B&logoColor=white&color=2bbc8a)

## Demo 
![cover_image](media/demo.gif)
### Demo Video []()
## Dependencies
Our package is only based on standard ROS pkgs, with no other external dependencies:
* C++11 above
* CMake: 3.0.2 above
* Eigen (included)
* ROS Packages:
  * roscpp
  * rospy
  * tf
  * tf2_ros
  * std_msgs
  * nav_msgs
  * geometry_msgs
  * autoware_msgs
  * visualization_msgs
  * dynamic_reconfigure

## Installation
Clone the repo and install dependencies:
```bash
# clone the repo
git clone https://github.com/SS47816/fiss_planner.git
cd fiss_planner

# install dependencies
rosdep install --from-paths src --ignore-src -r -y

# build using `catkin_make` , or you can build with `catkin build`
catkin_make
# source 
source devel/setup.bash
```

## Usage

Launch the Planner node by running:
```bash
# Launch nodes
roslaunch fiss_planner fiss_planner.launch
```

## Contribution
You are welcome contributing to the package by opening a pull-request

We are following: 
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), 
[C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#main), 
and [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)


## License
Our [`fiss_planner`](https://github.com/SS47816/fiss_planner) ROS package is licensed under [Apache License 2.0](https://github.com/SS47816/fiss_planner/blob/main/LICENSE)

The included Eigen Library follows its own [Mozilla Public License v. 2.0](http://mozilla.org/MPL/2.0/)