# FISS Planner
## [RA-L 2022] FISS: A Trajectory Planning Framework using Fast Iterative Search and Sampling Strategy for Autonomous Driving

[![CodeFactor](https://www.codefactor.io/repository/github/ss47816/lgsvl_utils/badge)](https://www.codefactor.io/repository/github/ss47816/fiss_planner)
![Code Grade](https://api.codiga.io/project/31518/status/svg)
![Code Quality Score](https://api.codiga.io/project/31518/score/svg)
![GitHub Repo stars](https://img.shields.io/github/stars/ss47816/fiss_planner?color=FFE333)
![GitHub Repo forks](https://img.shields.io/github/forks/ss47816/fiss_planner?color=FFE333)

![Ubuntu](https://img.shields.io/badge/OS-Ubuntu-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)
![ROS](https://img.shields.io/badge/Tools-ROS-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)
![C++](https://img.shields.io/badge/Code-C++-informational?style=flat&logo=c%2B%2B&logoColor=white&color=2bbc8a)

This is an improved framework based on the framework used in [`frenet_optimal_planner`](https://github.com/SS47816/frenet_optimal_planner). We utilize historical planning results as prior information in heuristics and introduce an iterative search-generate-evaluate strategy to look for the optimal trajectory candidate. Compared to the current frameworks, our method can significantly reduce the number of computationally expensive operations spent during the planning and achieve 2-6 times faster. 

_Our work has been accepted by RA-L 2022 and IROS 2022._

## Demo 
![cover_image](media/demo_1.gif)

## Performance (Planning Frequency in Hz)

| # of Trajectory Samples | FOP  | FOP* | FOP Improved | FOP Improved* | FISS  |   FISS*   |
| :---------------------: | :--: | :--: | :----------: | :-----------: | :---: | :-------: |
|           100           | 9.2  | 33.0 |     84.2     |     94.2      | 221.1 | **241.1** |
|           250           | 3.2  | 15.0 |     42.8     |     51.5      | 199.9 | **207.0** |
|           500           | 1.5  | 7.7  |     24.6     |     32.5      | 137.6 | **146.9** |
|          1000           | 0.8  | 3.8  |     16.2     |     18.1      | 97.6  | **98.3**  |

All Results above are obtained on an Intel i7-8700K CPU with 6 cores @3.70 GHz and 16 GB RAM

\* are using Asynchronous collision checking module (with `use_async = True` in settings)

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

### Update July 2022
The latest instructions to run our code will be coming soon, since the LGSVL simulator is deprecated recently

## Contribution
You are welcome contributing to the package by opening a pull-request

We are following: 
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), 
[C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#main), 
and [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)


## License
Our [`fiss_planner`](https://github.com/SS47816/fiss_planner) ROS package is licensed under [Apache License 2.0](https://github.com/SS47816/fiss_planner/blob/main/LICENSE)

The included Eigen Library follows its own [Mozilla Public License v. 2.0](http://mozilla.org/MPL/2.0/)
