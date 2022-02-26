# FISS Planner: A General Trajectory Planning Framework using Fast Iterative Search and Sampling Strategy for Autonomous Driving

![image](https://github.com/SS47816/fiss_planner/blob/main/media/demo.png "Demo")

## Dependencies
* C++11 above
* CMake: 3.0.2 above
* ROS Packages:
  * roscpp
  * rospy
  * tf
  * tf2_ros
  * std_msgs
  * nav_msgs
  * geometry_msgs
  * autoware_msgs
  * message_generation
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
# clone the repo
roslaunch fiss_planner fiss_planner.launch
```

## Contribution
You are welcome contributing to the package by opening a pull-request

We are following: 
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), 
[C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#main), 
and [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)


## License
[Apache 2 License](https://github.com/SS47816/fiss_planner/blob/main/LICENSE)