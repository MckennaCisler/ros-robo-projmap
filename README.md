# ros-robo-projmap
A ROS package enabling AR without headsets: tool to represent a robot's internal state via live projection mapping. A project for Brown University's Collaborative Robotics class.

# Dependencies
- [ROS](https://wiki.ros.org/ROS/Installation)
- [IAI Kinect2](https://github.com/code-iai/iai_kinect2)
- For projection node
  - OpenGL
  - [libglfw3-dev](https://www.glfw.org/)
  - [libglew-dev](http://glew.sourceforge.net/)
  - [libglm-dev](https://glm.g-truc.net/0.9.9/index.html)

# Install
Clone this repo into your ROS package `src` directory and then run `catkin_make` 

# Usage