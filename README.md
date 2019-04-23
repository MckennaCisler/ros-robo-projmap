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
- For demos
  - [OpenCV](https://opencv.org/)

# Install
1) Clone this repo into your ROS package `src` directory and then run `catkin_make` 
2) Install the required C++ Python library by running the following in this directory:
``` 
cd include/ros_robo_projmap/gl_projector/
./compile_extension.sh
```

# Usage