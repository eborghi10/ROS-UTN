# My ROS robot

# Installation

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/eborghi10/rosabridge.git
$ git clone https://github.com/eborghi10/AS5048A.git
$ cd ..
$ catkin_make --only-pkg-with-deps rosabridge AS5048A
$ cp -r src/rosabridge/rosabridge_arduino/sketchbook/libraries/ros_lib/rosabridge_msgs ~/Arduino/libraries/ros_lib/
```

# Test

`roscore`

`rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0`

`roslaunch rosabridge_server odom_proxy_node.launch`