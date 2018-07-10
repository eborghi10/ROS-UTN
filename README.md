# ROS UTN

**Robot Multiplo N6 con ROS | UTN FRBA**

### Instalación

```sh
$ cd /catkin_ws/src
$ git clone git@github.com:eborghi10/ROS-UTN.git
$ cd ..
$ rosdep install --from-path src/ -y -i
$ catkin_make
```

### Ejecución

Para correr una demo:

1. `roslaunch robot_control arduino_robot.launch`
