# ADAS_ROS2-OpenCV-Lane-Detection-Auto-Braking-with-LIDAR-Object-Detection-Gazebo-Teleop-Keyboard
ADAS Application with ROS2 using Camera and LIDAR using Gazebo Simulator.


This is an open source project using License Apache 2.0 to understand simple ADAS applications using ROS2 Crystal and Gazebo Simulator.You can drive around the robot in the simulator and have Lane Detection and Auto Brake when object is detected. It uses a Hough Transform algorithm which works only for straight lines.
This setup requires:
  1. ROS2 crystal. Install (https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Install-Debians/)
  2. Gazebo 9. Install (http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
  3. OpenCV & cv_bridge. Install (https://www.learnopencv.com/install-opencv3-on-ubuntu/)

![alt text](https://github.com/Viplav04/ADAS_ROS2-OpenCV-Lane-Detection-Auto-Braking-with-LIDAR-Object-Detection-Gazebo-Teleop-Keyboard/blob/master/RO2_ADAS.png)



Steps to run the demo:

1.Clone the repo:

```git clone https://github.com/Viplav04/ADAS_ROS2-OpenCV-Lane-Detection-Auto-Braking-with-LIDAR-Object-Detection-Gazebo-Teleop-Keyboard.git```

2.Paste the model folders : my_robot, hokuyo and depth_camera in .gazebo/models folder. 

Be careful, these folder have been modified so change the names before pasting as they may over write original folders in models directory.


3.In 1st terminal

Source ROS2 :

```source /opt/ros/crystal/setup.bash```

Source gazebo:

```source /usr/share/gazebo/setup.sh```

To run gazebo:

```gazebo --verbose myworld.world```

4.In 2nd terminal, source ROS2 crystal and
To build Lane_detection_node :

```
source /opt/ros/crystal/setup.bash
cd ~/Lane_detection_node
colcon build
source insall/local_setup.bash
```

To run lane detection node: 

```ros2 run lane_assist lane_node```


5.In another terminal, source ROS2 crystal:
To build laser_obstacle_avoidance :

```
source /opt/ros/crystal/setup.bash
cd ~/laser_obstacle_avoidance
colcon build
```

To run obstacle node: 

```ros2 run laser_obstacle_avoidance obstacle_node```


6.In another terminal : 

source ROS2 crystal

```
source /opt/ros/crystal/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


Play around!!!
![alt text1](https://github.com/Viplav04/ADAS_ROS2-OpenCV-Lane-Detection-Auto-Braking-with-LIDAR-Object-Detection-Gazebo-Teleop-Keyboard/blob/master/Screenshot%20from%202019-08-30%2014-12-47.png)
Please open a issue or write to me for issues. A SLAM node and navigation package can be added easily.

