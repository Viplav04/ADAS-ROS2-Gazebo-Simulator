# ADAS_ROS2-OpenCV-Lane-Detection-Auto-Braking-with-LIDAR-Object-Detection-Gazebo-Teleop-Keyboard
ADAS Application with ROS2 using Camera and LIDAR using Gazebo Simulator.

This is an open source project using License Apache 2.0 to understand simple ADAS applications using ROS2 Crystal and Gazebo Simulator.
This setup requires:
  1. ROS2 crystal
  2. Gazebo 9
  3. OpenCV

You can drive around the robot in the simulator and have Lane Assist and Auto Brake when object is detected.

Steps to run the demo:

Clone the repo
Paste the model folders : my_robot, hokuyo and depth_camera in .gazebo/models folder. Be careful, these folder have been modified so change the names before pasting as they may over write original folders in models directory.


In 1st terminal

Source ROS2 :

source /opt/ros/crystal/setup.bash

Source gazebo:

source /usr/share/gazebo/setup.sh

To run:

gazebo --verbose myworld.world

In 2nd terminal, source ROS2 crystal and
To build Lane_detection_node :

cd ~/Lane_detection_node

colcon build

source insall/local_setup.bash

To run : 

ros2 run lane_assist lane_node


In another terminal, source ROS2 crystal:
To build laser_obstacle_avoidance :

cd ~/laser_obstacle_avoidance

colcon build

To run : 

ros2 run laser_obstacle_avoidance obstacle_node


In another terminal : 

source ROS2 crystal

ros2 run teleop_twist_keyboard teleop_twist_keyboard


Play around!!!

Please open a issue or write to me for issues. A SLAM node and navigation package can be added easily.

