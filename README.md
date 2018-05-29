# Blue Internal Software
This repository contains all of our internal software, which includes things like:
- Manufacturing/calibration scripts
- Demos (ie teleoperation)
- Other proof-of-concepts (ie Gazebo)

-----

## How do I run the teleop demo?

- Start the control stack, ie:
  ```bash
  # For a full robot (see README in blue_core)
  roslaunch blue_bringup full.launch
  ```
  ```bash
  # For a left arm
  roslaunch blue_bringup left.launch
  ```
  ```bash
  # For a right arm
  roslaunch blue_bringup right.launch
  ```
  ```bash
  # For the simulator (extremely experimental)
  roslaunch blue_gazebo blue_world.launch
  ``` 
- Start the teleop nodes:
  ```bash
  # For a full robot
  roslaunch blue_teleop teleop_full.launch
  ```
  ```bash
  # For a left arm
  roslaunch blue_teleop teleop_left.launch
  ```
  ```bash
  # For a right arm
  roslaunch blue_teleop teleop_right.launch
  ``` 
  This should start copies of the inverse kinematics node, rviz teleop node, and Vive teleop node.
- **(rviz teleop)** Draggable [interactive markers](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started) should now appear on the `/left_arm/cartesian_pose_teleop` and `/right_arm/cartesian_pose_teleop` namespaces in rviz. Moving these will publish 6DOF pose targets that our IK node will attempt to reach.
- **(Vive teleop)** Nothing more needs to be done on the ROS side to run the VR demo; it's already waiting for connections from the Unity code via [rosbridge](http://wiki.ros.org/rosbridge_suite). To run the Unity side:
  1. Open Unity on the VR computer
  2. Open the "mind_meld" project
  3. Hit the "Play" button at the top of the window
      - Make sure the controllers/headset are on and connected!
  4. This can be a little finicky -- double-click the "Play" button in Unity to restart everything if there are issues
