# The zerog_moveit_servo package

# Description:
This package contains an example on how to use the servoing feature from the `moveit_servo` package for the ZeroG Lab robots.
The example has been implemented and tested on the wall mounted UR10e robot. 

## Prerequisite:

- Familiarization with the `zerog_description`, `zerog_gazebo` and `zerog_moveit_config` packages.
- Basic but clear understanding of `tf/tf2`.
- It is recommended to familiarize oneself with [Moveit Servoing](https://ros-planning.github.io/moveit_tutorials/doc/realtime_servo/realtime_servo_tutorial.html) before using this package.



## What is possible with this package?

In this package there are two primary examples:

1. Sending twist commands to the robot (For developing Teleoperation applications)
2. Pose tracking

For now, the examples have been adapted from the `moveit_servo` package.

## How to use:

### Assumption:
You already know how to turn on the robots and operate them using basic ros-control (packages mentioned in the prerequisites). If you don't know how to do that, you are obligated to ask for help. 
**Warning:**
Before attempting to implement the steps. Read carefully at least once, to get an idea of what you are about to do.

### In Gazebo Simulation: For sending Twist commands

1. Type the following in the terminal and hit enter:

```
$ roslaunch zerog_bringup zerog_gazebo_bringup.launch
```
2. Then type:

```
$ roslaunch zerog_moveit_config full_zerog_gazebo_moveit_planning_execution.launch 
```

3. In `rviz` move the wall_arm to the config `nsw` using basic `plan and execute`.

4. using `rqt_controller_manager`  unload the `wall_arm_pos_traj_controller` and instead load and start `wall_arm_joint_group_position_controller`

5. Then run the following in the terminal:

```
$ roslaunch zerog_moveit_servo spacenav_cpp_simulated.launch 
```

6. Open `rqt_ez_publisher`. Add `/servo_server/delta_twist_cmds` topic. Choose one axis, e.g z-axis and move the slider **slowly**, to publish to the topic. Verify whether the robot is moving in along the correct axis.


### In Gazebo Simulation: For Pose Tracking

1. Type the following in the terminal and hit enter:

```
$ roslaunch zerog_bringup zerog_gazebo_bringup.launch
```
2. Then type:

```
$ roslaunch zerog_moveit_config full_zerog_gazebo_moveit_planning_execution.launch 
```
3. An Rviz window will open. Click file --> Open Config.
4. Navigate to ~/home/(your_path_to_catkin_ws)/src/zerog/zerog_moveit_config/rviz and select "zerog_gazebo_moveit_rviz.rviz". Click on "Open".
5. A saved rviz config will be loaded. It contains the Motion planning panel and TF displays.
6. In the Motion Planning Panel, click on the `Planning Tab`.
7. Plan and Execute the motion for the wall arm with  `w_servo_test_config` as the goal.
8. Plan and execute the motion for ceiling track with `c_track_test_config` as the goal.
9. Plan and execute the motion for ceiling arm `c_servo_test_config` as the goal.
10. Using `rqt_controller_manager` change unload the `wall_arm_pos_traj_controller` and instead load and start`wall_arm_joint_group_position_controller`
11. Open `rqt_joint_trajectory_controller`. Select `ceiling_track_position_trajectory_controller` and click the green button to turn it on.
12. Open `wall_arm_simulated_config.yaml` file in the config zerog_moveit_servo/config folder and make sure that you have the following parameters set as below:

```
robot_link_command_frame: wall_arm_tool0 
ee_frame_name: wall_arm_tool0
```

We will have the setup ready for pose-tracking.

13. Now launch the following launch file:
```
$  roslaunch zerog_moveit_servo simple_posetracking_ee_based_simulated.launch
```
This launch file launches a set of nodes, at different starting times, to ensure proper flow and exchange of information between each node):
|Order of Launch| ROS Nodes Names| C++ filename  | Package Name |Function description |
|--| ----------- |-----------------|---------------|-------------------|
|1| reference_target_pose|N/A| tf2_ros     | Creates a static transform (reference frame) in rviz wrt which a target frame can move|
|2| moving_target_frame_node| moving_target_frame.cpp | zerog_moveit_servo | Creates a moving target frame wrt reference_target_pose i.e. a static transform |
|3| target_frame_acquisition_node |reference_transformer.cpp | zerog_moveit_servo | Obtains the pose of the moving target frame wrt robot tool link or (base link, depending on in which frame do we want to give desired pose-commands) and publishes to pose tracking node|
|4| servo_server | zerog_pose_tracking_simple_ee_based  | zerog_moveit_servo |Receives the desired position set-points and performs the pose-tracking calculations (command frame is robot ee) |
|4| servo_server | zerog_pose_tracking_simple_base_link_based  | zerog_moveit_servo |Receives the desired position set-points and performs the pose-tracking calculations (command frame is robot's base link) |
|5| tf_info_node | tf_info_node.cpp | zerog_moveit_servo | Publishes any requested transformation over a rostopic so that it can be plotted or recorded using rosbag |
|6| plot_juggler | N/A | plotjuggler | For plotting all topics | 

**Note:**
Each of the node listed in the table can also be launched separately. For each node, there is a launch file, that can be launched individually in the same order as `Order of launch` specified in the table. The static transform is defined wrt to the ceiling tool frame but almost close to the wall tool frame. For this purpose, we've chosen low PID gains, and selected really low tolerance values so that, pose-tracking algorithm keeps running. Once the file has been launched, we can get the ceiling track to move.

14. Now with `rqt_joint_trajectory_controller` and `ceiling_track_position_trajectory_controller` selected in step 11, move the slider slowly and see the wall robot follow the ceiling rail's motion. 



### In Gazebo Simulation: For Pose Tracking with an external trajectory
- wrote moving_target_frame_with_subscriber: it tracks vivek's trajectory
problems: moving frame doesnt appear until we publish to the concerned topic. 
          Once the trajectory is over, the robot continues to move until hitting a singularity/emergency stop


### With the Real Robots: For sending Twist commands

**Warning:**
Using these steps with the real robots is very dangerous and utmost caution must be extered at all times. It is highly recommended to have atleast one person present to assist with pressing the emergency button should the need arise.

1. Type the following in the terminal and hit enter:

```
$ roslaunch zerog_bringup full_zerog_bringup.launch
```
2. Then type:

```
$ roslaunch zerog_moveit_config full_zerog_moveit_planning_execution.launch 
```

3. In `rviz` select the wall_arm config to `servo_config` and then click `Plan`. Observe the 'ghost' of the robot assume the desired configuration.
If the plan worked fine. Hit the `Execute` button for the robot to move into that position.

**Warning: DO NOT press `Plan and Execute` together. It might work no doubt, but it is safe to first visualize before executing**

4. using `rqt_controller_manager`  unload the `/wall_arm/pos_joint_traj_controller` and instead load and start `/wall_arm/joint_group_pos_controller`

5. Then run the following in the terminal:

```
$ roslaunch zerog_moveit_servo spacenav_cpp.launch 
```
6. Open `rqt_ez_publisher`. Add `/servo_server/delta_twist_cmds` topic. Choose one axis, e.g z-axis and move the slider **slowly** to a small value like 0.04 to 0.06, to publish to the topic. **Verify** whether the robot is moving in along the correct axis.
Since it is a velocity value, the robot will continue to move in that particular axis. If you publish a negative value, like -0.04, the robot should move in the direction opposite to what it was before. Publishing 0.0 value should stop the robot. 

7. Once done kill the launch initiated in step 5 to stop servoing node from running.


### With the Real Robots: For Pose-Tracking

**Warning:**
Using these steps with the real robots is very dangerous and utmost caution must be extered at all times. It is highly recommended to have atleast one person present to assist with pressing the emergency button should the need arise.

1. Type the following in the terminal and hit enter:

```
$ roslaunch zerog_bringup full_zerog_bringup.launch
```
2. Then type:

```
$ roslaunch zerog_moveit_config full_zerog_moveit_planning_execution.launch 
```
3. An Rviz window will open. **Check if the Robot Model and Motion Planning are already loaded. If so you have to skip 4,5 and 6. If it isn't then** Click file --> Open Config.
4. Navigate to ~/home/(your_path_to_catkin_ws)/src/zerog/zerog_moveit_config/rviz and select "zerog_gazebo_moveit_rviz.rviz". Click on "Open".
5. A saved rviz config will be loaded. It contains the Motion planning panel and TF displays.
6. In the Motion Planning Panel, click on the `Planning Tab`.
7. Plan and Execute the motion for the wall arm with  `w_servo_test_config` as the goal.
8. Plan and execute the motion for ceiling track with `c_track_test_config` as the goal.
9. Plan and execute the motion for ceiling arm `c_servo_test_config` as the goal.
10. Using `rqt_controller_manager` change unload the `/wall_arm/scaled_pos_joint_traj_controller` and instead load and start`/wall_arm/joint_group_pos_controller`
11. Open `rqt_joint_trajectory_controller`. Select `/ceiling_track/slider_position_trajectory_controller` and click the green button to turn it on.
12. Open `wall_arm_real_config.yaml` file in the config zerog_moveit_servo/config folder and make sure that you have the following parameters set as below:

```
robot_link_command_frame: wall_arm_tool0 
ee_frame_name: wall_arm_tool0
```

We will have the setup ready for pose-tracking.

13. Now launch the following launch file:
```
$  roslaunch zerog_moveit_servo simple_posetracking_ee_based.launch
```
This launch file launches a set of nodes, at different starting times, to ensure proper flow and exchange of information between each node:
|Order of Launch| ROS Nodes Names| C++ filename  | Package Name |Function description |
|--| ----------- |-----------------|---------------|-------------------|
|1| reference_target_pose|N/A| tf2_ros     | Creates a static transform (reference frame) in rviz wrt which a target frame can move|
|2| moving_target_frame_node| moving_target_frame.cpp | zerog_moveit_servo | Creates a moving target frame wrt reference_target_pose i.e. a static transform |
|3| target_frame_acquisition_node |reference_transformer.cpp | zerog_moveit_servo | Obtains the pose of the moving target frame wrt robot tool link or (base link, depending on in which frame do we want to give desired pose-commands) and publishes to pose tracking node|
|4| servo_server | zerog_pose_tracking_simple_ee_based  | zerog_moveit_servo |Receives the desired position set-points and performs the pose-tracking calculations (command frame is robot ee) |
|4| servo_server | zerog_pose_tracking_simple_base_link_based  | zerog_moveit_servo |Receives the desired position set-points and performs the pose-tracking calculations (command frame is robot's base link) |
|5| tf_info_node | tf_info_node.cpp | zerog_moveit_servo | Publishes any requested transformation over a rostopic so that it can be plotted or recorded using rosbag |
|6| plot_juggler | N/A | plotjuggler | For plotting all topics | 

**Note:**
Each of the node listed in the table can also be launched separately. For each node, there is a launch file, that can be launched individually in the same order as `Order of launch` specified in the table. The static transform is defined wrt to the ceiling tool frame but almost close to the wall tool frame. For this purpose, we've chosen low PID gains, and selected really low tolerance values so that, pose-tracking algorithm keeps running. Once the file has been launched, we can get the ceiling track to move.

14. Now with `rqt_joint_trajectory_controller` and `/ceiling_track/slider_position_trajectory_controller` selected in step 11, move the slider slowly and see the wall robot follow the ceiling rail's motion. 

**Note:**
When adding frames to robot, servoing will be successful only after, additional frames/changes are specified in:

- zerog_system xacro
- zerog_moveit_servo.srdf
- wall_arm_real_config.yaml (servoing config file)
- pose tracking code
- also on the fly specifying the new command frame in reference transformer node