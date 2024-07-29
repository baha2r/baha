# ZeroG Packages shared:

1. `zerog_description`
2. `zerog_gazebo`
3. `zerog_moveit_config`
4. `zerog_moveit_servo`
5. `zerog_robot_calibrations`
6. `zerog_utilities`

**Pre-requisites:**
1. Functional understanding/experience working with the ROS Ecosystem, such as:
 - How to create a workspace
 - How to build a ROS package
 - How to create Publishers/Subscribers, understand /tf, ROS params,rosservice etc.
 - How to debug using rqt_graph 

2. Understanding of robot singularities in particular the UR robot singularities.

**Dependencies**

**Note:**
Please refer to the `ReadMe.md` in each package to get started.

# General Overview:
## zerog_description:

This package contains the `URDF`,`XACROS` and `.stl` files required to populate the `robot_description` parameter on the ROS Parameter server which helps to create ZeroG Lab's robot visualizations in RViz.

The Readme in this package contains instructions on how to add additional links and joints etc (for the gripper,ft sensors etc)

## zerog_gazebo:
This package consists of the launch file required to spawn the robots in Gazebo and loading the robot controllers.
The Readme in this package contains instruction on how to load a simulation, test the robot motion using the robot trajectory controllers and how to use the robots with Moveit 

## zerog_moveit_config:
This package contains the config and launch files necessary for motion planning of the robot. This package is handy for having pre-defined joint configurations and for collision avoidance. It is also necessary to launch this package before starting any pose-tracking applications such as servoing.

## zerog_moveit_servoing:
This is probably the most important package for you. It contains the configuration and launch files necessary for enabling Catesian motion based Pose-tracking/Servoing of the robots. The waypoints that your algorithm will generate will need to be publihed to nodes launched from within this package. 

## zerog_robot-calibrations:
Contains configuration files for the robot. No need to touch them. But they're necessary for launching the robot_description package correctly.

## zerog_utilites: 
This package contains useful pieces of code,ROS nodes to facilitate robot control, obtaining position, accessing transformation between frames and velocity information etc. 

# Demo Example: Instructions for Launching For Pose-Tracking

Here I provide a basic demo for pose-tracking. The ceiling robot follows a set of given waypoints. Feel free to modify it to your use-case.

1. Launch the gazebo simulation. This will launch the ZeroG Lab Simulation
```
roslaunch zerog_gazebo zerog_gazebo.launch 
```

2. Launch Moveit motion-planning.
```
roslaunch zerog_moveit_config full_zerog_gazebo_moveit_planning_execution.launch 
```

**Note: Wait for the Motion Planning to be able to start. In case of warnings, press `Reset` at the bottom left corner in Rviz. Then choose `start_config` as the `Goal State` and click `Plan and Execute`. The robot should move to the initial configuration**

3. Define a fixed frame with respect to which way-points may be generated:
```
roslaunch zerog_utilities stp.launch x:=2.237 y:=-2.034 z:=1.522 qx:=-0.002 qy:=-0.706 qz:=0.004 qw:=0.708 frame_id:=world child_frame_id:=CW namespace:=world_CW


```
3. Launch the node that looks up the tf tree and transforms the waypoints to robot control frame:
```
roslaunch zerog_utilities reference_transformer_target.launch namespace_reference_transformer:=ceiling_arm_tool0_target_waypoint parent_frame_rt:=ceiling_arm_tool0 child_frame_rt:=target_waypoint
```

4. Launch Cartesian Motion Control:

```
roslaunch zerog_moveit_servo simple_posetracking_chaser_and_target_ee_based_simulated.launch
```

5. Launch sample trajectory(for ceiling robot), you'll have to press `Enter` to start the trajectory:

```
roslaunch zerog_utilities create_waypoints.launch namespace_moving_target_frame:=ceiling parent_frame_mtf:=CW child_frame_mtf:=target_waypoint traj_duration:=30.0 
```
