# This document outlines testing various IK Solvers for Cartesian Control.

## Goal:

The idea is to test how FDCC based Cartesian Control compares with the traditional IK solver methods for ground validation of on-orbital scenarios.

## Materials and Methods:

- Coppelia Sim
- Gazebo Environment
- Cartesian Controllers repo.

## Tests to be performed on a 6 Dof System (Due to certainity of Singular configurations)

### Drive Robot in to Singularity using Jinv

Changes needed to be made in: 
- zerog_system.xacro (choose default config)
- zerog_moveit_servo.srdf (specify start and end-config for ease of motion plannning)
- zerog_moveit_servoing yaml (to make sure the simulation do not enforce singularity avoidance)
    - robot_link_command_frame: ceiling_arm_base_link
    - lower_singularity_threshold:  17  
    - hard_stop_singularity_threshold: 30 
    - joint_limit_margin: 0.1 

- zerog_moveit_servoing node (try controling using base-link control, for consistency with FDCC: zerog_pose_tracking_simple_base_link_based.cpp)
- p5_stp.launch 


#### Steps to use with Moveit Servo or Jacobian Inverse Control:

1. Launch simulation

roslaunch zerog_bringup zerog_gazebo_bringup.launch 

2. Launch motion planning

roslaunch zerog_moveit_config full_zerog_gazebo_moveit_planning_execution.launch 

3. Launch STP

roslaunch zerog_utilities p5_stp.launch

4. Make the actual position wrt CW available as pose:

roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=CW_tool0 parent_frame:=CW child_frame:=ceiling_arm_tool0

5. Launch way-points generating node

rosrun zerog_utilities generate_trajectory 

6. Launch node that "is ready to" receive waypoints 

roslaunch zerog_utilities obtain_waypts_transform.launch namespace_transformer:=CW_target_ref parent_frame_rt:=CW child_frame_rt:=target_ref topic_to_subscribe_to:=/Way_pts_target_pose

7. Launch the node that "will transform the waypoints" into the control frame: tool0 when received

roslaunch zerog_utilities reference_transformer.launch namespace_reference_transformer:=ceiling_arm_tool0_target_ref parent_frame_rt:=ceiling_arm_tool0 child_frame_rt:=target_ref

8. Make the actual position available as pose:
roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=cb_ct parent_frame:=ceiling_arm_base_link child_frame:=ceiling_arm_tool0

9. Make the actual position available as pose:
roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=d_cb_ct parent_frame:=ceiling_arm_base_link child_frame:=target_ref

10. Change the controller for servoing

rosservice call /controller_manager/switch_controller "start_controllers:
- 'ceiling_arm_joint_group_pos_controller'
stop_controllers:
- 'ceiling_arm_pos_joint_traj_controller'
strictness: 2"


11. Launch pose-tracking node

roslaunch zerog_moveit_servo p5_launch_jacobian_inverse_pose_tracking.launch

12. Launch video recorder


13. Launch data recording

roslaunch zerog_utilities record_data.launch topics_name:="/d_cb_ct/pose /cb_ct/pose /joint_states"


14. call service:

rosservice call /trajectory_generator_node/start_trajectory "{}" 

15. Kill servo

16. Change back to orginal controller:

rosservice call /controller_manager/switch_controller "start_controllers:
- 'ceiling_arm_pos_joint_traj_controller'
stop_controllers:
- 'ceiling_arm_joint_group_pos_controller'
strictness: 2"

17. Plan and execute to intial position


**Notes:**
Two primary tests are performed with the following parameters for J^-1:
 - lower_singularity_threshold:  17000  
    - hard_stop_singularity_threshold: 30000
    - joint_limit_margin: 0.1

The trajectory generation node, is such that, it allows for a trajectory in two parts:

traj1: moving, traj2: fixed

Two experiments were run:
First:
Traj1: 10 sec  robot moved and then 
Traj2: 30 sec  robot stopped at this position

Second:

Traj1: 180 sec  robot moved and then 
Traj2: 30 sec  robot stopped at this position

The goal of having traj 2 is simply to have sufficient time to kill servo node and rosbag, while we have normal servo behaviour
and not the weird one, when waypoints end.

The only thing changing was:

waypoint_stamped.pose.position.z = initial_pose_.pose.position.z + 0.55*std::abs(1.0 * sin(0.2 *speed_factor* elapsed_time)); // Adjust as needed; 

in the generate trajectory.cpp


--------------------------------------------------------------------------------

### Drive Robot in to Singularity using FDCC

We use the same xacros and trajectory generator, however,
we may not be able to use moveit with it
- p5_stp.launch 


#### Steps to use with FDCC:

1. Launch simulation

roslaunch zerog_interaction_control p5_interaction_gazebo.launch

2. Launch STP

roslaunch zerog_utilities p5_stp.launch

3. Make the actual position wrt CW available as pose:

roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=CW_tool0 parent_frame:=CW child_frame:=ceiling_arm_tool0

4. Launch way-points generating node

rosrun zerog_utilities generate_trajectory 

5. Launch node that "is ready to" receive waypoints 

roslaunch zerog_utilities obtain_waypts_transform.launch namespace_transformer:=CW_target_ref parent_frame_rt:=CW child_frame_rt:=target_ref topic_to_subscribe_to:=/Way_pts_target_pose

6. Launch the node that "will transform the waypoints" into the control frame: base_link when received

roslaunch zerog_utilities reference_transformer.launch namespace_reference_transformer:=ceiling_arm_base_link_target_ref parent_frame_rt:=ceiling_arm_base_link child_frame_rt:=target_ref topic_to_publish_to:=/ceiling_arm_target_frame

7. Make the actual position available as pose:
roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=cb_ct parent_frame:=ceiling_arm_base_link child_frame:=ceiling_arm_tool0

8. Make the actual position available as pose:
roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=d_cb_ct parent_frame:=ceiling_arm_base_link child_frame:=target_ref


9. Launch video recorder

10. Launch data recording

roslaunch zerog_utilities record_data.launch topics_name:="/d_cb_ct/pose /cb_ct/pose /joint_states"

11. call service:

rosservice call /trajectory_generator_node/start_trajectory "{}" 

12. Kill ROSBAG

13. Kill ALL nodes

14. Restart


