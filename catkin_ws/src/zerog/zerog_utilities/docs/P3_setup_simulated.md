# Steps for 6 Dof tf based

roslaunch zerog_bringup zerog_gazebo_bringup.launch

roslaunch zerog_moveit_config full_zerog_gazebo_moveit_planning_execution.launch

roslaunch zerog_utilities stp.launch 

roslaunch zerog_utilities launch_visualizer.launch 

roslaunch zerog_utilities jacobian_test.launch 

rqt  change controller

record screen

roslaunch zerog_moveit_servo p3_simulated_exp_pose_tracking.launch


**broadcast your own way pointsas tf**
roslaunch zerog_utilities create_waypoints.launch 

roslaunch zerog_utilities reference_transformer_target.launch namespace_reference_transformer:=ceiling_arm_tool0_target_ref parent_frame_rt:=ceiling_arm_tool0 child_frame_rt:=target_ref

roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=cab_ct parent_frame:=ceiling_arm_base_link child_frame:=ceiling_arm_tool0

roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=CW_tool0 parent_frame:=CW child_frame:=ceiling_arm_tool0

roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=CW_target_ref parent_frame:=CW child_frame:=target_ref

roslaunch zerog_utilities record_data.launch

start trajectory

kill rosbag

kill servo






# Steps for 7 dof tf based

roslaunch zerog_bringup zerog_gazebo_bringup.launch

roslaunch zerog_combined_hardware_interface zerog_gazebo_combined_robot_hardware.launch

roslaunch zerog_full_moveit_config full_gazebo_moveit_planning_execution.launch

**Check file before launching 7dof**
roslaunch zerog_utilities stp.launch 

roslaunch zerog_utilities launch_visualizer.launch full_condition:=true

roslaunch zerog_utilities 7_dof_jacobian_test.launch 

rqt  change controller

record screen

roslaunch zerog_full_moveit_servo simple_posetracking_ee_based_simulated.launch


**Check file before launching. broadcast your own way pointsas tf**
roslaunch zerog_utilities create_waypoints.launch 

roslaunch zerog_utilities reference_transformer_target.launch namespace_reference_transformer:=c_a_tool0_target_ref parent_frame_rt:=c_a_tool0 child_frame_rt:=target_ref

roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=cab_ct parent_frame:=c_a_base_link child_frame:=c_a_tool0

roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=CW_tool0 parent_frame:=CW child_frame:=c_a_tool0

roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=CW_target_ref parent_frame:=CW child_frame:=target_ref

roslaunch zerog_utilities record_data.launch

start trajectory

kill rosbag

kill servo

