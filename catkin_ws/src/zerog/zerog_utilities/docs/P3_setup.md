# Steps for 6 Dof tf based

roslaunch zerog_bringup full_zerog_bringup.launch

rosservice call /ceiling_track/start_rail "{}" 

roslaunch zerog_utilities stp.launch 

roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=CW_target parent_frame:=CW child_frame:=target

**publishes the waypoints as tf after subscribing to them**
rosrun zerog_utilities obtain_waypts_transform_target CW target_ref

roslaunch zerog_utilities reference_transformer_target.launch namespace_reference_transformer:=ceiling_arm_tool0_target_ref parent_frame_rt:=ceiling_arm_tool0 child_frame_rt:=target_ref


roslaunch zerog_moveit_config full_zerog_moveit_planning_execution.launch

roslaunch zerog_utilities launch_visualizer.launch 

----------------------------------------------------------roslaunch zerog_utilities launch_manipulability_metrics_six_dof.launch
roslaunch zerog_utilities jacobian_test.launch 


rqt  change controller

record screen

roslaunch zerog_moveit_servo p3_exp_pose_tracking.launch

vivek--> launch

roslaunch zerog_utilities record_data.launch

kill rosbag

kill servo


# Steps for 7 dof tf based

roslaunch zerog_bringup full_zerog_bringup.launch

rosservice call /ceiling_track/start_rail "{}" 

roslaunch zerog_combined_hardware_interface zerog_combined_robot_hardware.launch

**Check file before launching 7dof**
roslaunch zerog_utilities stp.launch 

roslaunch zerog_utilities tf_info_node.launch plot_tf_namespace:=CW_target parent_frame:=CW child_frame:=target

rosrun zerog_utilities obtain_waypts_transform_target CW target_ref


roslaunch zerog_utilities reference_transformer_target.launch namespace_reference_transformer:=c_a_tool0_target_ref parent_frame_rt:=c_a_tool0 child_frame_rt:=target_ref

roslaunch zerog_full_moveit_config full_gazebo_moveit_planning_execution.launch

roslaunch zerog_utilities launch_visualizer.launch full_condition:=true

----------------------------------------------------------------------roslaunch zerog_utilities launch_full_state_manipulability_metrics.launch

roslaunch zerog_utilities 7_dof_jacobian_test.launch 

record screen

roslaunch zerog_full_moveit_servo simple_posetracking_ee_based_real.launch

vivek--> launch

**Check file before launching 7dof**

roslaunch zerog_utilities record_data.launch

kill rosbag

kill servo