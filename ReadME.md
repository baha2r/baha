# Config

Xvfb :1 -screen 0 1024x768x16 &
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1

```
roslaunch zerog_gazebo zerog_headless.launch
```
```
roslaunch zerog_moveit_config full_zerog_gazebo_moveit_planning_execution_headless.launch
```
```
roslaunch zerog_moveit_servo simple_posetracking_chaser_and_target_ee_based_simulated.launch
```


## Default Param
default wall arm joints = [0.0  , 0.0  , 0.0, -1.57, 1.57, 0]

default ceiling arm joints = [-1.57, -1.57, 0.0, -1.57, -1.57, 1.57]

default wall track = 0

default cailing track = 0

## Changing param
```
rosrun zerog_moveit_servo joints_control_arms.py
```
ceiling track = 0.6


## Add capture_frame
```
rosrun zerog_moveit_servo create_capture_frame.py
```

# Run the ee pose publisher
```
rosrun zerog_moveit_servo ee_pose_publisher.py 
```
