# About this package

This package is used to store the robot calibrations (factory calibrations)
Factory calibrations are unique to each robot and doesn't seem to change on addition of an extra tool

# Usage

## Naming convention of the files:
The calibration file is a .yaml file and should be named according to the following convention:

robot_arm_tool_name_calibration_date.yaml

For example, when no end-effector is mounted, we have:

ceiling_arm_tool0_calibration_01012024.yaml


## How to obtain calibration file:

1. Launch the bringup file as usual

```
roslaunch zerog_bringup full_zerog_bringup.launch
```

2. Launch the following:

```
roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.88.* target_filename:="$(rospack find zerog_robot_calibrations)/config/robot_name_tool_name_calibration_date.yaml"
```

**Note: replace the * in the roslaunch command above with the appropriate number in each robot IP**

The calibration file will be saved in the zerog_robot_calibrations/config folder

### How to use the calibration file:

When Launching the bringup file you can use the following arguments:

`wall_arm_kinematics_config` and `ceiling_arm_kinematics_config`
They can be used as shown below:

```
roslaunch zerog_bringup full_zerog_bringup.launch wall_arm_kinematics_config:="$(rospack find zerog_robot_calibrations)/config/wall_arm_camera_tool_calibration.yaml"

```

