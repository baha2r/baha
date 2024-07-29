# The zerog_description package

# Description

This package contains the meshes, URDF, config and launch files necessary to load the `/robot_description` parameter into the ROS parameter server.
The important contents of this package are described as under:

|**Folder**|**Description**|
|:------|:------|
|meshes|It contains the necessary cad models that are used in the URDF and Xacro files|
|urdf|this folder contains all the urdf, macros and xacros for each component of the zerog lab i.e. UR10e|
|tests|contains a .xml file to test whether the `/robot_description` is loaded and whether the robot model can be viewed in Rviz|
|launch| contains the launch files necessary to load the `/robot_description` and hence the ZeroG visualization in RViz|
|config|Contains the necessary yaml files to load the necessary kinematics, joint_limits, visual and physical parameters into the ROS parameter server|
|rviz| contains some saved Rviz configurations|


# The URDF architecture

The `urdf` folder has the following important components:

## **inc:**
This folder consists of the xacro files for:
1. The ceiling track and the wall track in their in the `ceiling` and `wall` folders
2. A `common` folder which contains xacro files defining certain parameters in gazebo simulations, transmission tags for robot and rail urdfs(that specify the kind of hardware interface being used for the robots etc)
3. `zerog_system.xacro`: In this xacro file, all the macros for robots and rails are called. If there is a need to attach an end-effector or an additional link/joint. **That has to be specified in this file**

## **zerog.xacro:**
This is the top level xacro file that calls the `zerog_system.xacro`. This is the xacro that is loaded into the `/robot_description` parameter.

# How to use this package:

## Addition of extras: links/joints/end-effectors 
As of now the following usage is foreseen:
1. Addition of extra dummy links and joints. This addition must be done in `zerog_system.xacro`.
Here is an example for adding extra link and a revolute joint to the ceiling robot:
```xml
<link name="target_sat"/>
<joint name="target_sat_to_ceiling_arm_tool0" type="fixed">
    <parent link="ceiling_arm_tool0"/>
    <child link="target_sat"/>
    <origin xyz="-0.015 -0.017 0.174" rpy="-1.697 -0.014 0.152 "/>
</joint>
```
2. Addition of extra end-effectors/grippers/mockups including the mesh files for visualization or collision avoidance purposes. This addition must be done in `zerog_system.xacro`.
Here is an example for adding a cubesat mockup to the end of the ceiling robot. Take note that you need to include the CAD model (.STL, .dae, etc) into the `meshes` folder of this package and mention the path as shown in the example:
```xml
<link name="cubesat">
		<visual>
			<geometry>
				<mesh filename="package://zerog_description/meshes/misc/cubesat.STL" scale="0.0015 0.0015 0.0015"/>
			</geometry>
			<material name="cubesat" >
				<color rgba="0 0 1.0 1.0"/>
      </material>
		</visual>
		<collision>
			<geometry>
				<mesh filename="package://zerog_description/meshes/misc/cubesat.STL" scale="0.0015 0.0015 0.0015"/>
			</geometry>
		</collision>
</link>

<joint name="ceiling_arm_tool0-cubesat" type="fixed">
    <parent link="ceiling_arm_tool0" />
    <child link="cubesat"/>
    <origin xyz="-0.035 -0.035 0.03"/>
</joint>

```
In the example, the `cubesat.STL` was placed in a folder called `misc` that was added to the `meshes` folder and the path to the file is specified in
```xml
<mesh filename="package://zerog_description/meshes/misc/cubesat.STL" scale="0.0015 0.0015 0.0015"/>
```
## Loading and Checking the whether the /robot_description has been loaded:

Just to check/learn/debug without using the real or simulated robots, one can follow these instructions.
### Loading the xacro
1. One can load the `zerog.xacro` into the ROS parameter server as follows:
```terminal
$ roslaunch zerog_description load_zerog_description.launch
```
2. In the terminal, type the following and hit 'enter':
```terminal
$ rosparam list
```
You will see that `/robot_description` parameter has been loaded (for example):
```terminal
/robot_description
/rosdistro
/roslaunch/uris/host_localhost__36071
/rosversion
/run_id
```

### Checking that we can visualize the robot model
1. In the terminal, run:
```terminal
$ roslaunch zerog_description view_zerog.launch
```
It will launch Rviz and you will be able to visualize the ZeroG facility.



### See also:
- zerog_gazebo package



