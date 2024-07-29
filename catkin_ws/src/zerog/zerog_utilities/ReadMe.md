# The zerog_utlities package

This package consists of rosnodes and launch files needed for realizing certain common actions/primitives

## Pre-requites:
The user is expected to understand the following concepts in ROS:

1. tf, tf Broadcaster, tf Listener, tf_tree
2. Rosbags
3. ros topics: geometry_msgs/PoseStamped, geometry_msgs/WrenchStamped
4. How to use the ubuntu command line terminal to start ROS nodes and launch files.

## Key features offered by this package:

1. Creation of Static Transform Publishers
2. Data Recording
3. Position Transformation as a rostopic
4. Conversion from tf to a topic of type geometry_msgs/PoseStamped
5. Conversion from topic of type geometry_msgs/PoseStamped to tf frame

**Note to users**:

The examples shown here are default examples. Should you need to use any of the above features, you are instructed to create a separate launch file by copy the contents of the default launch files to adapt to your application. 

## Static Transform Publishers

More information is available [here](http://wiki.ros.org/tf#static_transform_publisher)

See file `stp.launch` in the `launch` folder of this package. Consider the following example:


```xml
<launch>

    <node pkg="tf2_ros" type="static_transform_publisher" name="frameA_frameB" args="x y z qx qy qz qw frameA frameB " />

</launch>
```
If we want to express a fixed transformation between `frameA` and `frameB` such that `frameA` is the parent of `frameB`. You can use the code above. For instance, to create a static transform between a frame `world` and frame `CW` you could have the following as the content of your launch file:

```xml
<launch>

    <node pkg="tf2_ros" type="static_transform_publisher" name="world_CW" args="1.0 1.0 1.0 0.0 0.0 0.0 1.0 world CW " />

</launch>
```

This will create a frame `CW` at a distance of x = 1 m, y = 1 m and z = 1 m from the origin of `world`. The orientation of `CW` will be the same as that of the `world`.

You can add multiple such transformation in the same launch file as follows:
```xml
<launch>

    <node pkg="tf2_ros" type="static_transform_publisher" name="frameA_frameB" args="x y z qx qy qz qw frameA frameB " />
    <node pkg="tf2_ros" type="static_transform_publisher" name="world_CW" args="1.0 1.0 1.0 0.0 0.0 0.0 1.0 world CW " />

</launch>
```

## Data-Recording:

Data can be recorded in the form of ROS topics, from which you can extract information using `Matlab ROS toolbox` or `bagpy` in python.


See file `record_data.launch` in the `launch` folder of this package. Consider the following example:

```xml
<launch>

    <arg name="topics_name" default="/topic1 /topic2 /topic3" />

    <node type="record" name="record" pkg="rosbag" args="record $(arg topics_name) -o /home/you_custom_path/Desktop/Recording/" /> 


</launch>
```
Launching a launch file having the above lines in it, will record the topics `/topic1`, `/topic2` and `/topic3` in a rosbag that will be saved in the folder `Recording` in the `Desktop` folder. 

You can press `Ctrl + C` from the terminal it was launched to stop the recording.

Note that the topics will be recording only if they are being published to.


## Conversion from tf to a topic of type geometry_msgs/PoseStamped
Suppose your tf tree contains two frames `frameA` and `frameB` and you want to obtain the pose of `frameB` expressed in `frameA`.
You can lookup the tranformation between the two frames on the tf tree, and publish this transformation as a topic of type `geometry_msgs/PoseStamped`. Consider the example launch file called `reference_transformer_chaser.launch`:

```xml

    <arg name="namespace_reference_transformer" default="" doc="Specify the name as parent_to_child"/>
    <arg name="parent_frame_rt" default="" doc="Specify the parent frame name"/>
    <arg name="child_frame_rt" default="" doc="Specify the child frame name"/>
    <arg name="frequency_rt" default="15.0" doc="Specify the frequency in hz"/>
    <arg name="topic_to_publish_to" default="/chaser_frame_pose" doc="Specify the frequency in hz"/>
    




    <group ns="$(arg namespace_reference_transformer)">
        
        <param name="parent_frame_rt" type="str" value="$(arg parent_frame_rt)"/>
        <param name="child_frame_rt" type="str" value="$(arg child_frame_rt)"/>
        <param name="frequency_rt" type="double" value="$(arg frequency_rt)"/>
        <param name="topic_to_publish_to" type="str" value="$(arg topic_to_publish_to)"/>

        <node name="chaser_frame_acquisition_node" pkg="zerog_utilities" type="reference_transformer_chaser"/>

    </group>


```

To launch this file with the desired transformation, you can launch the following from the terminal:

```terminal
roslaunch zerog_utilities reference_transformer_target.launch namespace_reference_transformer:=frameA_frameB parent_frame_rt:=frameA child_frame_rt:=frameB

```
This will create a topic called `/frameA_frameB/pose` which will contain the pose of `frameB` expressed with respect to `frameA`.


## Conversion from topic of type geometry_msgs/PoseStamped to tf frame

Suppose you have a topic called `/world_frameA/pose` which gives you the time-varying pose of an object to which `frameA` is attached. In other words, moving the object causes `frameA` to move. And at each time instant, the position and orientation of `frameA` is expressed in the `world` frame. We want this time-varying pose of `frameA` to appear as a tf frame. This can be thorough the launch file `obtain_waypts_transform.launch`:

```xml
<launch>
        <arg name="namespace_transformer" default="" doc="Specify the namespace in which you want the node"/>
        <arg name="parent_frame_rt" default="" doc="Specify the parent frame name"/>
        <arg name="child_frame_rt" default="" doc="Specify the child frame name"/>
        <arg name="frequency_rt" default="15.0" doc="Specify the frequency in hz"/>
        <arg name="topic_to_subscribe_to" default="" doc="Specify the frequency in hz"/>
        
    
    
    
    
        <group ns="$(arg namespace_transformer)">
            
            <param name="parent_frame_rt" type="str" value="$(arg parent_frame_rt)"/>
            <param name="child_frame_rt" type="str" value="$(arg child_frame_rt)"/>
            <param name="frequency_rt" type="double" value="$(arg frequency_rt)"/>
            <param name="topic_to_subscribe_to" type="str" value="$(arg topic_to_subscribe_to)"/>
    
            <node name="pose_to_tf" pkg="zerog_utilities" type="obtain_waypts_transform"/>
    
        </group>
</launch>
```

To obtain the `frameA` in the `tf_tree`, we can launch the above launch file from the command-line terminal as follows:

```terminal
roslaunch zerog_utilities obtain_waypts_transform.launch namespace_transformer:=wall_arm parent_frame_rt:=world child_frame_rt:=frameA topic_to_subscribe_to:=/world_frameA/pose
```

On running the above , you will observe an additional frame called `frameA` as part of your tf in the Rviz environment. 

**Note:** Specifying `namespace_transformer` helps launch multiple instances of the ROS node, so that we can have as many new frames as needed.

