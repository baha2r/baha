#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from std_msgs.msg import Float64MultiArray, Bool

class ActionListener:
    def __init__(self):
        self.action = None
        rospy.Subscriber('/robotiq_action', Float64MultiArray, self.callback)
        
    def callback(self, msg):
        self.action = list(msg.data)

    def get_action(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.action is not None:
                action = self.action
                self.action = None
                return action
            rate.sleep()

def initialize_robot():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    return robot, scene

def initialize_move_group(group_name):
    return moveit_commander.MoveGroupCommander(group_name)

def transform_pose(tf_listener, pose, from_frame, to_frame):
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = from_frame
    pose_stamped.pose = pose

    try:
        tf_listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))
        transformed_pose_stamped = tf_listener.transformPose(to_frame, pose_stamped)
        return transformed_pose_stamped.pose
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
        rospy.logerr(ex)
        return None

def apply_action(current_pose, action):
    new_pose = geometry_msgs.msg.Pose()
    euler = tf.transformations.euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, 
                                                      current_pose.orientation.z, current_pose.orientation.w])
    new_pose.position.x = current_pose.position.x + action[0] * 0.01
    new_pose.position.y = current_pose.position.y + action[1] * 0.01
    new_pose.position.z = current_pose.position.z + action[2] * 0.01
    new_quaternion = tf.transformations.quaternion_from_euler(euler[0] + action[3]* 0.01, euler[1] + action[4]* 0.01, euler[2] + action[5]* 0.01)
    new_pose.orientation.x = new_quaternion[0] 
    new_pose.orientation.y = new_quaternion[1] 
    new_pose.orientation.z = new_quaternion[2] 
    new_pose.orientation.w = new_quaternion[3]
    return new_pose

def set_tolerance_and_planning_time(move_group):
    move_group.set_goal_position_tolerance(0.0001)
    move_group.set_goal_orientation_tolerance(0.1)
    move_group.set_planning_time(100)  # Increase planning time

def set_target_pose(move_group, target_pose):
    move_group.set_pose_target(target_pose)
    plan = move_group.go(wait=True)
    return plan

def main():
    robot, scene = initialize_robot()
    tf_listener = tf.TransformListener()
    rospy.sleep(1.0)

    wall_arm_control = initialize_move_group("wall_arm")
    set_tolerance_and_planning_time(wall_arm_control)
    
    action_listener = ActionListener()
    completion_pub = rospy.Publisher('/action_completion', Bool, queue_size=10)
    
    while not rospy.is_shutdown():
        action = action_listener.get_action()
        if action:
            wall_arm_current_pose = wall_arm_control.get_current_pose().pose
            capture_frame = "capture_frame"
            wall_arm_transformed_pose = transform_pose(tf_listener, wall_arm_current_pose, wall_arm_control.get_planning_frame(), capture_frame)
            wall_arm_target_transformed_pose = apply_action(wall_arm_transformed_pose, action)
            rospy.loginfo("Applying action: {}, and moving the wall arm to the {}".format(action, wall_arm_target_transformed_pose))
            wall_arm_target_pose = transform_pose(tf_listener, wall_arm_target_transformed_pose, capture_frame, wall_arm_control.get_planning_frame())
            plan = set_target_pose(wall_arm_control, wall_arm_target_pose)
            if not plan:
                rospy.logwarn("Planning failed, no valid plan found.")
            else:
                wall_arm_control.stop()
                wall_arm_control.clear_pose_targets()
                completion_pub.publish(Bool(data=True))
                rospy.loginfo("Action completed")
                
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
