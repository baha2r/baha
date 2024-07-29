#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('reference_transformer_node')
    
    parent_frame = rospy.get_param('~parent_frame_rt', 'world')
    child_frame = rospy.get_param('~child_frame_rt', 'ceiling_arm_tool0')
    frequency = rospy.get_param('~frequency_rt', 15.0)
    topic_to_publish_to = rospy.get_param('~topic_to_publish_to', '/transformed_waypoints')
    max_transform_age = rospy.get_param('~max_transform_age', 2.0)
    
    pub = rospy.Publisher(topic_to_publish_to, PoseStamped, queue_size=10)
    rate = rospy.Rate(frequency)
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    while not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time(0), rospy.Duration(max_transform_age))
            
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = parent_frame
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation.x = transform.transform.rotation.x
            pose_msg.pose.orientation.y = transform.transform.rotation.y
            pose_msg.pose.orientation.z = transform.transform.rotation.z
            pose_msg.pose.orientation.w = transform.transform.rotation.w

            pub.publish(pose_msg)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logerr(ex)
            rospy.sleep(1.0)
            continue
        
        rate.sleep()

if __name__ == '__main__':
    main()
