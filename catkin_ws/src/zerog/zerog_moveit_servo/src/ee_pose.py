#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def get_transform(parent_frame, child_frame):
    rospy.init_node('tf_listener_node', anonymous=True)
    listener = tf.TransformListener()
    
    # Wait for the listener to get the first message
    rospy.sleep(1.0)  # Ensuring the listener has enough time to start
    listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(4.0))

    # List all available frames
    # frames = listener.getFrameStrings()
    # print("Available frames: ", frames)

    try:
        # Lookup the transform at the latest available time
        (trans, rot) = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(e)
        return None, None
    
    return trans, rot

if __name__ == '__main__':
    # ('Available frames: ', ['wall_arm_upper_arm_link', 'ceiling_arm_flange', 'ceiling_arm_base', 
    # 'capture_frame', 'ceiling_arm_tool0', 'wall_arm_shoulder_link', 'ceiling_arm_base_link', 
    # 'ceiling_arm_upper_arm_link', 'wall_arm_base', 'wall_track_base_link_inertia', 'ceiling_arm_wrist_3_link', 
    # 'ceiling_arm_wrist_2_link', 'wall_arm_flange', 'CW', 'wall_arm_wrist_3_link', 'wall_arm_base_link_inertia', 
    # 'wall_arm_wrist_2_link', 'ceiling_arm_shoulder_link', 'wall_arm_forearm_link', 'wall_arm_base_link', 
    # 'ceiling_track_slider_link', 'ceiling_track_base_link_inertia', 'wall_arm_wrist_1_link', 'ceiling_arm_wrist_1_link',
    # 'ceiling_arm_forearm_link', 'ceiling_arm_base_link_inertia', 'wall_track_base_link', 'wall_track_slider_link',
    # 'wall_arm_tool0', 'zerog_room_frame', 'ceiling_track_base_link'])
    parent_frame = 'capture_frame'
    child_frame = 'wall_arm_tool0'
    trans, rot = get_transform(parent_frame, child_frame)
    print("{} located at {} in the coordinate system of the {}".format(child_frame,trans,parent_frame))
    
    if trans is not None and rot is not None:
        print("Got the transformation from {} to {}".format(parent_frame, child_frame))
        print("Translation vector: {}".format(trans))
        # Get the current time
        current_time = rospy.Time.now()
        
        # Print the transformation
        print("At time {}".format(current_time.to_sec()))
        print("- Translation: [{}, {}, {}]".format(trans[0], trans[1], trans[2]))
        print("- Rotation: in Quaternion [{}, {}, {}, {}]".format(rot[0], rot[1], rot[2], rot[3]))
        
        # Convert quaternion to Euler angles
        euler = tf.transformations.euler_from_quaternion(rot)
        print("            in RPY (radian) [{}, {}, {}]".format(euler[0], euler[1], euler[2]))
        print("            in RPY (degree) [{}, {}, {}]".format(euler[0] * 180.0 / 3.14159, euler[1] * 180.0 / 3.14159, euler[2] * 180.0 / 3.14159))
    else:
        print("Failed to get transformation from {} to {}".format(parent_frame, child_frame))
