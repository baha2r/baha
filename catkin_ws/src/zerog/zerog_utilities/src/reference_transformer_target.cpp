#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_frame_acquisition_node");
    ros::NodeHandle node;
    
    std::string parent_frame, child_frame, name_space, topic_to_publish_to;
    double freq, max_transform_age;

    node.getParam("namespace_reference_transformer", name_space);
    node.getParam("parent_frame_rt", parent_frame);
    node.getParam("child_frame_rt", child_frame);
    node.getParam("frequency_rt", freq);
    node.getParam("topic_to_publish_to", topic_to_publish_to);

    // Set the maximum acceptable age for a transform (in seconds)
    node.param("max_transform_age", max_transform_age, 2.0); // Default to 2 seconds if not set

    ros::Publisher marker_pose = node.advertise<geometry_msgs::PoseStamped>(topic_to_publish_to, 10);
    ros::Rate rate(freq);

    // Initialize the Buffer with a specific duration and then the TransformListener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);

    while (node.ok())
    {
        geometry_msgs::TransformStamped transform;

        try 
        {
            // Attempt to look up the transform with the buffer
            transform = tfBuffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // Check the age of the transform
        if ((ros::Time::now() - transform.header.stamp).toSec() > max_transform_age)
        {
            ROS_WARN("Transform is too old, skipping this publish cycle.");
            continue;
        }

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = parent_frame;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = transform.transform.translation.x;
        pose_msg.pose.position.y = transform.transform.translation.y;
        pose_msg.pose.position.z = transform.transform.translation.z;
        pose_msg.pose.orientation.x = transform.transform.rotation.x;
        pose_msg.pose.orientation.y = transform.transform.rotation.y;
        pose_msg.pose.orientation.z = transform.transform.rotation.z;
        pose_msg.pose.orientation.w = transform.transform.rotation.w;

        marker_pose.publish(pose_msg);
        rate.sleep();
    }

    return 0;
}
