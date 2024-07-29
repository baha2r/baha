#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_in_ctrl_frame_acquisition_node");
    ros:: NodeHandle node;
    tf::TransformListener listener;
    std::string parent_frame,child_frame,name_space, topic_to_publish_to; double freq;
    node.getParam("namespace_reference_transformer",name_space);
    node.getParam("parent_frame_rt",parent_frame);
    node.getParam("child_frame_rt",child_frame);
    node.getParam("frequency_rt",freq);
    node.getParam("topic_to_publish_to",topic_to_publish_to);

    ros::Publisher marker_pose = node.advertise<geometry_msgs::PoseStamped>(topic_to_publish_to,10);
    //ros::Publisher marker_pose = node.advertise<geometry_msgs::PoseStamped>("/Way_pts_chaser_pose",10);

    //ros::Publisher marker_pose = node.advertise<geometry_msgs::PoseStamped>("/servo_server/target_pose",10);

    ros::Rate rate(freq);

    while (node.ok())
    {
        tf::StampedTransform transform;

        try 
        {
        listener.waitForTransform(parent_frame,child_frame,ros::Time(0),ros::Duration(5.0));
        listener.lookupTransform(parent_frame,child_frame,ros::Time(0),transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = parent_frame;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = transform.getOrigin().x();
        pose_msg.pose.position.y = transform.getOrigin().y();
        pose_msg.pose.position.z = transform.getOrigin().z();
        pose_msg.pose.orientation.x = transform.getRotation().x();
        pose_msg.pose.orientation.y = transform.getRotation().y();        
        pose_msg.pose.orientation.z = transform.getRotation().z();        
        pose_msg.pose.orientation.w = transform.getRotation().w();

        marker_pose.publish(pose_msg);

        rate.sleep();


    }

    return 0;


};