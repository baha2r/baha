/*******************************************************************************
 *      Title     : twist_tf_info_node.cpp
 *      Project   : Interspace
 *      Created   : 11/06/2022
 *      Author    : Mohatashem Reyaz Makhdoomi
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, <insert company name>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <zerog_utilities/TfInfoConfig.h> // Replace with your package name

double averaging_interval_sec = 0.1; // Default averaging interval

void getTwist(tf::TransformListener &listener, const std::string& parent_frame, const std::string& child_frame, const ros::Time& time, const ros::Duration& averaging_interval, geometry_msgs::Twist& twist)
{
    try 
    {
        tf::StampedTransform start, end;
        ros::Time latest_time;
        listener.getLatestCommonTime(parent_frame, child_frame, latest_time, NULL);
        
        ros::Time target_time = (time == ros::Time()) ? latest_time : time;
        ros::Time end_time = std::min(target_time + averaging_interval * 0.5 , latest_time);
        ros::Time start_time = std::max(ros::Time().fromSec(.00001) + averaging_interval, end_time) - averaging_interval;
        ros::Duration corrected_averaging_interval = end_time - start_time;

        listener.lookupTransform(parent_frame, child_frame, start_time, start);
        listener.lookupTransform(parent_frame, child_frame, end_time, end);

        tf::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
        tf::Quaternion quat_temp;
        temp.getRotation(quat_temp);
        tf::Vector3 o = start.getBasis() * quat_temp.getAxis();
        tfScalar ang = quat_temp.getAngle();
        
        double delta_x = end.getOrigin().x() - start.getOrigin().x();
        double delta_y = end.getOrigin().y() - start.getOrigin().y();
        double delta_z = end.getOrigin().z() - start.getOrigin().z();

        tf::Vector3 twist_vel(delta_x / corrected_averaging_interval.toSec(), 
                              delta_y / corrected_averaging_interval.toSec(),
                              delta_z / corrected_averaging_interval.toSec());
        tf::Vector3 twist_rot = o * (ang / corrected_averaging_interval.toSec());

        twist.linear.x = twist_vel.x();
        twist.linear.y = twist_vel.y();
        twist.linear.z = twist_vel.z();
        twist.angular.x = twist_rot.x();
        twist.angular.y = twist_rot.y();
        twist.angular.z = twist_rot.z();
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void callback(zerog_utilities::TfInfoConfig &config, uint32_t level) // 
{
    averaging_interval_sec = config.averaging_interval;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist_tf_info_node");
    ros::NodeHandle node;

    std::string parent_frame, child_frame;
    double freq;
    node.getParam("parent_frame", parent_frame);
    node.getParam("child_frame", child_frame);
    node.getParam("freq", freq);    

    /* Logging
    ROS_INFO("parent frame is: %s", parent_frame.c_str());
    ROS_INFO("child frame is: %s", child_frame.c_str());
    ROS_INFO("freq is: %f", freq);*/
    
    ros::Publisher marker_pose = node.advertise<geometry_msgs::PoseStamped>("pose", 10);
    ros::Publisher velocity_pub = node.advertise<geometry_msgs::TwistStamped>("twist", 10);

    tf::TransformListener listener;
    ros::Rate rate(freq);

    // Dynamic reconfigure server
    dynamic_reconfigure::Server<zerog_utilities::TfInfoConfig> server; // Replace with your package name
    dynamic_reconfigure::Server<zerog_utilities::TfInfoConfig>::CallbackType f; // Replace with your package name

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    while (node.ok())
    {
        tf::StampedTransform transform;

        try 
        {
            listener.waitForTransform(parent_frame, child_frame, ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
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

        /* Logging
        ROS_INFO("parent frame is: %s", parent_frame.c_str());
        */

        marker_pose.publish(pose_msg);

        geometry_msgs::Twist twist;
        ros::Duration averaging_interval(averaging_interval_sec);
        getTwist(listener, parent_frame, child_frame, ros::Time(0), averaging_interval, twist);

        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header.frame_id = parent_frame;
        twist_msg.header.stamp = ros::Time::now();
        twist_msg.twist = twist;

        velocity_pub.publish(twist_msg);

        rate.sleep();
    }

    return 0;
}
