/*******************************************************************************
 *      Title     : tf_info_node.cpp
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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_info_node");
    ros:: NodeHandle node;

    std::string parent_frame,child_frame;
    double freq;
    node.getParam("parent_frame",parent_frame);
    node.getParam("child_frame",child_frame);
    node.getParam("freq",freq);


    /* Logging
    ROS_INFO("parent frame is: %s",parent_frame.c_str());
    ROS_INFO("child frame is: %s",child_frame.c_str());
    ROS_INFO("freq is: %f",freq);*/
    
    ros::Publisher marker_pose = node.advertise<geometry_msgs::PoseStamped>("pose",10);

    tf::TransformListener listener;
    ros::Rate rate(freq);

    while (node.ok())
    {
        tf::StampedTransform transform;

        try 
        {
        listener.waitForTransform(parent_frame,child_frame,ros::Time(0),ros::Duration(3.0));
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

        /* Logging
        ROS_INFO("parent frame is: %s",parent_frame.c_str());
        */

        marker_pose.publish(pose_msg);

        rate.sleep();


    }

    return 0;


};