/*******************************************************************************
 *      Title     : moving_target_frame_with_subscriber.cpp
 *      Project   : Interspace
 *      Created   : 08/07/2022
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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
/**
 * Instantiate a moving frame wrt a specified parent frame.
 * This frame can be used as a tool with the pose-tracking code to enhance pose-tracking accuracy
 * The moving frame is imparted as a chaser motion.
 */

std::string parent_frame,child_frame;



void posCallback(const geometry_msgs::PoseStampedConstPtr& msg){

  static tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parent_frame;//"reference_target_pose";
  transformStamped.child_frame_id = child_frame;//"cookie";
  transformStamped.transform.translation.x = msg->pose.position.x;// 0.0;//0.15*sin(0.3*(ros::Time::now().toSec()));
  transformStamped.transform.translation.y = msg->pose.position.y;
  transformStamped.transform.translation.z = msg->pose.position.z;//0.15*cos(0.3*(ros::Time::now().toSec()));
  
  transformStamped.transform.rotation.x = msg->pose.orientation.x;
  transformStamped.transform.rotation.y = msg->pose.orientation.y;
  transformStamped.transform.rotation.z = msg->pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.orientation.w;
  //Logging
  ROS_INFO("target_x pose is: %f",msg->pose.position.x);
      
  tfb.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "moving_target_frame_node_target");

  if (argc != 3){ROS_ERROR("need parent and child_frame as argument"); return -1;};
  parent_frame = argv[1];
  child_frame = argv[2];
    
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/Way_pts_target_pose", 10, &posCallback);

  ros::spin();
  return 0;
};

