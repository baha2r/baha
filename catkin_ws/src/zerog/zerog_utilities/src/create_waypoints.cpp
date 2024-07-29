/*******************************************************************************
 *      Title     : create_waypoints.cpp
 *      Project   : Interspace
 *      Created   : 23/07/2023
 *      Author    : Mohatashem Reyaz Makhdoomi
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, <insert company name>
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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


/**
 * Instantiate a moving frame wrt a specified parent frame.
 * This frame can be used as a tool with the pose-tracking code to enhance pose-tracking accuracy when tuning controller
 * The moving frame is imparted a periodic motion.
 */

int main(int argc, char** argv){
    ros::init(argc, argv, "create_waypoints");
    ros::NodeHandle node;

    std::string parent_frame,child_frame;double freq, traj_duration;
    node.getParam("parent_frame_mtf",parent_frame);
    node.getParam("child_frame_mtf",child_frame);
    node.getParam("frequency_mtf",freq);
    node.getParam("traj_duration", traj_duration);


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped, initialTransform;

    transformStamped.header.frame_id = parent_frame;
    transformStamped.child_frame_id = child_frame;
    
    double x_offset, y_offset, z_offset, roll_offset, pitch_offset, yaw_offset;

    geometry_msgs::Quaternion intial_quaternion, normalized_quaternion;
    tf2::Quaternion tf_quaternion ;
    
    ros::Rate rate(freq);
        
    // Display a prompt to the user
    std::cout << "Press Enter to continue: ";
    // Wait for the user to press Enter.
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    // Code continues automatically without requiring any other input.
    std::cout << "Continuing with the code..." << std::endl;

    // Flag to check if offset values have been obtained
    bool offset_obtained = false;
    double elapsed_time;
    // Store the initial time when the program starts
    ros::Time start_time = ros::Time::now();
    while (node.ok())
    {   

        if ((ros::Time::now() - start_time).toSec() >= traj_duration)
        {
            ROS_INFO("Elapsed time >= 30 seconds. Shutting down the node.");
            ros::shutdown(); // Terminate the node gracefully
        }
        if (!offset_obtained) {
            try {
                // Lookup the initial transform between parent_frame and child_frame
                initialTransform = tfBuffer.lookupTransform(
                    parent_frame, child_frame, ros::Time(0)
                );

                // Set the initial translation and rotation based on the lookup transform
                x_offset = initialTransform.transform.translation.x;
                y_offset = initialTransform.transform.translation.y;
                z_offset = initialTransform.transform.translation.z;

                tf2::fromMsg(initialTransform.transform.rotation, tf_quaternion);
                tf2::Matrix3x3 mat(tf_quaternion);
                mat.getRPY(roll_offset, pitch_offset, yaw_offset);

                // Mark the offset values as obtained
                offset_obtained = true;

                ROS_INFO("RPY Angles: Roll=%.3f Pitch=%.3f Yaw=%.3f", roll_offset, pitch_offset, yaw_offset);

            } catch (tf2::TransformException& ex) {
                // If the lookup fails, handle the exception here (you can set default initial conditions)
                ROS_WARN("Failed to lookup initial transform: %s", ex.what());
            }
        }
        
        elapsed_time = (ros::Time::now() - start_time).toSec();
        double speed_factor =1; 
        double x_motion = 0.0;// 1.5*std::abs(1.0 * sin(0.3 * speed_factor*elapsed_time));
        double y_motion = 0.0;//-0.1*(1.0 * sin(1.5 *speed_factor* elapsed_time));//0.0; // std::abs(1.0 * sin(0.3 * elapsed_time));
        double z_motion = 0.15*std::abs(1.0 * sin(0.2 *speed_factor* elapsed_time));
        double r_motion = 0.0; //std::abs(1.0 * sin(0.3 * elapsed_time));
        double p_motion = 0.0;//-0.785*sin(1.2 * speed_factor*elapsed_time); //std::abs(1.0 * sin(0.3 * elapsed_time));
        double yw_motion = 0.0;//-3.14*sin(0.3 * elapsed_time);; //std::abs(1.0 * sin(0.3 * elapsed_time));

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.frame_id = parent_frame;
        transformStamped.transform.translation.x = x_offset + x_motion;// 0.15*sin(0.3*(ros::Time::now().toSec()));
        transformStamped.transform.translation.y = y_offset + y_motion;
        transformStamped.transform.translation.z = z_offset + z_motion;//0.15*cos(0.3*(ros::Time::now().toSec()));
        
        double roll = roll_offset + r_motion ;
        double pitch = pitch_offset + p_motion;
        double yaw = yaw_offset + yw_motion;

        tf2::Quaternion tf_quaternion;
        // Set the orientation based on the RPY angles
        tf_quaternion.setRPY(roll, pitch, yaw);
        // Normalize the quaternion to ensure it is valid
        tf_quaternion.normalize();
        if (std::abs(tf_quaternion.length() - 1.0) > 1e-5) {
            // Handle the case where the quaternion is not normalized
            ROS_WARN("Quaternion is not normalized. Setting default values.");
            tf_quaternion.setRPY(0.0, 0.0, 0.0); // Set default orientation
        }

        ROS_INFO("RPY Angles: Roll=%.3f Pitch=%.3f Yaw=%.3f", roll, pitch, yaw);

        ROS_INFO("Quaternion: w=%.3f x=%.3f y=%.3f z=%.3f",
          tf_quaternion.w(), tf_quaternion.x(),
          tf_quaternion.y(), tf_quaternion.z());
        // Convert the tf2::Quaternion back to geometry_msgs::Quaternion
        geometry_msgs::Quaternion rpy_quaternion = tf2::toMsg(tf_quaternion);
        
        transformStamped.transform.rotation.w = rpy_quaternion.w;
        transformStamped.transform.rotation.x = rpy_quaternion.x;
        transformStamped.transform.rotation.y = rpy_quaternion.y;
        transformStamped.transform.rotation.z = rpy_quaternion.z;


        tfb.sendTransform(transformStamped);
        rate.sleep();
        /* Logging
        ROS_INFO("parent frame is: %s",parent_frame.c_str());
        */
    }

    return 0;
};
