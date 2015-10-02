/*
 * Author: Philipp Weidel
 *
 * MODIFIED FROM:
 *
 * Copyright 2012 Open Source Robotics Foundation
 * Copyright 2013 Dereck Wonnacott
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>


gazebo::transport::PublisherPtr cmd_publisher;


void cmd_callback(const geometry_msgs::Twist::ConstPtr& msg_in)
{ 
  // Generate a pose
  gazebo::math::Pose pose(msg_in->linear.x,
                          msg_in->linear.y,
                          msg_in->linear.z,
                          msg_in->angular.x,
                          msg_in->angular.y,
                          msg_in->angular.z);
  
  // Convert to a pose message
  gazebo::msgs::Pose msg_out;
  gazebo::msgs::Set(&msg_out, pose);
  cmd_publisher->Publish(msg_out);
}


int main(int argc, char** argv)
{

    // init ROS
    ros::init(argc, argv, "cmd_bridge");
    ros::NodeHandle n;

    // When launched from a launch file we need to give Gazebo time to load
    ros::Duration(5.0).sleep();

     // Initialize Gazebo
    gazebo::transport::init();
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::run();
    cmd_publisher = node->Advertise<gazebo::msgs::Pose>("/gazebo/default/jubot/vel_cmd");

    ros::Subscriber cmd_subscriber = n.subscribe("/jubot/cmd_vel", 1, cmd_callback);

    // Spin
    ros::spin();

    // Shutdown
    gazebo::transport::fini();

}

