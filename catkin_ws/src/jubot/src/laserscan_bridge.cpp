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

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

ros::Publisher publisher;
void laserscan_callback(ConstLaserScanStampedPtr &msg_in)
{ 

  sensor_msgs::LaserScan msg_out;
  msg_out.angle_min         = msg_in->scan().angle_min();
  msg_out.angle_max         = msg_in->scan().angle_min();
  msg_out.angle_increment   = msg_in->scan().angle_step();
  msg_out.time_increment    = 0;
  msg_out.scan_time         = 0;
  msg_out.range_min         = msg_in->scan().range_min();
  msg_out.range_max         = msg_in->scan().range_max();
  msg_out.header.stamp      = ros::Time::now();
  
  
  for(int i = 0; i < msg_in->scan().ranges_size(); i++){
    msg_out.ranges.push_back( msg_in->scan().ranges(i) );
  }
  for(int i = 0; i < msg_in->scan().intensities_size(); i++){
    msg_out.intensities.push_back( msg_in->scan().intensities(i) );
  }

  publisher.publish(msg_out);
}


int main(int argc, char** argv)
{

    // init ROS
    ros::init(argc, argv, "laserscan_bridge");
    ros::NodeHandle n;
    publisher = n.advertise<sensor_msgs::LaserScan>("/jubot/laserscan", 100);


    // When launched from a launch file we need to give Gazebo time to load
    ros::Duration(5.0).sleep();

    // init Gazebo
    gazebo::transport::init();
    gazebo::transport::NodePtr gz_node(new gazebo::transport::Node());
    gz_node->Init();
    gazebo::transport::run();

    gazebo::transport::SubscriberPtr gz_laserscan_sub = gz_node->Subscribe("/gazebo/default/jubot/hokuyo_link/laser/scan", laserscan_callback);

    // Spin
    ros::spin();

    // Shutdown
    gazebo::transport::fini();
    
}
