//
// The MIT License (MIT)
//
// Copyright (c) 2020 EAIBOT. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <limits>       // std::numeric_limits
#include "sdk/include/CYdLidar.h"



#define SDKROSVerision "1.0.0"
CYdLidar laser;

bool stop_scan(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res)
{
  ROS_DEBUG("Stop scan");
  return laser.turnOff();
}

bool start_scan(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res)
{
  ROS_DEBUG("Start scan");
  return laser.turnOn();
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "ydlidar_ros_driver");
  ROS_INFO("YDLIDAR ROS Driver Version: %s", SDKROSVerision);
  ros::NodeHandle nh;
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);

  ros::NodeHandle nh_private("~");
  std::string port = "/dev/ydlidar";
  nh_private.param<std::string>("port", port, "/dev/ydlidar");
  ///lidar port
  laser.setSerialPort(port);

  ///ignore array
  std::string ignore_array = "";
  nh_private.param<std::string>("ignore_array", ignore_array, "");

  std::string frame_id = "laser_frame";
  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");

  //////////////////////int property/////////////////
  /// lidar baudrate
  int baud = 256000;
  nh_private.param<int>("baudrate", baud, 256000);
  laser.setSerialBaudrate(baud);

  /// abnormal count
  int optval = 4;
  nh_private.param<int>("abnormal_check_count", optval, 4); 
  laser.setAbnormalCheckCount(optval);    

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  nh_private.param<bool>("fixed_resolution", b_optvalue, false);
  laser.setFixedResolution(b_optvalue);

  /// rotate 180
  b_optvalue = false;
  nh_private.param<bool>("reversion", b_optvalue, false);
  laser.setReversion(b_optvalue);
  
  b_optvalue = true;
  nh_private.param<bool>("auto_reconnect", b_optvalue, true);
  laser.setAutoReconnect(b_optvalue);

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  nh_private.param<float>("angle_max", f_optvalue, 180.f);
  laser.setMaxAngle(f_optvalue);

  f_optvalue = -180.0f;
  nh_private.param<float>("angle_min", f_optvalue, -180.f);
  laser.setMinAngle(f_optvalue);

  /// unit: m
  f_optvalue = 12.f;
  nh_private.param<float>("range_max", f_optvalue, 12.f);
  laser.setMaxRange(f_optvalue);

  f_optvalue = 0.01f;
  nh_private.param<float>("range_min", f_optvalue, 0.01f);
  laser.setMinRange(f_optvalue);

  /// unit: Hz
  f_optvalue = 6.f;
  nh_private.param<float>("frequency", f_optvalue, 6.f);
  laser.setScanFrequency(6.0);

  bool invalid_range_is_inf = false;
  nh_private.param<bool>("invalid_range_is_inf", invalid_range_is_inf, invalid_range_is_inf);

  ros::ServiceServer stop_scan_service = nh.advertiseService("stop_scan", stop_scan);
  ros::ServiceServer start_scan_service = nh.advertiseService("start_scan", start_scan);

  // initialize SDK and LiDAR
  bool ret = laser.initialize();
  if (ret) {//success
    //Start the device scanning routine which runs on a separate thread and enable motor.
    ret = laser.turnOn();
  } else {
    ROS_ERROR("%s\n", ydlidar::protocol::DescribeError(laser.getDriverError()));
  }

  ros::Rate r(30);
  while (ret && ros::ok()) {
    LaserScan scan;
    bool hardwareError;
    if (laser.doProcessSimple(scan, hardwareError)) {
      sensor_msgs::LaserScan scan_msg;
      ros::Time start_scan_time;
      start_scan_time.sec = scan.system_time_stamp/1000000000ul;
      start_scan_time.nsec = scan.system_time_stamp%1000000000ul;
      scan_msg.header.stamp = start_scan_time;
      scan_msg.header.frame_id = frame_id;
      scan_msg.angle_min =(scan.config.min_angle);
      scan_msg.angle_max = (scan.config.max_angle);
      scan_msg.scan_time = scan.config.scan_time;
      scan_msg.time_increment = scan.config.time_increment;
      scan_msg.range_min = (scan.config.min_range);
      scan_msg.range_max = (scan.config.max_range);
      int size = scan.data.size();
      scan_msg.angle_increment =  (scan_msg.angle_max - scan_msg.angle_min) / (size);
      scan_msg.ranges.resize(size, invalid_range_is_inf ? std::numeric_limits<float>::infinity() : 0.0);
      scan_msg.intensities.resize(size);
      for(size_t i=0; i < scan.data.size(); i++) {
        int index = std::ceil((scan.data[i].angle - scan.config.min_angle)/scan_msg.angle_increment);
        if(index >=0 && index < size) {
          if(scan.data[i].range >= scan.config.min_range) {
            scan_msg.ranges[index] = scan.data[i].range;
            scan_msg.intensities[index] = scan.data[i].intensity;
          }
        }
      }

      scan_pub.publish(scan_msg);

    } else {
      ROS_ERROR("Failed to get Lidar Data: %s",ydlidar::protocol::DescribeError(laser.getDriverError()));
    }
    r.sleep();
    ros::spinOnce();
  }

  laser.turnOff();
  ROS_INFO("[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.disconnecting();
  return 0;
}


