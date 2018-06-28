#ifndef PREPARE_DATA_H
#define PREPARE_DATA_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/AccelStamped.h>

class PrepareData
{
public:
  PrepareData();
  void callback(const geometry_msgs::AccelStamped& data);
  void initializeData();

private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;
};

#endif
