/********************************************
 * prepareData
 *
 * Ros node for preparation of an Imu msg to be used by MadgwickFilter.
 *
 * Adds a dummy orientation and acts as the placeholder for adding the Covariances
 * to an Imu msg. This eases the number of bytes sent over rosserial.
 *
 * Written and Maintained by Conor Messer
 * Last modified: 6/28/18
 *
 ********************************************/

#include "prepareData.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/AccelStamped.h>

  PrepareData::PrepareData()
  {
    // final_data publishes acceleration without gravity and oriented in the world frame
    pub = n.advertise<sensor_msgs::Imu>("imu/data_raw",1000);
    sub = n.subscribe("imu/acc_and_gyro", 1000, &PrepareData::callback, this);
  }

void PrepareData::callback(const geometry_msgs::AccelStamped& data)
  {
    // ROS initializations for publisher node
    sensor_msgs::Imu data_raw;

    if(ros::ok())
    {
      geometry_msgs::Quaternion orient;
      geometry_msgs::Vector3 ang_velo;
      geometry_msgs::Vector3 accel;

      // copy each field from data to final_data topic
      data_raw.header = data.header;

      // Covariances are unknown but should be found if KalmanFilter is used
      // data_raw.orientation_covariance = data.orientation_covariance;
      // data_raw.angular_velocity_covariance = data.angular_velocity_covariance;
      // data_raw.linear_acceleration_covariance = data.linear_acceleration_covariance;

      // orientation data is not used by MadgwickFilter
      orient.x = 0;
      orient.y = 0;
      orient.z = 0;
      orient.w = 1;
      data_raw.orientation = orient;

      // update the angular_velocity fields for prepare_data topic
      ang_velo = data.accel.angular;
      // ang_velo.x = data.angular_velocity.x;
      // ang_velo.y = data.angular_velocity.y;
      // ang_velo.z = data.angular_velocity.z;
      data_raw.angular_velocity = ang_velo;

      // update the acceleration fields for prepare_data topic
      accel = data.accel.linear;
      // accel.x = data.linear_acceleration.x;
      // accel.y = data.linear_acceleration.y;
      // accel.z = data.linear_acceleration.z;
      data_raw.linear_acceleration = accel;
    }


    pub.publish(data_raw);
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_raw");

  PrepareData PDObject;

  ros::spin();
  return 0;
}
