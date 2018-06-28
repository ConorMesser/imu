#ifndef FINAL_DATA_H
#define FINAL_DATA_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


class PublishFinalData
{
public:
  PublishFinalData();
  void callback(const sensor_msgs::Imu& data);
  double firstOrderApprox(double val_prev, double val_dot_prev, double val_dot_curr, double dt);
  void initializeData();
  void shiftData();

private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::Publisher pub_velocity;
  ros::Publisher pub_position;

  double acceleration_x[2], acceleration_y[2], acceleration_z[2];
  double velocity_x[2], velocity_y[2], velocity_z[2];
  double position_x[2], position_y[2], position_z[2];
  ros::Time data_time[2];
  double dt;
  const static double GRAV = 9.80665;
  int count;
  const static int COUNT_PARAM = 1000;
  double x_accel_accum;
  double y_accel_accum;
  double z_accel_accum;
  double ss_accel_x;
  double ss_accel_y;
  double ss_accel_z;
};

#endif
