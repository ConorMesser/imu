/********************************************
 * finalData
 *
 * Ros node for final modification of IMU data and publication.
 *
 * Removes the steady-state acceleration biases from the acceleration data and
 * calculates the velocity and position by integration of the acceleration.
 * Publishes an Imu msg on the /imu/final_data topic.
 *
 * Written and Maintained by Conor Messer
 * Last modified: 6/28/18
 *
 ********************************************/

#include "finalData.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <Quaternion.hpp>

  PublishFinalData::PublishFinalData()
  {
    // final_data publishes acceleration without gravity and oriented in the world frame
    pub = n.advertise<sensor_msgs::Imu>("imu/final_data",1000);
    pub_velocity = n.advertise<geometry_msgs::Vector3>("imu/velocity",1000);
    pub_position = n.advertise<geometry_msgs::Vector3>("imu/position",1000);
    sub = n.subscribe("imu/data", 1000, &PublishFinalData::callback, this);

    PublishFinalData::initializeData();
  }

void PublishFinalData::callback(const sensor_msgs::Imu& data)
  {


    // ROS initializations for publisher node
    sensor_msgs::Imu final_data;
    geometry_msgs::Vector3 velocity;
    geometry_msgs::Vector3 position;

    if(ros::ok())
    {
      count++;
      // removes gravity and returns a quaternion of the acceleration in sensor frame
      Quaternion acc_quat = Quaternion::removeGravity(data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z,data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w, GRAV);

      // Follows a certain length of time for Calibration
      //  COUNT_PARAM must be larger than the time for gyroscope calibration TODO
      if (count < COUNT_PARAM)
      {
        x_accel_accum += acc_quat.X;
        y_accel_accum += acc_quat.Y;
        z_accel_accum += acc_quat.Z;
      }
      // After calibration, the bias values for the acceleration are calculated
      if (count == COUNT_PARAM)
      {
        ss_accel_x = x_accel_accum/(COUNT_PARAM - 1);
        ss_accel_y = y_accel_accum/(COUNT_PARAM - 1);
        ss_accel_z = z_accel_accum/(COUNT_PARAM - 1);

        std::cout << "x: " << ss_accel_x << "\n";
        std::cout << "y: " << ss_accel_y << "\n";
        std::cout << "z: " << ss_accel_z << "\n";
      }

      // Rotate acceleration to world frame according to orientation of device with steady state values (and gravity) removed
      Quaternion acc_quat_final = Quaternion::changeFrame(acc_quat.X - ss_accel_x,acc_quat.Y - ss_accel_y,acc_quat.Z - ss_accel_z,data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w);

      // discard the w term of acc_quat_no_grav (should be 0)
      acceleration_x[1] = acc_quat_final.X;
      acceleration_y[1] = acc_quat_final.Y;
      acceleration_z[1] = acc_quat_final.Z;

      // copy each field from data to final_data topic
      final_data.header = data.header;
      final_data.orientation = data.orientation;
      final_data.orientation_covariance = data.orientation_covariance;
      final_data.angular_velocity = data.angular_velocity;
      final_data.angular_velocity_covariance = data.angular_velocity_covariance;
      final_data.linear_acceleration_covariance = data.linear_acceleration_covariance;

      // update the acceleration fields for final_data topic with the rotated
      // acceleration with gravity removed
      geometry_msgs::Vector3 accel;
      accel.x = acceleration_x[1];
      accel.y = acceleration_y[1];
      accel.z = acceleration_z[1];

      final_data.linear_acceleration = accel;

      // Position and velocity data only calculated after acceleration calibration
      if (count > COUNT_PARAM) {
        // timing for integration
        data_time[1] = data.header.stamp;

        double time_after = data_time[1].nsec;
        double time_before = data_time[0].nsec;
        if (time_after < time_before)  // only works if dt < 1 sec
        {
          time_after += 1000000000;
        }
        dt = (time_after - time_before)/1000000000;

        // calculate velocity and position vectors using firstOrderApprox and previous data
        velocity_x[1] = PublishFinalData::firstOrderApprox(velocity_x[0],acceleration_x[0],acceleration_x[1],dt);
        position_x[1] = PublishFinalData::firstOrderApprox(position_x[0],velocity_x[0],velocity_x[1],dt);
        velocity_y[1] = PublishFinalData::firstOrderApprox(velocity_y[0],acceleration_y[0],acceleration_y[1],dt);
        position_y[1] = PublishFinalData::firstOrderApprox(position_y[0],velocity_y[0],velocity_y[1],dt);
        velocity_z[1] = PublishFinalData::firstOrderApprox(velocity_z[0],acceleration_z[0],acceleration_z[1],dt);
        position_z[1] = PublishFinalData::firstOrderApprox(position_z[0],velocity_z[0],velocity_z[1],dt);

        // prepare velocity msg
        velocity.x = velocity_x[1];
        velocity.y = velocity_y[1];
        velocity.z = velocity_z[1];

        // prepare position msg
        position.x = position_x[1];
        position.y = position_y[1];
        position.z = position_z[1];
      }
    }

    // Publish on all topics
    pub.publish(final_data);
    pub_velocity.publish(velocity);
    pub_position.publish(position);

    PublishFinalData::shiftData();
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "final_data");

  PublishFinalData PFDObject;

  ros::spin();
  return 0;
}


/* Use first-order approximation (trapezoidal) to integrate over this timestep
 * @param  val_prev      integrated value at last timestep
 * @param  val_dot_prev  value at last timestep
 * @param  val_dot_curr  value at current timestep
 * @param  dt            duration of timestep
 * @return current integrated value
 */
double PublishFinalData::firstOrderApprox(double val_prev, double val_dot_prev, double val_dot_curr, double dt)
{
  double val_current = val_prev + (val_dot_prev + ((val_dot_curr - val_dot_prev)/2))*dt;
  return val_current;
}

/* Initialize all private array fields to zero
 * @return void
 */
void PublishFinalData::initializeData()
{
  acceleration_x[0] = 0;
  acceleration_y[0] = 0;
  acceleration_z[0] = 0;
  velocity_x[0] = 0;
  velocity_y[0] = 0;
  velocity_z[0] = 0;
  position_x[0] = 0;
  position_y[0] = 0;
  position_z[0] = 0;
  data_time[0] = ros::Time::now();
  count = 0;
  x_accel_accum = 0;
  y_accel_accum = 0;
  z_accel_accum = 0;
  ss_accel_x = 0;
  ss_accel_y = 0;
  ss_accel_z = 0;
}

/* Move the current data to the place for past data, making way for new data
 * @return void
 */
void PublishFinalData::shiftData()
{
  acceleration_x[0] = acceleration_x[1];
  acceleration_y[0] = acceleration_y[1];
  acceleration_z[0] = acceleration_z[1];
  velocity_x[0] = velocity_x[1];
  velocity_y[0] = velocity_y[1];
  velocity_z[0] = velocity_z[1];
  position_x[0] = position_x[1];
  position_y[0] = position_y[1];
  position_z[0] = position_z[1];
  data_time[0] = data_time[1];
}
