/********************************************
 * MPU9250_Full
 *
 * Sketch for the control of the MPU9250 with
 * calibration of the accelerator and gyroscope,
 * and a Kalman filter.
 *
 * Modified from MPU9250_Basic from SparkFun_MPU9250_DMP_Arduino_Library
 * and Motion Tracking Glove for Human-Machine Interaction article
 *
 * Maintained by Conor Messer
 * Last modified: 4/5/18
 *
 */

 #include <Arduino.h>

 #include <ros/ros.h>
 #include <sensor_msgs/Imu.h>
 #include <geometry_msgs/Quaternion.h>
 #include <geometry_msgs/Vector3.h>

 #include <SparkFunMPU9250-DMP.h>
 #include <Quaternion.hpp>

 // initialize imu object from SparkFun_MPU9250_DMP_Arduino_Library
 MPU9250_DMP imu;

 // ROS initializations for publisher node
 ros::NodeHandle nh;
 ros::Publisher pub_imu = node_handle.advertise<sensor_msgs::Imu>("imu",1000);
 sensor_msgs::Imu msg;

 // Global variables
 double x_angular_rate[2], y_angular_rate[2], z_angular_rate[2];
 double x_angle[2], y_angle[2], z_angle[2];
 double x_acceleration[2], y_acceleration[2], z_acceleration[2];
 double x_velocity[2], y_velocity[2], z_velocity[2];
 double x_position[2], y_position[2], z_position[2];

 Quaternion acc_quat_no_grav = Quaternion(0,0,0,0);

 double ss_gyro_x, ss_gyro_y, ss_gyro_z, ss_accel_x, ss_accel_y, ss_accel_z; //used for calibration**

 double start_time = 0;
 double dt = 0; //time step between samples

 float angle, x_pos, y_pos, z_pos;

 // to space out how often data prints and publishes
 int count = 0;

 void setup()
{
  Serial.begin(115200);  // TODO do I still need serial is this is publisher node?

  nh.initNode(); // initialize this as a node
  nh.advertise(pub_imu); // advertise this as a publisher node

  // initialize globals
  initializeData();

  // Call imu.begin() to verify communication with and
  // initialize the MPU-9250 to it's default values.
  // Most functions return an error code - INV_SUCCESS (0)
  // indicates the IMU was present and successfully set up
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }

  // Use setSensors to turn on or off MPU-9250 sensors.
  // Any of the following defines can be combined:
  // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
  // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  // Enable all sensors:
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(1000);
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(4);
  // Note: the MPU-9250's magnetometer FSR is set at
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(20);

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(400);

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(10);

  // Enable gyro calibration - *** this needs FIFO to work?
  // imu.dmpBegin(DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO, 10);

  // calibrate gyro, using average values taken over 8 seconds.
  calibrateGyro();
  // calibrate accel*****??????TODO
  // if sensor is steady, the magnitude of the accel should be 1 (gravity term)
  // if sensor is perfectly oriented with z vertical, it could be calibrated at start
  // also possibly using magnetometer???

  // do we want band-pass filter of gyro or accel data?????

  /**
  * How do I work with the dmp?
  * And specifically how do I work with the FIFO? I can enable it
  * although last I checked, it didn't pull the accel data correctly.
  * But I don't know if I want to use it because of the timing. The way it was
  * implemented in the example is to pull data when it is full but I want
  * to process the data right away.
  */

  // Set which sensors should be stored in the buffer
  // imu.configureFifo(INV_XYZ_GYRO | INV_XYZ_ACCEL); // wasn't working for accel


  start_time = millis(); // reset the start time after calibration
}

void loop()
{
    count += 1;

    // Call updateFifo to update ax, ay, az, gx, gy, and/or gz
      if ( imu.dataReady() )
      {
        // Call update() to update the imu objects sensor data.
        // You can specify which sensors to update by combining
        // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
        // UPDATE_TEMPERATURE.
        // (The update function defaults to accel, gyro, compass,
        //  so you don't have to specify these values.)
        imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

        dt = (millis() - start_time)/1000; // time step of this data sample in sec
        start_time = millis(); // recording the present time for the next data sample

        // read in values to arrays
        readAccelData();
        readGyroData();

        // Kalman filter of data*******TODO

        // integrate angular acc using first-order approximation
        x_angle[1] = firstOrderApprox(x_angle[0],x_angular_rate[0],x_angular_rate[1],dt);
        y_angle[1] = firstOrderApprox(y_angle[0],y_angular_rate[0],y_angular_rate[1],dt);
        z_angle[1] = firstOrderApprox(z_angle[0],z_angular_rate[0],z_angular_rate[1],dt);

        // Get quaternions for the orientation from the Euler angles (in degrees)
        // and for the acceleration vector
        Quaternion acc_quat_final_rot = Quaternion::changeFrame(x_acceleration[1],y_acceleration[1],z_acceleration[1],z_angle[1],x_angle[1],y_angle[1]);

        // remove gravity from acc (subtract 1 g from the z-dir in Earth frame)
        acc_quat_no_grav = Quaternion::removeGravity(acc_quat_final_rot);

        // allows for data to be printed/published at lower frequency TODO
        if (count % 1 == 0) {
          printIMUData();
          prepareDataPublishing();
          pub_imu.publish(&msg);
        }

        // shift the data to accomodate for next data set
        shiftData();
      }

      nh.spin(); //TODO right placement?

    }

/* Prints the current data in each array to the serial stream.
 * Prints the orientation, raw angular velocity, acceleration transformed into
 * the world frame with gravity removed, and raw acceleration.
 */
void printIMUData(void)
{
  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.


  float accelX = Quaternion::getX(acc_quat_no_grav);
  float accelY = Quaternion::getY(acc_quat_no_grav);
  float accelZ = Quaternion::getZ(acc_quat_no_grav);
  float gyroX = x_angle[1];
  float gyroY = y_angle[1];
  float gyroZ = z_angle[1];
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);

  Serial.println("Orientation: " + String(gyroX) + " " + String(gyroY) + " " + String(gyroZ) + " degrees");
  Serial.println("GYRO: " + String(x_angular_rate[1]) + "," + String(y_angular_rate[1]) + "," + String(z_angular_rate[1]));
  Serial.println("Accel: " + String(accelX) + ", " + String(accelY) + ", " + String(accelZ) + " g");
  Serial.println("ACCEL: " + String(imu.calcAccel(imu.ax)) + "," + String(imu.calcAccel(imu.ay)) + "," + String(imu.calcAccel(imu.az)));
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();
}

/* Places the data in the sensor_msgs data structure to prepare for publishing.
 */
void prepareDataPublishing(void)
{
  geometry_msgs::Quaternion orient;
  geometry_msgs::Vector3 ang_velo;
  geometry_msgs::Vector3 accel;

  double orient_covar [9];
  double angVelo_covar [9];
  double accel_covar [9];

  Quaternion orient_quat = Quaternion(z_angle[1],x_angle[1],y_angle[1]); // check function
  orient.x = Quaternion::getX(orient_quat);
  orient.y = Quaternion::getY(orient_quat);
  orient.z = Quaternion::getZ(orient_quat);
  orient.w = Quaternion::getW(orient_quat);

  // Angular velocity components
  ang_velo.x = x_angular_rate[1];
  ang_velo.y = y_angular_rate[1];
  ang_velo.z = z_angular_rate[1];

  // Acceleration components (this is in global frame, gravity removed)
  accel.x = Quaternion::getX(acc_quat_no_grav);
  accel.y = Quaternion::getY(acc_quat_no_grav);
  accel.z = Quaternion::getZ(acc_quat_no_grav);

  // covariances for each data group TODO
  orient_covar = {};
  angVelo_covar = {};
  accel_covar = {};

  msg.orientation = orient;
  msg.orientation_covariance = orient_covar;
  msg.angular_velocity = ang_velo;
  msg.angular_velocity_covariance = angVelo_covar;
  msg.linear_acceleration = accel;
  msg.linear_acceleration_covariance = accel_covar;
}

// calibrate the gyroscope at the beginning of sampling, taking an average
// of the angular velocities over an 8 sec period.
// Sensor must remain stillfirstOrderApprox
void calibrateGyro(void)
{
  Serial.println("Sensor must remain stationary for 8 seconds.");
  Serial.println("Gyroscope calibrating...");

  int calibCount = 0;
  double x_gyro_accum = 0;
  double y_gyro_accum = 0;
  double z_gyro_accum = 0;

  double x_accel_accum = 0;
  double y_accel_accum = 0;
  double z_accel_accum = 0;


  // run this loop for 8 seconds
  while(calibCount < imu.getSampleRate() * 8)
  {
    if (imu.dataReady())
    {
      imu.update(UPDATE_ACCEL | UPDATE_GYRO);
      readGyroData();

      x_gyro_accum += x_angular_rate[1];
      y_gyro_accum += y_angular_rate[1];
      z_gyro_accum += z_angular_rate[1];

      readAccelData();
      x_accel_accum += x_acceleration[1];
      y_accel_accum += y_acceleration[1];
      z_accel_accum += z_acceleration[1];

      calibCount += 1;
    }
  }

  // stead state values are the accumulated value divided by number of samples
  ss_gyro_x = x_gyro_accum / calibCount;
  ss_gyro_y = y_gyro_accum / calibCount;
  ss_gyro_z = z_gyro_accum / calibCount;

  ss_accel_x = x_accel_accum / calibCount;
  ss_accel_y = y_accel_accum / calibCount;
  ss_accel_z = z_accel_accum / calibCount;

  double accelMag = pow(ss_accel_x,2) + pow(ss_accel_y,2) + pow(ss_accel_z,2);
   // this gives avg magnitude of accel while stationary (should be 1)
  accelMag = sqrt(accelMag);


  Serial.println("Gyroscope calibration finished.");
  Serial.println("Offset values:");
  Serial.println("X Angular Vel: " + String(ss_gyro_x));
  Serial.println("Y Angular Vel: " + String(ss_gyro_y));
  Serial.println("Z Angular Vel: " + String(ss_gyro_z));
  Serial.println("X Accel: " + String(ss_accel_x));
  Serial.println("Y Accel: " + String(ss_accel_y));
  Serial.println("Z Accel: " + String(ss_accel_z));
  Serial.println("Magnitude of accel:" + String(accelMag));
}


/* Use first-order approximation (trapezoidal) to integrate over this timestep
 * @param  val_prev      integrated value at last timestep
 * @param  val_dot_prev  value at last timestep
 * @param  val_dot_curr  value at current timestep
 * @param  dt            duration of timestep
 * @return current integrated value
 */
double firstOrderApprox(double val_prev, double val_dot_prev, double val_dot_curr, double dt)
{
  double val_current = val_prev + (val_dot_prev + ((val_dot_curr - val_dot_prev)/2))*dt;
  return val_current;
}


// initialize all starting values as 0
void initializeData(void)
{
  x_angular_rate[0] = 0;
  y_angular_rate[0] = 0;
  z_angular_rate[0] = 0;
  x_angle[0] = 0;
  y_angle[0] = 0;
  z_angle[0] = 0;
  x_acceleration[0] = 0;
  y_acceleration[0] = 0;
  z_acceleration[0] = 0;
  x_velocity[0] = 0;
  y_velocity[0] = 0;
  z_velocity[0] = 0;
  x_position[0] = 0;
  y_position[0] = 0;
  z_position[0] = 0;
  ss_gyro_x = 0;
  ss_accel_x = 0;
  ss_accel_y = 0;
  ss_accel_z = 0;
  acc_quat_no_grav = Quaternion(0,0,0,0);
}

// put data from digital input into arrays, current angular velocities
// Direction of the angular velocities follow the orientation of the axes,
// using the right hand rule.
void readGyroData(void) {
  x_angular_rate[1] = imu.calcGyro(imu.gx) - ss_gyro_x;
  y_angular_rate[1] = imu.calcGyro(imu.gy) - ss_gyro_y;
  z_angular_rate[1] = imu.calcGyro(imu.gz) - ss_gyro_z;
}

// put data from digital input into arrays, current accelerations.
// The orientations of the axes follow the right hand rule and is labeled
// on the sensor.
void readAccelData(void) {
  x_acceleration[1] = imu.calcAccel(imu.ax);
  y_acceleration[1] = imu.calcAccel(imu.ay);
  z_acceleration[1] = imu.calcAccel(imu.az);
}

// shift the current data into the old positions in each vector
void shiftData(void) {
  x_angle[0] = x_angle[1];
  y_angle[0] = y_angle[1];
  z_angle[0] = z_angle[1];
  x_angular_rate[0] = x_angular_rate[1];
  y_angular_rate[0] = y_angular_rate[1];
  z_angular_rate[0] = z_angular_rate[1];
  x_acceleration[0] = x_acceleration[1];
  y_acceleration[0] = y_acceleration[1];
  z_acceleration[0] = z_acceleration[1];
  x_velocity[0] = x_velocity[1];
  y_velocity[0] = y_velocity[1];
  z_velocity[0] = z_velocity[1];
  x_position[0] = x_position[1];
  y_position[0] = y_position[1];
  z_position[0] = z_position[1];
}
