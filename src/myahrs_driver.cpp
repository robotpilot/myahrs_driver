//------------------------------------------------------------------------------
// Copyright (c) 2015, Yoonseok Pyo
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of myahrs_driver nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//------------------------------------------------------------------------------
#include <myahrs_driver/myahrs_plus.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

//------------------------------------------------------------------------------
using namespace WithRobot;

//------------------------------------------------------------------------------
static const char* DIVIDER = "1";  // 100 Hz

#define DEG2RAD (M_PI/180.0)
#define RAD2DEG (180.0/M_PI)

//------------------------------------------------------------------------------
void handle_error(const char* error_msg)
{
  fprintf(stderr, "ERROR: %s\n", error_msg);
  exit(1);
}

//------------------------------------------------------------------------------
class MyAhrsDriverForROS : public iMyAhrsPlus
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  void OnSensorData(int sensor_id, SensorData data)
  {
    LockGuard _l(lock);
    sensor_data = data;
    sensor_data.euler_angle = sensor_data.quaternion.to_euler_angle();

    publish_topic(sensor_id);
  }

  void OnAttributeChange(int sensor_id, std::string attribute_name, std::string value)
  {
    printf("OnAttributeChange(id %d, %s, %s)\n", sensor_id, attribute_name.c_str(), value.c_str());
  }

public:
  ros::Publisher data_pub;
  sensor_msgs::Imu imu_data;
  Platform::Mutex lock;
  SensorData sensor_data;
  tf::TransformBroadcaster broadcaster;

  std::string port_;
  int baud_rate_;
  std::string frame_id_;
  bool autocalibrate_;
  double linear_acceleration_stddev_;
  double angular_velocity_stddev_;
  double magnetic_field_stddev_;
  double orientation_stddev_;

  MyAhrsDriverForROS(std::string port="", unsigned int baudrate=115200)
  : iMyAhrsPlus(port, baudrate),
    nh_priv_("~")
  {
    nh_priv_.param("port", port_, std::string("/dev/ttyACM0"));
    nh_priv_.param("baud", baud_rate_, 115200);
    nh_priv_.param("frame_id", frame_id_, std::string("imu_link"));
    nh_priv_.param("autocalibrate", autocalibrate_, false);
    nh_priv_.param("linear_acceleration_stddev", linear_acceleration_stddev_, 0.0);
    nh_priv_.param("angular_velocity_stddev", angular_velocity_stddev_, 0.0);
    nh_priv_.param("magnetic_field_stddev", magnetic_field_stddev_, 0.0);
    nh_priv_.param("orientation_stddev", orientation_stddev_, 0.0);

    data_pub = nh_.advertise<sensor_msgs::Imu>("imu/data", 10);
  }

  ~MyAhrsDriverForROS()
  {}

  bool initialize()
  {
    bool ok = false;

    do
    {
      if(start() == false) break;
      if(cmd_binary_data_format("QUATERNION, IMU") == false) break;
      if(cmd_divider(DIVIDER) == false) break;
      if(cmd_mode("BC") == false) break;
      ok = true;
    } while(0);

    return ok;
  }

  inline void get_data(SensorData& data)
  {
    LockGuard _l(lock);
    data = sensor_data;
  }

  inline SensorData get_data()
  {
    LockGuard _l(lock);
    return sensor_data;
  }

  void publish_topic(int sensor_id)
  {
    printf(".");

    Quaternion& q = sensor_data.quaternion;
    EulerAngle& e = sensor_data.euler_angle;
    ImuData<float>& imu = sensor_data.imu;

    printf("sensor_id %d, Quaternion(xyzw)=%.4f,%.4f,%.4f,%.4f, Angle(rpy)=%.1f, %.1f, %.1f, Accel(xyz)=%.4f,%.4f,%.4f, Gyro(xyz)=%.4f,%.4f,%.4f, Magnet(xyz)=%.2f,%.2f,%.2f\n",
      sensor_id,
      q.x, q.y, q.z, q.w,
      e.roll, e.pitch, e.yaw,
      imu.ax, imu.ay, imu.az,
      imu.gx, imu.gy, imu.gz,
      imu.mx, imu.my, imu.mz);

    ros::Time now = ros::Time::now();
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = now;

    imu_msg.header.frame_id = "imu_base";

    imu_msg.orientation.x = q.x;
    imu_msg.orientation.y = q.y;
    imu_msg.orientation.z = q.z;
    imu_msg.orientation.w = q.w;

    imu_msg.angular_velocity.x = imu.gx * DEG2RAD;
    imu_msg.angular_velocity.y = imu.gy * DEG2RAD;
    imu_msg.angular_velocity.z = imu.gz * DEG2RAD;

    imu_msg.linear_acceleration.x = imu.ax * 9.80665;
    imu_msg.linear_acceleration.y = imu.ay * 9.80665;
    imu_msg.linear_acceleration.z = imu.az * 9.80665;

    data_pub.publish(imu_msg);

    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(e.roll*DEG2RAD, -e.pitch*DEG2RAD, -e.yaw*DEG2RAD),
                                                                 tf::Vector3(0.0, 0.0, 0.1)),
                                                   ros::Time::now(), "imu_base", "imu"));
  }
};


//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "myahrs_driver");

  MyAhrsDriverForROS sensor("/dev/ttyACM0", 115200);

  if(sensor.initialize() == false)
  {
    handle_error("initialize() returns false");
  }

  ros::Rate loop_rate(100); // 0.01sec

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

//------------------------------------------------------------------------------
