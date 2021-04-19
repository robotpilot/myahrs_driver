//------------------------------------------------------------------------------
// Copyright (c) 2015, Yoonseok Pyo
// All rights reserved.

// License: BSD

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

#ifndef __MYAHRS_ROS2_H__
#define __MYAHRS_ROS2_H__

#include <myahrs_ros2_driver/myahrs_plus.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/float64.hpp>
#include <unistd.h>


using std::placeholders::_1;

//------------------------------------------------------------------------------
namespace WithRobot{

//------------------------------------------------------------------------------
class MyAhrsDriverForROS : public rclcpp::Node, iMyAhrsPlus{
  public:
    MyAhrsDriverForROS(std::string port, int baud_rate);

    ~MyAhrsDriverForROS();


    bool initialize();

    inline void get_data(SensorData& data)
    {
      LockGuard _l(lock_);
      data = sensor_data_;
    }

    inline SensorData get_data()
    {
      LockGuard _l(lock_);
      return sensor_data_;
    }

    void publish_topic(int sensor_id);

    tf2::Quaternion Euler2Quaternion(float roll, float pitch, float yaw);


  private:

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_raw_pub_, imu_data_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr imu_mag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr imu_temperature_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;


    Platform::Mutex lock_;
    SensorData sensor_data_;
    bool publish_tf_;
    std::string parent_frame_id_;
    std::string frame_id_;
    double linear_acceleration_stddev_;
    double angular_velocity_stddev_;
    double magnetic_field_stddev_;
    double orientation_stddev_;


    void OnSensorData(int sensor_id, SensorData data);

    void OnAttributeChange(int sensor_id, std::string attribute_name, std::string value);

};
};
#endif //__MYAHRS_ROS2_H__