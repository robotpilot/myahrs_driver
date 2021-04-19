// Copyright 2021 clobot
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <myahrs_ros2_driver/myahrs_ros2_driver.hpp>

using namespace WithRobot;

MyAhrsDriverForROS::MyAhrsDriverForROS(std::string port, int baud_rate):iMyAhrsPlus(port, baud_rate),Node("myahrs_ros2")
{
    // dependent on user device
    publish_tf_ = false;
    frame_id_ = "imu_link";
    parent_frame_id_ = "base_link";
    linear_acceleration_stddev_ = 0.026831;
    angular_velocity_stddev_ = 0.002428;
    magnetic_field_stddev_ = 0.00000327486;
    orientation_stddev_ = 0.002143;
    

    // publisher for streaming
    imu_data_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1);
    imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);
    imu_mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 1);
    imu_temperature_pub_ = this->create_publisher<std_msgs::msg::Float64>("imu/temperature", 1);
 
 
   if(initialize() == false)
  {
    std::cout <<"Initialize() returns false, please check your devices."<<std::endl;
    exit(0);
  }
  else
  {
    std::cout <<"Initialization OK!\n"<<std::endl;
  }

 }

MyAhrsDriverForROS::~MyAhrsDriverForROS(){

}

bool MyAhrsDriverForROS::initialize(){
    bool ok = false;

    do
    {
      if(start() == false) break;
      //Euler angle(x, y, z axis)
      //IMU(linear_acceleration, angular_velocity, magnetic_field)
      if(cmd_binary_data_format("EULER, IMU") == false) break;
      // 100Hz
      if(cmd_divider("1") == false) break;
      // Binary and Continue modeimu_data_raw_pub_
      if(cmd_mode("BC") == false) break;
      ok = true;
    } while(0);

    return ok;
}


void MyAhrsDriverForROS::publish_topic(int sensor_id){

    auto imu_data_raw_msg = sensor_msgs::msg::Imu();
    auto imu_data_msg = sensor_msgs::msg::Imu();
    auto imu_magnetic_msg = sensor_msgs::msg::MagneticField();
    auto imu_temperature_msg = std_msgs::msg::Float64();

    double linear_acceleration_cov = linear_acceleration_stddev_ * linear_acceleration_stddev_;
    double angular_velocity_cov    = angular_velocity_stddev_ * angular_velocity_stddev_;
    double magnetic_field_cov      = magnetic_field_stddev_ * magnetic_field_stddev_;
    double orientation_cov         = orientation_stddev_ * orientation_stddev_;

    imu_data_raw_msg.linear_acceleration_covariance[0] =
    imu_data_raw_msg.linear_acceleration_covariance[4] =
    imu_data_raw_msg.linear_acceleration_covariance[8] =
    imu_data_msg.linear_acceleration_covariance[0] =
    imu_data_msg.linear_acceleration_covariance[4] =
    imu_data_msg.linear_acceleration_covariance[8] = linear_acceleration_cov;

    imu_data_raw_msg.angular_velocity_covariance[0] =
    imu_data_raw_msg.angular_velocity_covariance[4] =
    imu_data_raw_msg.angular_velocity_covariance[8] =
    imu_data_msg.angular_velocity_covariance[0] =
    imu_data_msg.angular_velocity_covariance[4] =
    imu_data_msg.angular_velocity_covariance[8] = angular_velocity_cov;

    imu_data_msg.orientation_covariance[0] =
    imu_data_msg.orientation_covariance[4] =
    imu_data_msg.orientation_covariance[8] = orientation_cov;

    imu_magnetic_msg.magnetic_field_covariance[0] =
    imu_magnetic_msg.magnetic_field_covariance[4] =
    imu_magnetic_msg.magnetic_field_covariance[8] = magnetic_field_cov;

    static double convertor_g2a  = 9.80665;    // for linear_acceleration (g to m/s^2)
    static double convertor_d2r  = M_PI/180.0; // for angular_velocity (degree to radian)
    static double convertor_r2d  = 180.0/M_PI; // for easy understanding (radian to degree)
    static double convertor_ut2t = 1000000;    // for magnetic_field (uT to Tesla)
    static double convertor_c    = 1.0;        // for temperature (celsius)

    double roll, pitch, yaw;

    // original sensor data used the degree unit, convert to radian (see ROS REP103)
    // we used the ROS's axes orientation like x forward, y left and z up
    // so changed the y and z aixs of myAHRS+ board
    roll  =  sensor_data_.euler_angle.roll*convertor_d2r;
    pitch = -sensor_data_.euler_angle.pitch*convertor_d2r;
    yaw   = -sensor_data_.euler_angle.yaw*convertor_d2r;

    double ang_roll, ang_pitch, ang_yaw;
    ang_roll = roll * 180/3.141592;
    ang_pitch = pitch * 180/3.141592;
    ang_yaw = yaw * 180/3.141592;
    
    // std::cout <<"roll : "<<ang_roll<<", pitch : "<<ang_pitch<<", yaw : "<<ang_yaw<<std::endl;

    ImuData<float>& imu = sensor_data_.imu;

    tf2::Quaternion tf_orientation = Euler2Quaternion(roll,pitch,yaw);

    rclcpp::Time now = this->now();

    imu_data_raw_msg.header.stamp =
    imu_data_msg.header.stamp     =
    imu_magnetic_msg.header.stamp = now;

    imu_data_raw_msg.header.frame_id =
    imu_data_msg.header.frame_id     =
    imu_magnetic_msg.header.frame_id = frame_id_;

    // orientation
    imu_data_msg.orientation.x = tf_orientation.x();
    imu_data_msg.orientation.y = tf_orientation.y();
    imu_data_msg.orientation.z = tf_orientation.z();
    imu_data_msg.orientation.w = tf_orientation.w();

    // original data used the g unit, convert to m/s^2
    imu_data_raw_msg.linear_acceleration.x =
    imu_data_msg.linear_acceleration.x     =  imu.ax * convertor_g2a;
    imu_data_raw_msg.linear_acceleration.y =
    imu_data_msg.linear_acceleration.y     = -imu.ay * convertor_g2a;
    imu_data_raw_msg.linear_acceleration.z =
    imu_data_msg.linear_acceleration.z     = -imu.az * convertor_g2a;

    // original data used the degree/s unit, convert to radian/s
    imu_data_raw_msg.angular_velocity.x =
    imu_data_msg.angular_velocity.x     =  imu.gx * convertor_d2r;
    imu_data_raw_msg.angular_velocity.y =
    imu_data_msg.angular_velocity.y     = -imu.gy * convertor_d2r;
    imu_data_raw_msg.angular_velocity.z =
    imu_data_msg.angular_velocity.z     = -imu.gz * convertor_d2r;

    // original data used the uTesla unit, convert to Tesla
    imu_magnetic_msg.magnetic_field.x =  imu.mx / convertor_ut2t;
    imu_magnetic_msg.magnetic_field.y = -imu.my / convertor_ut2t;
    imu_magnetic_msg.magnetic_field.z = -imu.mz / convertor_ut2t;

    // original data used the celsius unit
    imu_temperature_msg.data = imu.temperature;

    // publish the IMU data
    imu_data_raw_pub_->publish(std::move(imu_data_raw_msg));
    imu_data_pub_->publish(std::move(imu_data_msg));
    imu_mag_pub_->publish(std::move(imu_magnetic_msg));
    imu_temperature_pub_->publish(std::move(imu_temperature_msg));

    // publish tf
    if(publish_tf_)
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now;
      tf.header.frame_id = parent_frame_id_;
      tf.child_frame_id = frame_id_;
      tf.transform.translation.x = 0.0;
      tf.transform.translation.y = 0.0;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = imu_data_msg.orientation;

      broadcaster_->sendTransform(tf);
    }
}



void MyAhrsDriverForROS::OnSensorData(int sensor_id, SensorData data)
{
  LockGuard _l(lock_);
  sensor_data_ = data;
  publish_topic(sensor_id);
}

void MyAhrsDriverForROS::OnAttributeChange(int sensor_id, std::string attribute_name, std::string value)
{
  printf("OnAttributeChange(id %d, %s, %s)\n", sensor_id, attribute_name.c_str(), value.c_str());
}


tf2::Quaternion MyAhrsDriverForROS::Euler2Quaternion(float roll, float pitch, float yaw)
{

    float qx = (sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) - (cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2));
    float qy = (cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2)) + (sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2));
    float qz = (cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2)) - (sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2));
    float qw = (cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) + (sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2));

    tf2::Quaternion q(qx, qy, qz, qw);

    return q;
}
