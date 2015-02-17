#include <myahrs_driver/myahrs_driver.hpp>

int main(int argc, char **argv)
{
  //Init ROS node
  ros::init(argc, argv, "myahrs_driver");
  ros::spin();
  return 0;
}
