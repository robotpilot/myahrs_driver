#include <myahrs_ros2_driver/myahrs_ros2_driver.hpp>
#include <memory>


int main(int argc, char** argv){

    rclcpp::init(argc, argv);

    std::string port = std::string("/dev/ttyACM0");
    int baud_rate    = 115200;

    rclcpp::spin(std::make_shared<WithRobot::MyAhrsDriverForROS>(port,baud_rate));
    rclcpp::shutdown();


    return 0;
}