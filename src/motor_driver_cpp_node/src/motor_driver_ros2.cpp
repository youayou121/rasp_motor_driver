#include"motor_driver_cpp_node/motor_driver_ros2.h"
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MotorDriverROS> node = std::make_shared<MotorDriverROS>("motor_driver_cpp_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}