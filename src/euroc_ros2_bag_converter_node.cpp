#include <euroc_ros2_bag_converter/euroc_ros2_bag_converter.h>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EurocROS2BagConverterNode>();
  rclcpp::shutdown();
  return 0;
}
