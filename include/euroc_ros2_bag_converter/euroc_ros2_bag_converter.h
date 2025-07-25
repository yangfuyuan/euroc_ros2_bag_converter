#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <builtin_interfaces/msg/time.hpp>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>

namespace fs = std::filesystem;

template <typename T, typename NodeT>
T readParam(NodeT node, const std::string& name, T default_value = T()) {
  T value;
  node->template declare_parameter<T>(name, default_value);
  if (!node->template get_parameter<T>(name, value)) {
    throw std::runtime_error("Parameter '" + name + "' not found.");
  }
  return value;
}

inline std::string expand_user_home(const std::string& path) {
  if (!path.empty() && path[0] == '~') {
    const char* home = std::getenv("HOME");
    if (home) {
      return std::string(home) + path.substr(1);  // skip the '~'
    }
  }
  return path;
}

class EurocROS2BagConverterNode : public rclcpp::Node {
 public:
  EurocROS2BagConverterNode();
  bool initialize();
  void convertToBag();

 private:
  struct SensorMsg {
    builtin_interfaces::msg::Time stamp;
    std::string topic;
    std::shared_ptr<rclcpp::SerializedMessage> msg;
  };

  void loadImuDataFromCsv(const std::string& imu_file);

  void loadImagesFromFolder(const std::string& image_dir,
                            const std::string& topic_prefix);
  void loadGroundTruthFromCsv(const std::string& gt_file);

  void loadCameraInfoFromYaml(const std::string& topic,
                              const std::string& yaml_path);

  template <typename MsgT>
  void addSerializedMsg(const std::string& topic, const MsgT& msg) {
    rclcpp::Serialization<MsgT> serializer;
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    serializer.serialize_message(&msg, serialized_msg.get());
    SensorMsg sm;
    sm.stamp = msg.header.stamp;
    sm.topic = topic;
    sm.msg = serialized_msg;
    all_msgs_.push_back(sm);
  }
  void writeMessagesToBag();
  // 放在类外或类成员中
  const std::vector<std::pair<std::string, std::string>> kTopicTypePatterns = {
      {"image_raw", "sensor_msgs/msg/Image"},
      {"imu0", "sensor_msgs/msg/Imu"},
      {"odom", "nav_msgs/msg/Odometry"},
      {"path", "nav_msgs/msg/Path"},
      {"camera_info", "sensor_msgs/msg/CameraInfo"},
  };
  std::string inferMessageType(const std::string& topic_name) {
    for (const auto& [pattern, type] : kTopicTypePatterns) {
      if (topic_name.find(pattern) != std::string::npos) {
        return type;
      }
    }
    return "";  // Unknown
  }

  std::string euroc_root_;
  std::string output_bag_path_;
  std::vector<SensorMsg> all_msgs_;
  nav_msgs::msg::Path groundtruth_path_;
  std::map<std::string, sensor_msgs::msg::CameraInfo> camera_info_map_;
  rosbag2_cpp::writers::SequentialWriter writer;
};
