#include <euroc_ros2_bag_converter/euroc_ros2_bag_converter.h>
static builtin_interfaces::msg::Time toMsgTime(uint64_t t_ns) {
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(t_ns / 1000000000ULL);
  t.nanosec = static_cast<uint32_t>(t_ns % 1000000000ULL);
  return t;
}

EurocROS2BagConverterNode::EurocROS2BagConverterNode()
    : rclcpp::Node("euroc_ros2_bag_converter"),
      euroc_root_("."),
      output_bag_path_(".") {
  if (initialize()) {
    convertToBag();
  }
}

bool EurocROS2BagConverterNode::initialize() {
  euroc_root_ = readParam<std::string>(this, "euroc_root", ".");
  euroc_root_ = expand_user_home(euroc_root_);
  output_bag_path_ = readParam<std::string>(this, "bag_path", ".");
  output_bag_path_ = expand_user_home(output_bag_path_);

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = output_bag_path_;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options{"", ""};
  try {
    writer.open(storage_options, converter_options);
  } catch (std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
                 "[euroc2bag] Failed to open bag: %s. Reason: %s",
                 output_bag_path_.c_str(), e.what());
    return false;
  }
  return true;
}
void EurocROS2BagConverterNode::convertToBag() {
  loadImuDataFromCsv(euroc_root_ + "/mav0/imu0/data.csv");
  loadImagesFromFolder(euroc_root_ + "/mav0/cam0/data", "/cam0/image_raw");
  loadImagesFromFolder(euroc_root_ + "/mav0/cam1/data", "/cam1/image_raw");
  loadGroundTruthFromCsv(euroc_root_ +
                         "/mav0/state_groundtruth_estimate0/data.csv");
  loadCameraInfoFromYaml("/cam0/image_raw",
                         euroc_root_ + "/mav0/cam0/sensor.yaml");
  loadCameraInfoFromYaml("/cam1/image_raw",
                         euroc_root_ + "/mav0/cam1/sensor.yaml");

  writeMessagesToBag();
}

void EurocROS2BagConverterNode::loadImuDataFromCsv(
    const std::string& imu_file) {
  RCLCPP_INFO(this->get_logger(), "[euroc2bag] Loading IMU data from: %s",
              imu_file.c_str());
  std::ifstream fin(imu_file);
  if (!fin.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "[euroc2bag] Failed to open IMU file: %s",
                 imu_file.c_str());
    return;
  }
  std::string line;
  std::getline(fin, line);  // skip header
  size_t count = 0;
  while (std::getline(fin, line)) {
    std::stringstream ss(line);
    std::string field;
    std::vector<std::string> fields;

    while (std::getline(ss, field, ',')) fields.push_back(field);
    if (fields.size() != 7) continue;

    auto imu_msg = sensor_msgs::msg::Imu();
    int64_t t_ns = std::stoll(fields[0]);
    imu_msg.header.stamp = toMsgTime(t_ns);
    imu_msg.header.frame_id = "imu";

    imu_msg.angular_velocity.x = std::stod(fields[1]);
    imu_msg.angular_velocity.y = std::stod(fields[2]);
    imu_msg.angular_velocity.z = std::stod(fields[3]);

    imu_msg.linear_acceleration.x = std::stod(fields[4]);
    imu_msg.linear_acceleration.y = std::stod(fields[5]);
    imu_msg.linear_acceleration.z = std::stod(fields[6]);

    addSerializedMsg("/imu0", imu_msg);
    ++count;
  }
  RCLCPP_INFO(this->get_logger(), "[euroc2bag] Loaded %lu IMU messages", count);
}

void EurocROS2BagConverterNode::loadImagesFromFolder(
    const std::string& image_dir, const std::string& topic_prefix) {
  RCLCPP_INFO(this->get_logger(), "[euroc2bag] Loading camera images from: %s",
              image_dir.c_str());
  size_t count = 0;
  for (const auto& entry : fs::directory_iterator(image_dir)) {
    std::string filename = entry.path().filename().string();
    if (entry.path().extension() != ".png" &&
        entry.path().extension() != ".jpg") {
      RCLCPP_WARN(this->get_logger(),
                  "[euroc2bag] Skipping unsupported image file: %s",
                  filename.c_str());
      continue;
    }

    std::string timestamp_str = filename.substr(0, filename.find('.'));
    int64_t t_ns = std::stoll(timestamp_str);
    auto img = cv::imread(entry.path().string(), cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      RCLCPP_WARN(this->get_logger(), "[euroc2bag] Failed to load image: %s",
                  entry.path().string().c_str());

      continue;
    }
    std_msgs::msg::Header header;
    header.stamp = toMsgTime(t_ns);
    header.frame_id = topic_prefix.substr(1, 4);
    auto msg = cv_bridge::CvImage(header, "mono8", img).toImageMsg();

    addSerializedMsg(topic_prefix, *msg);
    ++count;

    if (camera_info_map_.count(topic_prefix)) {
      auto cam_info = camera_info_map_[topic_prefix];
      cam_info.header = msg->header;
      addSerializedMsg(topic_prefix + "/camera_info", cam_info);
    }
  }
  RCLCPP_INFO(this->get_logger(), "[euroc2bag] Loaded %lu images from %s",
              count, topic_prefix.c_str());
}

void EurocROS2BagConverterNode::loadGroundTruthFromCsv(
    const std::string& gt_file) {
  RCLCPP_INFO(this->get_logger(), "[euroc2bag] Loading ground truth from: %s",
              gt_file.c_str());
  std::ifstream fin(gt_file);
  if (!fin.is_open()) {
    RCLCPP_ERROR(this->get_logger(),
                 "[euroc2bag] Failed to open ground truth file: %s",
                 gt_file.c_str());
    return;
  }
  std::string line;
  std::getline(fin, line);  // skip header
  size_t count = 0;
  while (std::getline(fin, line)) {
    std::stringstream ss(line);
    std::string field;
    std::vector<std::string> fields;

    while (std::getline(ss, field, ',')) fields.push_back(field);
    if (fields.size() != 17) continue;

    int64_t t_ns = std::stoll(fields[0]);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = toMsgTime(t_ns);
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "imu";

    tf_msg.transform.translation.x = std::stod(fields[1]);
    tf_msg.transform.translation.y = std::stod(fields[2]);
    tf_msg.transform.translation.z = std::stod(fields[3]);
    tf_msg.transform.rotation.w = std::stod(fields[4]);
    tf_msg.transform.rotation.x = std::stod(fields[5]);
    tf_msg.transform.rotation.y = std::stod(fields[6]);
    tf_msg.transform.rotation.z = std::stod(fields[7]);

    nav_msgs::msg::Odometry odom;
    odom.header = tf_msg.header;
    odom.child_frame_id = tf_msg.child_frame_id;
    odom.pose.pose.position.x = tf_msg.transform.translation.x;
    odom.pose.pose.position.y = tf_msg.transform.translation.y;
    odom.pose.pose.position.z = tf_msg.transform.translation.z;
    odom.pose.pose.orientation = tf_msg.transform.rotation;
    addSerializedMsg("/groundtruth/odom", odom);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = tf_msg.header;
    pose.pose.position.x = tf_msg.transform.translation.x;
    pose.pose.position.y = tf_msg.transform.translation.y;
    pose.pose.position.z = tf_msg.transform.translation.z;
    pose.pose.orientation = tf_msg.transform.rotation;

    if (groundtruth_path_.header.frame_id.empty()) {
      groundtruth_path_.header = tf_msg.header;
    }
    groundtruth_path_.poses.push_back(pose);
    ++count;
  }
  addSerializedMsg("/groundtruth/path", groundtruth_path_);
  RCLCPP_INFO(this->get_logger(), "[euroc2bag] Loaded %lu ground truth entries",
              count);
}

void EurocROS2BagConverterNode::loadCameraInfoFromYaml(
    const std::string& topic, const std::string& yaml_path) {
  YAML::Node node = YAML::LoadFile(yaml_path);
  auto& cam = camera_info_map_[topic];

  cam.header.frame_id = topic.substr(1);
  cam.height = node["resolution"][1].as<int>();
  cam.width = node["resolution"][0].as<int>();
  auto intrinsics = node["intrinsics"];
  cam.k = {intrinsics[0].as<double>(),
           0,
           intrinsics[2].as<double>(),
           0,
           intrinsics[1].as<double>(),
           intrinsics[3].as<double>(),
           0,
           0,
           1};
  cam.d = {0, 0, 0, 0, 0};
  cam.distortion_model = "plumb_bob";
}

void EurocROS2BagConverterNode::writeMessagesToBag() {
  RCLCPP_INFO(this->get_logger(), "[euroc2bag] Writing to bag: %s",
              output_bag_path_.c_str());

  // Register all topics
  std::set<std::string> topics;
  for (const auto& sm : all_msgs_) topics.insert(sm.topic);
  for (const auto& topic : topics) {
    std::string type_name = inferMessageType(topic);
    if (type_name.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "[euroc2bag] Unknown topic type for %s, skipping",
                  topic.c_str());
      continue;
    }
    rosbag2_storage::TopicMetadata topic_metadata;
    topic_metadata.name = topic;
    topic_metadata.type = type_name;
    topic_metadata.serialization_format =
        rmw_get_serialization_format();        // usually "cdr"
    topic_metadata.offered_qos_profiles = "";  // optional

    writer.create_topic(topic_metadata);

    RCLCPP_INFO(this->get_logger(),
                "[euroc2bag] Created topic: %s with type %s", topic.c_str(),
                type_name.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "[euroc2bag] Total messages to write: %lu",
              all_msgs_.size());

  std::sort(all_msgs_.begin(), all_msgs_.end(),
            [](const auto& a, const auto& b) {
              if (a.stamp.sec == b.stamp.sec)
                return a.stamp.nanosec < b.stamp.nanosec;
              return a.stamp.sec < b.stamp.sec;
            });

  for (const auto& sm : all_msgs_) {
    auto serialized_msg =
        std::make_shared<rosbag2_storage::SerializedBagMessage>();

    serialized_msg->time_stamp = rclcpp::Time(sm.stamp).nanoseconds();
    serialized_msg->topic_name = sm.topic;
    serialized_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>(
        sm.msg->get_rcl_serialized_message());
    writer.write(serialized_msg);
  }
  RCLCPP_INFO(this->get_logger(), "[euroc2bag] Bag writing complete.");
}
