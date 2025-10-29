#include "rr_buffer_services/rr_controller.hpp"

using namespace rr_buffer_services;

void RrController::init_state()
{
  pluginlib::ClassLoader<RR_STATE_MAINTAINER_CLASS> poly_loader(RR_COMMON_BASE, RR_STATE_MAINTAINER);
  state_ = poly_loader.createSharedInstance(RR_DEFAULT_STATE_MAINTAINER);
}

void RrController::init_factory()
{
  this->declare_parameter<std::string>(RR_CLI_ROBOT_TYPE, RR_DEFAULT_ROBOT_TYPE);
  // load factory base class.
  pluginlib::ClassLoader<RR_STATE_BUF_FACTORY_CLASS> poly_loader(RR_COMMON_BASE, RR_STATE_BUF_FACTORY);

  RCLCPP_INFO(this->get_logger(), "loading robot '%s'", this->get_parameter(RR_CLI_ROBOT_TYPE).as_string().c_str());
  factory_ = poly_loader.createSharedInstance(this->get_parameter(RR_CLI_ROBOT_TYPE).as_string());

  RCLCPP_INFO(this->get_logger(), "initializing robot '%s'", this->get_parameter(RR_CLI_ROBOT_TYPE).as_string().c_str());
  factory_->initialize(shared_from_this(), state_);

  RCLCPP_INFO(this->get_logger(), "retrieving callback group for image stream");
  sub_img_group_ = factory_->sub_img_group();
}

void RrController::init()
{
  try {
    RCLCPP_INFO(this->get_logger(), "loading plugins");
    init_state();
    init_factory();

    RCLCPP_INFO(this->get_logger(), "creating publisher");
    // create a separate callback group,  may not be required, but want to separate this from the subscription
    // group.
    publisher_ = this->create_publisher<rr_interfaces::msg::BufferResponse>(RR_BUF_TOPIC, RR_QUEUE_LIMIT);
    publish_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto timer_callback = std::bind(&RrController::callback, this);

    // ticks should be tunable.
    std::chrono::duration<int, std::milli> ticks(RR_TOPIC_TICK);
    timer_ = this->create_wall_timer(ticks, timer_callback, publish_group_);
  }
  catch (pluginlib::PluginlibException &ex) {
    RCLCPP_FATAL(this->get_logger(), "could not load plugins, failed on the following: %s", ex.what());
  }
}


std::string RrController::uuid_to_string(const unique_identifier_msgs::msg::UUID &uuid_msg)
{
  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  const auto &bytes = uuid_msg.uuid;

  // 16 bytes in hex, with hyphens after bytes 4, 6, 8, 10
  for (size_t i = 0; i < bytes.size(); ++i) {
    ss << std::setw(2) << static_cast<int>(bytes[i]);
    if (i == 3 || i == 5 || i == 7 || i == 9) {
      ss << "-";
    }
  }
  return ss.str();
}

/**
 * @fn callback
 * 
 * @brief look for all aspects of the frame and publish it for the normalizer.
 */
void RrController::callback()
{
  RCLCPP_DEBUG(this->get_logger(), "called publisher");
  boost::uuids::uuid boost_uuid = boost::uuids::random_generator()();
  unique_identifier_msgs::msg::UUID uuid_msg;
  std::copy(boost_uuid.begin(), boost_uuid.end(), uuid_msg.uuid.begin());

  rr_interfaces::msg::BufferResponse msg_resp;

  // note that these attributes can change while being read, so very time critical
  // things should be done last.
  if (state_->has_batt_state()) {
    msg_resp.batt_state = state_->get_batt_state();
  }

  if (state_->has_gps()) {
    msg_resp.gps = state_->get_gps();
  }
  if (state_->has_joystick()) {
    msg_resp.joystick = state_->get_joystick();
  }

  if (state_->has_imu()) {
    msg_resp.imu = state_->get_imu();
  }

  // asside from range sensors image is the most crucual for object detection.
  if (state_->has_image()) {
    msg_resp.img = state_->get_image();
  }

  // note range sensors are distinguished by frame_link, so they are just bundled together.
  if (state_->has_ranges()) {
    msg_resp.ranges = state_->get_ranges();
  }

  // publish the message.
  msg_resp.guid = uuid_msg;
  msg_resp.exec_time = this->now();
  this->publisher_->publish(msg_resp);

  // For speed this can be disabled,  translating UUID to string can be expensive.
  // if (this->get_logger().get_level() == rclcpp::Logger::Level::Debug) {
    RCLCPP_DEBUG(this->get_logger(), "published: %s", uuid_to_string(uuid_msg).c_str());
  // }
}