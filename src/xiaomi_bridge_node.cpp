#include <xiaomi_bridge/xiaomi_topic_handler.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XiaomiTopicHandler>());
  rclcpp::shutdown();
  return 0;
}
