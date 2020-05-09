#include <xiaomi_bridge/xiaomi_topic_handler.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  XiaomiTopicHandler *handle = new XiaomiTopicHandler();
  handle->run();

  delete handle;
  rclcpp::shutdown();
  return 0;
}
