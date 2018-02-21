#include <xiaomi_bridge/xiaomi_topic_handler.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xiaomi_bridge_node");
  ros::NodeHandle nh("~");

  XiaomiTopicHandler *handler = new XiaomiTopicHandler(nh);
  handler->run();

  return 0;
}