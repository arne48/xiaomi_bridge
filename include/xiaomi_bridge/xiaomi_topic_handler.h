#ifndef XIAOMI_BRIDGE_XIAOMI_TOPIC_HANDLER_H
#define XIAOMI_BRIDGE_XIAOMI_TOPIC_HANDLER_H

#include <math.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

#include <xiaomi_bridge/xiaomi_player_interface.h>

class XiaomiTopicHandler
{
public:
  XiaomiTopicHandler();
  XiaomiTopicHandler(ros::NodeHandle nh);
  ~XiaomiTopicHandler();

  void run();

private:
  void cmdVelCallback_(const geometry_msgs::Twist::ConstPtr&);
  void publishWallDistance_(struct irData_t);
  void publishCliffData_(struct irData_t);
  void publishFrontSonar_(double);
  void publishBatteryState_(struct batteryState_t);
  void publishLaserScan_(float*);
  void publishOdometry_(struct odometryData_t);

  ros::NodeHandle nh_;
  XiaomiPlayerInterface *player_interface_;
  ros::Subscriber cmd_vel_sub_;
  tf::TransformBroadcaster tf_broadcaster_;

  ros::Publisher odom_pub_;
  ros::Publisher laser_pub_;
  ros::Publisher battery_pub_;
  ros::Publisher sonar_pub_;

  ros::Publisher wall_ir_pub_;

  ros::Publisher cliff_0_pub_;
  ros::Publisher cliff_1_pub_;
  ros::Publisher cliff_2_pub_;
  ros::Publisher cliff_3_pub_;

};

#endif //XIAOMI_BRIDGE_XIAOMI_TOPIC_HANDLER_H
