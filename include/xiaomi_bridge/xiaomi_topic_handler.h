#ifndef XIAOMI_BRIDGE_XIAOMI_TOPIC_HANDLER_H
#define XIAOMI_BRIDGE_XIAOMI_TOPIC_HANDLER_H

#include <math.h>
#include <string>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <xiaomi_bridge/xiaomi_player_interface.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class XiaomiTopicHandler : public rclcpp::Node
{
public:
  XiaomiTopicHandler();
  ~XiaomiTopicHandler();

  void publish();

private:
  void cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg) const;
  void publishBatteryState_(struct batteryState_t);
  void publishOdometry_(struct odometryData_t);
  void publishWallDistance_(struct irData_t);
  void publishCliffData_(struct irData_t);
  void publishFrontSonar_(double);
  void publishLaserScan_(float*);

  XiaomiPlayerInterface *player_interface_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub_;

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr wall_ir_pub_;

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr cliff_fr_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr cliff_fl_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr cliff_r_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr cliff_l_pub_;

  std::string vacuum_ip_;
  float laser_scan_data_[360];

};

#endif //XIAOMI_BRIDGE_XIAOMI_TOPIC_HANDLER_H
