#include <xiaomi_bridge/xiaomi_topic_handler.h>

XiaomiTopicHandler::XiaomiTopicHandler()
: Node("xiaomi_bridge_node")
{
  this->declare_parameter("connection_parameters.vacuum_ip", "192.168.8.1");
  this->get_parameter_or<std::string>("connection_parameters.vacuum_ip", vacuum_ip_, std::string("192.168.8.1"));
 
  RCLCPP_INFO(this->get_logger(), "Connecting to Xiaomi Cleaner.");
  // TODO add logic if connection to robot couldn't be established
  player_interface_ = new XiaomiPlayerInterface(vacuum_ip_);
  RCLCPP_INFO(this->get_logger(), "Successfully connected to Xiaomi Cleaner.");

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&XiaomiTopicHandler::cmdVelCallback_, this, _1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
  sonar_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/front_sonar", 10);

  wall_ir_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/wall_distance", 10);

  cliff_fr_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/cliff/front_right", 10);
  cliff_fl_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/cliff/front_left", 10);
  cliff_r_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/cliff/right", 10);
  cliff_l_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/cliff/left", 10);
  
  timer_ = this->create_wall_timer(75ms, std::bind(&XiaomiTopicHandler::publish, this));
}

XiaomiTopicHandler::~XiaomiTopicHandler()
{
  RCLCPP_INFO(this->get_logger(), "Closing connection to Xiaomi Cleaner.");
  player_interface_->cleanup();
  RCLCPP_INFO(this->get_logger(), "Connection to Xiaomi Cleaner closed.");
  delete player_interface_;
}

void XiaomiTopicHandler::publish()
{
  player_interface_->updateRobotState();
  struct irData_t ir_data = player_interface_->getIrSensorData();
  publishWallDistance_(ir_data);
  publishCliffData_(ir_data);

  double sonar_data = player_interface_->getSonarData();
  publishFrontSonar_(sonar_data);

  struct batteryState_t battery_data = player_interface_->getBatteryData();
  publishBatteryState_(battery_data);

  player_interface_->getLaserData(laser_scan_data_);
  publishLaserScan_(laser_scan_data_);

  struct odometryData_t odom_data = player_interface_->getOdometryData();
  publishOdometry_(odom_data);
}

void XiaomiTopicHandler::publishWallDistance_(struct irData_t data)
{
  auto msg = sensor_msgs::msg::Range();
  msg.header.stamp = this->now();
  msg.header.frame_id = "wall_distance_link";
  msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  msg.max_range = 0.055;
  msg.min_range = 0.001;
  msg.field_of_view = 0.1;
  msg.range = data.wall;

  wall_ir_pub_->publish(msg);
}

void XiaomiTopicHandler::publishCliffData_(struct irData_t data)
{

  auto msg = sensor_msgs::msg::Range();
  msg.header.stamp = this->now();
  msg.header.frame_id = "cliff_sensor_fr_link";
  msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  msg.max_range = 1.0;
  msg.min_range = 0.0;
  msg.field_of_view = 0.1;
  msg.range = data.cliff0;
  cliff_fr_pub_->publish(msg);

  msg.header.frame_id = "cliff_sensor_fl_link";
  msg.range = data.cliff1;
  cliff_fl_pub_->publish(msg);

  msg.header.frame_id = "cliff_sensor_r_link";
  msg.range = data.cliff2;
  cliff_r_pub_->publish(msg);

  msg.header.frame_id = "cliff_sensor_l_link";
  msg.range = data.cliff3;
  cliff_l_pub_->publish(msg);

}

void XiaomiTopicHandler::publishFrontSonar_(double data)
{
  auto msg = sensor_msgs::msg::Range();
  msg.header.stamp = this->now();
  msg.header.frame_id = "front_sonar_link";
  msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  msg.max_range = 0.36;
  msg.min_range = 0.0;
  msg.field_of_view = 0.1;
  msg.range = (float)data;

  sonar_pub_->publish(msg);
}

void XiaomiTopicHandler::publishBatteryState_(struct batteryState_t data)
{
  auto msg = sensor_msgs::msg::BatteryState();
  msg.header.stamp = this->now();
  msg.present = 1;
  msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  msg.percentage = (float)data.percentage;

  if(data.charging)
  {
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  } else
  {
    if(data.percentage > 99.0)
    {
      msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
    } else
    {
      msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    }
  }

  battery_pub_->publish(msg);
}

void XiaomiTopicHandler::publishLaserScan_(float *scan_data)
{
  auto msg = sensor_msgs::msg::LaserScan();
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_laser_link";

  msg.angle_max = (float)M_PI;
  msg.angle_min = (float)-M_PI;
  msg.angle_increment = (float)(2 * M_PI) / 360;
  msg.time_increment = (1 / 24) / 360;
  msg.range_max = 10.0;
  msg.range_min = 0.0;

  msg.ranges.assign(scan_data, scan_data + 360);

  laser_pub_->publish(msg);
}

void XiaomiTopicHandler::publishOdometry_(struct odometryData_t data)
{
  auto msg = nav_msgs::msg::Odometry();
  msg.header.stamp = this->now();
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_footprint";

  msg.twist.twist.linear.x = data.vx;
  msg.twist.twist.linear.y = data.vy;
  msg.twist.twist.linear.z = 0.0;
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = data.rot_vy;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, data.rot_py);
    
  msg.pose.pose.position.x = data.px;
  msg.pose.pose.position.y = data.py;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  odom_pub_->publish(msg);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_footprint";
  transform.transform.translation.x = data.px;
  transform.transform.translation.y = data.py;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(transform);
}

void XiaomiTopicHandler::cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
  struct velCmd_t cmd = {
    .px = msg->linear.x,
    .py = msg->linear.y,
    .pz = msg->linear.z,
    .ax = msg->angular.x,
    .ay = msg->angular.y,
    .az = msg->angular.z
  };

  player_interface_->setVelocityCommand(cmd);

}
