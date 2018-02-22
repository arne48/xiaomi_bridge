#include <xiaomi_bridge/xiaomi_topic_handler.h>

XiaomiTopicHandler::XiaomiTopicHandler(ros::NodeHandle nh)
{
  nh_ = nh;
  nh_.param<std::string>("vacuum_ip", vacuum_ip_, "192.168.8.1");
  ROS_INFO("Connecting to Xiaomi Cleaner.");
  player_interface_ = new XiaomiPlayerInterface(vacuum_ip_);
  ROS_INFO("Successfully connected to Xiaomi Cleaner.");
}

XiaomiTopicHandler::~XiaomiTopicHandler()
{
  delete player_interface_;
}

void XiaomiTopicHandler::run()
{
  cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &XiaomiTopicHandler::cmdVelCallback_, this);

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
  laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 10);
  battery_pub_= nh_.advertise<sensor_msgs::BatteryState>("/battery_state", 10);
  sonar_pub_= nh_.advertise<sensor_msgs::Range>("/front_sonar", 10);

  wall_ir_pub_= nh_.advertise<sensor_msgs::Range>("/wall_distance", 10);

  cliff_fr_pub_= nh_.advertise<sensor_msgs::Range>("/cliff/front_right", 10);
  cliff_fl_pub_= nh_.advertise<sensor_msgs::Range>("/cliff/front_left", 10);
  cliff_r_pub_= nh_.advertise<sensor_msgs::Range>("/cliff/right", 10);
  cliff_l_pub_= nh_.advertise<sensor_msgs::Range>("/cliff/left", 10);


  ros::Rate loop_rate(20);
  float laser_scan_data[360] = {0};
  while (ros::ok())
  {
    player_interface_->updateRobotState();
    struct irData_t ir_data = player_interface_->getIrSensorData();
    publishWallDistance_(ir_data);
    publishCliffData_(ir_data);

    double sonar_data = player_interface_->getSonarData();
    publishFrontSonar_(sonar_data);

    struct batteryState_t battery_data = player_interface_->getBatteryData();
    publishBatteryState_(battery_data);

    player_interface_->getLaserData(laser_scan_data);
    publishLaserScan_(laser_scan_data);

    struct odometryData_t odom_data = player_interface_->getOdometryData();
    publishOdometry_(odom_data);

    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout<<"Closing connection to Xiaomi Cleaner."<<std::endl;
  player_interface_->cleanup();
  std::cout<<"Connection to Xiaomi Cleaner closed."<<std::endl;

}

void XiaomiTopicHandler::publishWallDistance_(struct irData_t data)
{
  sensor_msgs::Range msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "wall_distance_link";
  msg.radiation_type = sensor_msgs::Range::INFRARED;
  msg.max_range = 0.055;
  msg.min_range = 0.001;
  msg.field_of_view = 0.1;
  msg.range = data.wall;

  wall_ir_pub_.publish(msg);
}

void XiaomiTopicHandler::publishCliffData_(struct irData_t data)
{

  sensor_msgs::Range msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "cliff_sensor_fr_link";
  msg.radiation_type = sensor_msgs::Range::INFRARED;
  msg.max_range = 1.0;
  msg.min_range = 0.0;
  msg.field_of_view = 0.1;
  msg.range = data.cliff0;
  cliff_fr_pub_.publish(msg);

  msg.header.frame_id = "cliff_sensor_fl_link";
  msg.range = data.cliff1;
  cliff_fl_pub_.publish(msg);

  msg.header.frame_id = "cliff_sensor_r_link";
  msg.range = data.cliff2;
  cliff_r_pub_.publish(msg);

  msg.header.frame_id = "cliff_sensor_l_link";
  msg.range = data.cliff3;
  cliff_l_pub_.publish(msg);

}

void XiaomiTopicHandler::publishFrontSonar_(double data)
{
  sensor_msgs::Range msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "front_sonar_link";
  msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  msg.max_range = 0.36;
  msg.min_range = 0.0;
  msg.field_of_view = 0.1;
  msg.range = (float)data;

  sonar_pub_.publish(msg);
}

void XiaomiTopicHandler::publishBatteryState_(struct batteryState_t data)
{
  sensor_msgs::BatteryState msg;
  msg.header.stamp = ros::Time::now();
  msg.present = 1;
  msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  msg.percentage = (float)data.percentage;

  if(data.charging)
  {
    msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  } else
  {
    if(data.percentage > 99.0)
    {
      msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
    } else
    {
      msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    }
  }

  battery_pub_.publish(msg);
}

void XiaomiTopicHandler::publishLaserScan_(float *scan_data)
{
  sensor_msgs::LaserScan msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_laser_link";

  msg.angle_max = (float)M_PI;
  msg.angle_min = (float)-M_PI;
  msg.angle_increment = (float)(2 * M_PI) / 360;
  msg.time_increment = (1 / 24) / 360;
  msg.range_max = 10.0;
  msg.range_min = 0.0;

  msg.ranges.assign(scan_data, scan_data + 360);

  laser_pub_.publish(msg);
}

void XiaomiTopicHandler::publishOdometry_(struct odometryData_t data)
{
  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_footprint";

  msg.twist.twist.linear.x = data.vx;
  msg.twist.twist.linear.y = data.vy;
  msg.twist.twist.linear.z = 0.0;
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = data.rot_vy;

  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, data.rot_py);

  msg.pose.pose.position.x = data.px;
  msg.pose.pose.position.y = data.py;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = quat;

  odom_pub_.publish(msg);

  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time::now();;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_footprint";
  transform.transform.translation.x = data.px;
  transform.transform.translation.y = data.py;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation = quat;

  tf_broadcaster_.sendTransform(transform);
}

void XiaomiTopicHandler::cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg)
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
