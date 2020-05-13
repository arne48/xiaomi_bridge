#include <xiaomi_bridge/xiaomi_player_interface.h>
#include <libplayerc/playerc.h>

XiaomiPlayerInterface::XiaomiPlayerInterface(std::string vacuum_ip)
{

  robot_client_ = playerc_client_create(NULL, vacuum_ip.c_str(), 6665);
  if (0 != playerc_client_connect(robot_client_))
    std::cout<<"Error when connecting to: Robot"<<std::endl;

  ir_wall_ = playerc_ir_create(robot_client_, 0);
  if (playerc_ir_subscribe(ir_wall_, PLAYER_OPEN_MODE))
    std::cout<<"Error when subscribing to: Wall IR"<<std::endl;

  ir_cliff_ = playerc_ir_create(robot_client_, 1);
  if (playerc_ir_subscribe(ir_cliff_, PLAYER_OPEN_MODE))
    std::cout<<"Error when subscribing to: Cliff IR"<<std::endl;

  base_ = playerc_position2d_create(robot_client_, 0);
  if (playerc_position2d_subscribe(base_, PLAYER_OPEN_MODE))
    std::cout<<"Error when subscribing to: Base"<<std::endl;

  laser_scanner_ = playerc_laser_create(robot_client_, 0);
  if (playerc_laser_subscribe(laser_scanner_, PLAYER_OPEN_MODE))
    std::cout<<"Error when subscribing to: Laser Scanner"<<std::endl;

  front_sonar_ = playerc_sonar_create(robot_client_, 0);
  if (playerc_sonar_subscribe(front_sonar_, PLAYER_OPEN_MODE))
    std::cout<<"Error when subscribing to: Front Sonar"<<std::endl;

  battery_info_ = playerc_power_create(robot_client_, 0);
  if (playerc_power_subscribe(battery_info_, PLAYER_OPEN_MODE))
    std::cout<<"Error when subscribing to: Battery"<<std::endl;

}

XiaomiPlayerInterface::~XiaomiPlayerInterface() = default;

void XiaomiPlayerInterface::getLaserData(float* scan_data)
{
  for(int idx = laser_scanner_->scan_count - 1; idx >= 0; idx--)
  {
    //scanner samples are reversed
    scan_data[(laser_scanner_->scan_count - 1) - idx] = (float)laser_scanner_->ranges[idx];
  }
}

void XiaomiPlayerInterface::updateRobotState()
{
  playerc_client_read(robot_client_);
}

irData_t XiaomiPlayerInterface::getIrSensorData()
{
  if (ir_wall_->data.ranges_count == 4)
  {
    struct irData_t data = {
	    .wall = ir_wall_->data.ranges[0],
	    .cliff0 = ir_cliff_->data.ranges[0],
	    .cliff1 = ir_cliff_->data.ranges[1],
	    .cliff2 = ir_cliff_->data.ranges[2],
	    .cliff3 = ir_cliff_->data.ranges[3]
    };
    return data;
  } else	
  {	
    return {0.0, 0.0, 0.0, 0.0};	
  }
}

double XiaomiPlayerInterface::getSonarData()
{
  if(front_sonar_->scan_count==1)
  {
    return front_sonar_->scan[0];
  } else
  {
    return 0.0;
  }
}

batteryState_t XiaomiPlayerInterface::getBatteryData()
{
  struct batteryState_t battery_state = { .percentage = battery_info_->percent,
                                          .charging = battery_info_->charging != 0};

  return battery_state;
}

odometryData_t XiaomiPlayerInterface::getOdometryData()
{
  struct odometryData_t odom = { .px = base_->px, .py = base_->py, .vx = base_->vx,
                                 .vy= base_->vy, .rot_py = base_->pa, .rot_vy = base_->va};
  return odom;
}

void XiaomiPlayerInterface::setVelocityCommand(struct velCmd_t cmd)
{

  playerc_position2d_set_cmd_vel(base_, cmd.px, cmd.py, cmd.az, 1);

}

void XiaomiPlayerInterface::cleanup()
{

  playerc_ir_unsubscribe(ir_cliff_);
  playerc_ir_destroy(ir_cliff_);
  playerc_ir_unsubscribe(ir_wall_);
  playerc_ir_destroy(ir_wall_);

  playerc_position2d_unsubscribe(base_);
  playerc_position2d_destroy(base_);

  playerc_laser_unsubscribe(laser_scanner_);
  playerc_laser_destroy(laser_scanner_);

  playerc_sonar_unsubscribe(front_sonar_);
  playerc_sonar_destroy(front_sonar_);

  playerc_power_unsubscribe(battery_info_);
  playerc_power_destroy(battery_info_);

  playerc_client_disconnect(robot_client_);
  playerc_client_destroy(robot_client_);

}
