#ifndef XIAOMI_BRIDGE_XIAOMI_PLAYER_INTERFACE_H
#define XIAOMI_BRIDGE_XIAOMI_PLAYER_INTERFACE_H

#include <libplayerc/playerc.h>
#include <vector>
#include <map>
#include <iostream>

typedef std::map<std::string, float> SensorData;
struct irData_t { float wall; float cliff0; float cliff1; float cliff2; float cliff3; };
struct batteryState_t { double percentage; bool charging; };
struct velCmd_t { double px; double py; double pz;
                  double ax; double ay; double az; };
struct odometryData_t {double px; double py; double vx; double vy; double rot_py; double rot_vy; };

class XiaomiPlayerInterface
{
public:
  XiaomiPlayerInterface();
  XiaomiPlayerInterface(std::string);
  ~XiaomiPlayerInterface();

  void getLaserData(float*);
  irData_t getIrSensorData();
  batteryState_t getBatteryData();
  odometryData_t getOdometryData();
  double getSonarData();

  void updateRobotState();
  void setVelocityCommand(velCmd_t);
  void cleanup();

private:
  playerc_client_t *robot_client_;
  playerc_ir_t *ir_cliff_;
  playerc_ir_t *ir_wall_;
  playerc_sonar_t *front_sonar_;
  playerc_laser_t *laser_scanner_;
  playerc_position2d_t *base_;
  playerc_power_t *battery_info_;

};

#endif //XIAOMI_BRIDGE_XIAOMI_PLAYER_INTERFACE_H
