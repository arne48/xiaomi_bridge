# Xiaomi Vacuum Cleaner ROS2 Bridge

## Introduction
This package bridges between a rooted Xiaomi Vacuum Cleaner, publishes its sensor data, and forwards Twist messages to drive the robot.

### Functions
__What works__
- Odometry
- Driving
- Laser Sensor
- Cliff Sensor
- Wall Sensor
- Front Ultrasonic Sensor
- Battery Info

__What doesn't work__
- Motor Control (Vaccum, Main-Brush, Side-Brush) 
- Bumper
- Wheel Drop Sensor

## Requirements
__1. Rooted Vacuum Cleaner:__ Please use the great work of Dennis Giese and Daniel Wegemer for doing this. Their repository can be found at [https://github.com/dgiese/dustcloud](https://github.com/dgiese/dustcloud).

__2. Disable Watchdog for RoboController:__ On the robot, in _"/opt/rockrobo/watchdog/ProcessList.conf"_ comment out the line _"RoboController,setsid RoboController&,1,3,0"_

__3. Disable Firewall "Drop" rules for Player ports:__ On the robot, in _"/opt/rockrobo/watchdog/rrwatchdoge.conf"_ comment out the lines _"iptables -I INPUT -j DROP -p tcp --dport 6665"_ and _"iptables -I INPUT -j DROP -p udp --dport 6665"_

__Notice:__

The player client libraries are needed for this bridge and are built automatically.
They are installed locally using the catkin_devel_prefix. This doesn't require root permissions and keeps the host system clean.
Detailed information and how-tos for this library are available here, [http://playerstage.sourceforge.net/doc/Player-svn/player](http://playerstage.sourceforge.net/doc/Player-svn/player)

## Bridge Node
The node can be started on another computer in the same network as the vacuum cleaner. It connects to the internally used player server for getting sensor data and commanding the robot.

### Startup
#### Robot
1. Make sure the RoboController is not running.
2. Connect to the robot by ssh and start the RoboController via _"/opt/rockrobo/cleaner/bin/RoboController"_.
3. Push the "Home" button on the robot to let it search for the dock.
4. Once the robot keeps moving, kill the RoboController via _Ctrl-C_
5. Lift the robot to interrupt its navigation.
6. Now you should just hear the LaserScanner turning. If so, the robot is ready to be controlled by you.
#### Cartographer
1. _ros2 launch xiaomi_bridge xiaomi_bringup.launch.py_
2. _ros2 launch xiaomi_bridge xiaomi_cartographer.launch.py_
3. Saving your map -> _ros2 run nav2_map_server map_saver_cli_
#### Navigation
1. _ros2 launch xiaomi_bridge xiaomi_bringup.launch.py_
2. _ros2 launch xiaomi_bridge xiaomi_navigation.launch.py map:={PATH_TO_MAP}_

### Details
* *Parameter*
	* vacuum\_ip [str]_ __|__ _IP address of the robot._ __default:__ _192.168.8.1_
* *Topics*
	* _/cmd\_vel [geometry\_msgs::Twist]_
	* _/scan [sensor\_msgs::LaserScan]_
	* _/odom [nav\_msgs::Odometry]_
	* _/battery\_state [sensor\_msgs::BatteryState]_
	* _/front\_sonar [sensor\_msgs::Range]_
	* _/wall\_distance [sensor\_msgs::Range]_
	* _/cliff/front\_right [sensor\_msgs::Range]_
	* _/cliff/front\_left [sensor\_msgs::Range]_
	* _/cliff/right [sensor\_msgs::Range]_
	* _/cliff/left [sensor\_msgs::Range]_
* *Launch Files*
	* _xiaomi_bringup.launch.py_ __|__ starts the xiaomi_bridge_node, loads the robot model, and starts the robot state publisher. Contains the setting of the IP address parameter.
	* _xiaomi_robot_state.launch.py_ __|__ loads the robot's urdf model and starts the robot state publisher
	* _xiaomi_navigation.launch.py_ __|__ loads navigation paramteres, brings up nav2 and starts rviz2
	* _xiaomi_cartographer.launch.py_ __|__ starts cartographer, invokes occupancy_grid.launch.py, and starts rviz2
	* _occupancy_grid.launch.py_ __|__ starts the occupancy grid node
* *Config Files*
	* _robot_config.yaml_ __|__ contains the IP address of the robot
	* _xiaomi_navigation.yaml_ __|__ contains the navigation parameters for nav2
	* _xiaomi_cartographer.lua_ __|__ contains the parameters for cartographer
	
	
## Navigation
The configuration for the navigation stack is closely oriented to the TurtleBot. You can find more information and tutorials in Robotis' manual.
[https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_setup/](https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_setup/)

### rviz configuration overview
(to be updated)
![rviz overview](https://raw.githubusercontent.com/arne48/xiaomi_bridge/master/images/rviz_xiaomi.png)

## Known issues
1. The robot can't be controlled while a bumper is pressed.
2. Yes, the startup is that hacky. I am gratefully looking for another way to startup the motors and sensors. The best way would be, if that is even possible while the RoboController keeps running.
3. Preventing the RoboController from running drastically limits the normal functions of the robot. Such as the buttons, the power indicator via LED color and the Mi Home app.