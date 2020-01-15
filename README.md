# Xiaomi Vacuum Cleaner ROS Bridge

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
1. Make sure the RoboController is not running.
2. Connect to the robot by ssh and start the RoboController via _"/opt/rockrobo/cleaner/bin/RoboController"_.
3. Push the "Home" button on the robot to let it search for the dock.
4. Once the robot keeps moving, kill the RoboController via _Ctrl-C_
5. Lift the robot to interrupt its navigation.
6. Now you should just hear the LaserScanner turning. If so, the robot is ready to be controlled by you.

### Details
* *Parameter*
	* _\~ vacuum\_ip [str]_ __|__ _IP addres of the robot._ __default:__ _192.168.8.1_
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
	* _bringup.launch_ __|__ starts the xiaomi_bridge_node, loads the robot model, and starts the robot state publisher. Contains the setting of the IP address parameter.
	* _move_base.launch_ __|__ starts the move_base node
	* _navigation.launch_ __|__ starts all needed nodes and loads all parameters for navigation
	* _rviz.launch_ __|__ starts rviz with the provided config file
	
	
	
## Navigation
The configuration for the navigation stack is closely oriented to the TurtleBot. You can find more information and tutorials in its ROS wiki.
[http://wiki.ros.org/Robots/TurtleBot](http://wiki.ros.org/Robots/TurtleBot)

### rviz configuration overview

![rviz overview](https://raw.githubusercontent.com/arne48/xiaomi_bridge/images/rviz_xiaomi.png)

## Known issues
1. The robot can't be controlled while a bumper is pressed.
2. Yes, the startup is that hacky. I am gratefully looking for another way to startup the motors and sensors. The best way would be, if that is even possible while the RoboController keeps running.
3. Preventing the RoboController from running drastically limits the normal functions of the robot. Such as the buttons, the power indicator via LED color and the Mi Home app.
