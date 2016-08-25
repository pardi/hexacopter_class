#ifndef __HEXACOPTER_H__
#define __HEXACOPTER_H__

// ROS LIBs
#include <ros/ros.h>
// ROS MSG LIBs
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/BatteryStatus.h>
#include <sensor_msgs/NavSatFix.h>

// ROS MSG LIBs
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>

// Standardi LIBs
#include <termios.h>
#include <thread>

#define MINRC   1150
#define BASERC  1500
#define MAXRC   1800
#define BATTERY_MIN_V 14.5

namespace hxcpt {

	enum Guide_Mode { 	GUIDED 		= 0, 
						LOITER 		= 1, 
						STABILIZE 	= 2,
						ALT_HOLD	= 3,
						NOT_INIT 	= 255,
					};

	enum arm_mode 	{ 	ARM 	= true,
						DISARM 	= false
					};

	class hexacopter{

	public:

		hexacopter(ros::NodeHandle*, bool verbose = false);
		
		hexacopter(int, char**, bool verbose = false);

		~hexacopter();

		bool set_arm(bool);
		bool set_Mode(uint8_t);
		uint8_t get_Mode();
		bool get_arm();

		bool takeoff(const float);
		bool land();

		bool set_ORCIn(int, int, int, int);
		bool setlocaltwist(geometry_msgs::Twist);

		void spin();
		void ctrl();	

	private:

		void stateCallback(const mavros_msgs::State::ConstPtr&);
		void gpsFIXCallback(const sensor_msgs::NavSatFix::ConstPtr&);
		void batteryCallback(const mavros_msgs::BatteryStatus::ConstPtr&);
		void targetPosCallback(const geometry_msgs::PointStamped::ConstPtr&);

		bool preArm_check();
		void init();
		int getch();
		bool setSYSID_MYGCS();


		bool str2Guide_Mode(const std::string, uint8_t&);
		bool Guide_Mode2str(const uint8_t, std::string&);

	    ros::NodeHandle* n_;

	    bool verbose_;

	    uint8_t mode_;
	    int guided_;
	    int armed_;
	   	sensor_msgs::NavSatFix globalGPS_;
	   	float voltage_;
	   	int seq_;
	   	geometry_msgs::PointStamped target_pos_;

	   	int8_t roll_step_, pitch_step_, yaw_step_, throttle_step_;
	   	bool end_flag_;

		std::thread* th_ctrl_;
		std::thread* th_spin_;

		ros::Subscriber mavros_state_sub_;
		ros::Subscriber mavros_gpsFIX_sub_;
    	ros::Subscriber mavros_battery_sub_;
    	ros::Subscriber target_pos_sub_;

		ros::Publisher mavros_overrideIN_pub_;

	};
} 
#endif 