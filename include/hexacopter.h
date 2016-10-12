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
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/BatteryStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <mark_follower/markPoseStamped.h>
#include <std_msgs/Float64.h>

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
#define RELEASE_RC	65536

#define BUTTON_OFF	false
#define BUTTON_ON	true


#define K_P  ((altitude_> 12)?0.9:(altitude_> 3)?0.4:(altitude_> 2)?0.32:(altitude_> 1.2)?0.2:0.05)
#define K_I  0.01
#define sgn(x) ((x>0)?1:-1)

namespace hxcpt {

	enum Guide_Mode { 	
						GUIDED 		= 0, 
						LOITER 		= 1, 
						STABILIZE 		= 2,
						ALT_HOLD		= 3,
						NOT_INIT 		= 255,
					};

	enum arm_mode 	{ 	
						ARM 		= true,
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

		bool set_ORCIn(int, int, int, int, int ch5 = 0, int ch6  = 0, int ch7 = 0, int ch8 = 0);
		bool setlocaltwist(geometry_msgs::Twist);

		void spin();
		void control_rule();	

	private:

		void stateCallback(const mavros_msgs::State::ConstPtr&);
		void gpsFIXCallback(const sensor_msgs::NavSatFix::ConstPtr&);
		void batteryCallback(const mavros_msgs::BatteryStatus::ConstPtr&);
		void markposeCallback(const mark_follower::markPoseStampedPtr&);
		void altitudeCallback(const std_msgs::Float64Ptr&);
		void rcINCallback(const mavros_msgs::RCIn::ConstPtr&);

		bool preArm_check();
		void init();
		int getch();
		bool setSYSID_MYGCS();


		bool str2Guide_Mode(const std::string, uint8_t&);
		bool Guide_Mode2str(const uint8_t, std::string&);

		bool verbose_;

		// Flight params
		float altitude_;
		uint8_t mode_;
		int guided_;
		int armed_;
		sensor_msgs::NavSatFix globalGPS_;
		float voltage_;
		int seq_;

		// Control step
		int8_t roll_step_, pitch_step_, yaw_step_, throttle_step_;
		
		// Release/hold flag on RC override,, last button state
		bool lbstate_; 

		std::thread* th_control_rule_;
		std::thread* th_spin_;

		// Reference target position and informations
		geometry_msgs::Point targetRef_;
		int budgetResidual_;
		float refVariance_;
		ros::Time targetStamp_;

		// External RC command
		mavros_msgs::RCIn rcIn_;

		// ROS
		
		ros::NodeHandle* n_;

		ros::Subscriber mavros_state_sub_;
		ros::Subscriber mavros_gpsFIX_sub_;
		ros::Subscriber mavros_battery_sub_;
		ros::Subscriber mavros_rcIn_sub_;
		ros::Subscriber target_pos_sub_;
		ros::Subscriber altitude_sub_;

		ros::Publisher mavros_overrideIN_pub_;

	};
} 
#endif 