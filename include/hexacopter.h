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
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/BatteryStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <mark_follower/markPoseStamped.h>
#include <std_msgs/Float64.h>

// ROS MSG LIBs
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <ids_viewer/IDSparams.h>
#include <uav_semaphore/flyZone.h>
#include <uav_semaphore/holdDropZone.h>

// Standard LIBs
#include <termios.h>
#include <thread>

#define STD_VID_WIDTH 	1280 
#define STD_VID_HEIGHT 	720
#define STD_VID_BPP 		24

#define MINRC   1000
#define BASERC  1490
#define MAXRC   1850


#define BATTERY_MIN_V 14.5
#define RELEASE_RC	0 //65535

#define BUTTON_OFF	false
#define BUTTON_ON	true

#define FIRST_CHALLENGE		0
#define THIRD_CHALLENGE		1

#define MAX_ALTITUDE 20
#define UAV_ID 1

// Filter parameters
#define b1 663416.2537313433
#define b2 -1326699,83084577
#define b3 663283.5837479271
#define a2 1.323383084577114
#define a3 0.330016583747927


#define K_P  5//((altitude_> 12)?0.9:(altitude_> 3)?0.4:(altitude_> 2)?0.32:(altitude_> 1.2)?0.2:0.15)
#define K_I  0.15
#define sgn(x) ((x>0)?1:-1)

namespace hxcpt {

	enum Guide_Mode { 	
						GUIDED 		= 0, 
						LOITER 			= 1, 
						STABILIZE 		= 2,
						ALT_HOLD		= 3,
						LAND 			= 4,
						AUTO			= 5,
						NOT_INIT 		= 255
					};

	enum arm_mode 	{ 	
						ARM 		= true,
						DISARM 	= false
					};


	enum FState	{ 	
						SEEKING,
						STRATEGY_MAKER,
						APPROACHING,
						GRASPING,
						LIFT_OBJ,
						DRAG_TO_DZ,
						LOCATE_DROP_BOX,
						RELEASE_OBJ,
						LANDING,
						TAKEOFF,
						PILOT_CTRL				
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
		bool send_RCmsg(const std::vector<int>);
		bool reset_ORCIn();

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

		double C(const double, double*, double*);
		mavros_msgs::Waypoint generateWP(float, float, float, int command = 16, int frame = 0);
		void setWPMission();
		bool sendWP(mavros_msgs::Waypoint);
		double distWPs(mavros_msgs::Waypoint, mavros_msgs::Waypoint);
		bool check_grasp();
		bool hold_on_DZ(bool);


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
		
		// Release/hold flag on RC override, last button state
		bool lbstate_; 
		bool resetIntegralComponent_;

		std::thread* th_control_rule_;
		std::thread* th_spin_;

		// Reference target position and informations
		geometry_msgs::Point targetRef_;
		int budgetResidual_;
		float refVariance_;
		ros::Time targetStamp_;

		// Camera params
		int width_, height_;
		
		// External RC command
		mavros_msgs::RCIn rcIn_;
		bool rcStart_;

		// Challenge information
		bool challenge_;
		FState Fstatus_;

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