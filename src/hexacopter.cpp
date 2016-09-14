#include "hexacopter.h"

using namespace hxcpt;

hexacopter::hexacopter(int argc, char** argv, bool verbose){
	ros::init(argc, argv, "sssup_hexacpt");

	// Allocate node class

	n_ = new ros::NodeHandle();

	// set param
	verbose_ = verbose;

	// Subscriber to flight mode

	mavros_state_sub_ = n_->subscribe("/mavros/state", 1, &hexacopter::stateCallback, this);
	mavros_gpsFIX_sub_ = n_->subscribe("/mavros/global_position/raw/fix", 1, &hexacopter::gpsFIXCallback, this);
	mavros_battery_sub_ = n_->subscribe("/mavros/battery", 1, &hexacopter::batteryCallback, this);

	// Publisher

	mavros_overrideIN_pub_ = n_->advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

	// Init variables
	
	mode_ = NOT_INIT;
	globalGPS_.status.status = NOT_INIT;
	voltage_ = NOT_INIT;
	altitude_ = 0;

	end_flag_ = false;

	targetRef_.x = 0;
	budgetResidual_ = 0;
	refVariance_ = 0;
	targetStamp_ = ros::Time::now();

	// Thread

	th_spin_ = new std::thread(&hexacopter::spin, this);
	th_control_rule_ = new std::thread(&hexacopter::control_rule, this);


}

hexacopter::hexacopter(ros::NodeHandle* n, bool verbose){

	// Assign node pointer
	n_ = n;

	// set param
	verbose_ = verbose;

	// Subscriber to flight mode

	mavros_state_sub_ = n_->subscribe("/mavros/state", 1, &hexacopter::stateCallback, this); 
	mavros_gpsFIX_sub_ = n_->subscribe("/mavros/global_position/raw/fix", 1, &hexacopter::gpsFIXCallback, this);
	mavros_battery_sub_ = n_->subscribe("/mavros/battery", 1, &hexacopter::batteryCallback, this);

	// Target info by camera recognition

	target_pos_sub_ = n->subscribe("/ids_rec/pose", 100, &hexacopter::markposeCallback, this);

	// Publisher
	
	mavros_overrideIN_pub_ = n_->advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
    	altitude_sub_ = n_->subscribe("/mavros/global_position/rel_alt", 1, &hexacopter::altitudeCallback, this);

	// Init variables
	
	mode_ = NOT_INIT;
	globalGPS_.status.status = NOT_INIT;
	voltage_ = NOT_INIT;
	altitude_ = 0;
	
	end_flag_ = false;

	targetRef_.x = 0;
	budgetResidual_ = 0;
	refVariance_ = 0;
	targetStamp_ = ros::Time::now();

	// Thread

	th_spin_ = new std::thread(&hexacopter::spin, this);
	th_control_rule_ = new std::thread(&hexacopter::control_rule, this);
}


hexacopter::~hexacopter(){
	pthread_exit(NULL);
}

bool hexacopter::set_arm(bool arm){

	if (arm && !preArm_check())
			return false;

	// Arm Hexacpt

	ros::ServiceClient arming_cl = n_->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

	mavros_msgs::CommandBool srv_setMode;

	srv_setMode.request.value = true;

	if(arming_cl.call(srv_setMode)){
		if (verbose_ == true){
			if (srv_setMode.response.success == 0){
				ROS_INFO("ARM send failed.");
				return false;
			}
			else{
				ROS_INFO("ARM send success.");
				return true;
			}
		}
	}else{
		ROS_ERROR("Failed arming or disarming");

		return false;
	}
}

bool hexacopter::set_Mode(uint8_t mode){
	// Set Guide mode

	ros::ServiceClient cl = n_->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	
	mavros_msgs::SetMode srv_setMode;
	
	srv_setMode.request.base_mode = 0;
	
	if (!Guide_Mode2str(mode, srv_setMode.request.custom_mode))
		ROS_ERROR("Mode Not Recognized, set default value: STABILIZED.");

	if(cl.call(srv_setMode)){
		if (verbose_ == true){
			if (srv_setMode.response.success){
				ROS_INFO("set_Mode() success.");
				return true;
			}
			else{
				ROS_INFO("set_Mode() failed.");
				return false;
			}
		}
	}
	else{
		ROS_ERROR("set_Mode() failed.");
		return false;
	}
}


bool hexacopter::preArm_check(){

	return true; // TODO only Simulation


	// Perform pre arming check

	// 1.

	// GPS is fix? 

	if ((mode_ == LOITER) || (mode_ == GUIDED)){
		if (globalGPS_.status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX){
			ROS_ERROR("GPS is not FIX");
			return false;
		}
		else{
			if(verbose_ == true){
				switch(globalGPS_.status.status){
					case sensor_msgs::NavSatStatus::STATUS_FIX: 
						ROS_INFO("GPS FIX, unaugmented fix.");
						break;
					case sensor_msgs::NavSatStatus::STATUS_SBAS_FIX: 
						ROS_INFO("GPS FIX, satellite-based augmentation.");
						break;
					default: // STATUS_GBAS_FIX
						ROS_INFO("GPS FIX, ground-based augmentation.");
				}
			}
		}
	}

	// 2.

	// Board status
	// --> All modes

	// 3. 
	// Voltage status

	if (voltage_ < BATTERY_MIN_V){
		ROS_ERROR("Battery Low %f", voltage_);
		return false;
	}
	// 4. 
	// RC status
	// --> All modes
	
	return true;

}

bool hexacopter::takeoff(const float alt){

	// Start taking off procedure

	ros::ServiceClient takeoff_cl = n_->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

	mavros_msgs::CommandTOL srv_takeoff;

	srv_takeoff.request.altitude = alt;
	srv_takeoff.request.latitude = 0;
	srv_takeoff.request.longitude = 0;
	srv_takeoff.request.min_pitch = 0;
	srv_takeoff.request.yaw = 0;

	if(takeoff_cl.call(srv_takeoff)){
		if (verbose_ == true)
			ROS_ERROR("srv_takeoff send ok %d", srv_takeoff.response.success);

			// TODO: WAIT UNTIL THE 0.95 OF REQUESTED ALTITUDE IS REACHED

		return true;
	}else{
		if (verbose_ == true)
			ROS_ERROR("Failed Takeoff");
		
		return false;
	}
}

bool hexacopter::land(){
		
	// Start Landing procedure

	ros::ServiceClient land_cl = n_->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

	mavros_msgs::CommandTOL srv_land;

	srv_land.request.altitude = 10;
	srv_land.request.latitude = 0;
	srv_land.request.longitude = 0;
	srv_land.request.min_pitch = 0;
	srv_land.request.yaw = 0;

	if(land_cl.call(srv_land)){
		if (verbose_ == true)
			ROS_INFO("srv_land send ok %d", srv_land.response.success);

			return true;

	}else{
		if (verbose_ == true)
				ROS_ERROR("Failed Land");

		return false;
	}

}

bool hexacopter::set_ORCIn(int Roll, int Pitch, int Yaw, int Throttle){

	mavros_msgs::OverrideRCIn msg;

	// Limit the Roll
	if (Roll > MAXRC)
		Roll = MAXRC;
	else 
		if (Roll < MINRC)
			Roll = MINRC;

	// Limit the Pitch
	if (Pitch > MAXRC)
		Pitch = MAXRC;
	else 
		if (Pitch < MINRC)
			Pitch = MINRC;

	// Limit the Throttle
	if (Throttle > MAXRC)
		Throttle = MAXRC;
	else 
		if (Throttle < MINRC)
			Throttle = MINRC;

	// Limit the Throttle
	if (Yaw > MAXRC)
		Yaw = MAXRC;
	else 
		if (Yaw < MINRC)
			Yaw = MINRC;

	if (end_flag_){
			Roll = 65536;
			Pitch = 65536;
			Throttle = 65536;
			Yaw = 65536;
	}

	// Compose Message	    

	msg.channels[0] = Roll;     	//Roll
	msg.channels[1] = Pitch;    	//Pitch
	msg.channels[2] = Throttle;   	//Throttle
	msg.channels[3] = Yaw;        	//Yaw
	msg.channels[4] = 0;
	msg.channels[5] = 0;
	msg.channels[6] = 0;
	msg.channels[7] = 0;

	//ROS_INFO("%d, %d, %d, %d\n", Roll, Pitch, Throttle, Yaw);

	mavros_overrideIN_pub_.publish(msg);
}

uint8_t hexacopter::get_Mode(){
	return mode_;
}

bool hexacopter::get_arm(){
	return armed_;
}

void hexacopter::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
		if(msg->mode == std::string("CMODE(0)"))
				return;

		if (!str2Guide_Mode(msg->mode, mode_))
				ROS_ERROR("Mode Not Recognized, set default value: STABILIZED.");

		guided_ = msg->guided == 128;
		armed_ = msg->armed == 128;
}

void hexacopter::batteryCallback(const mavros_msgs::BatteryStatus::ConstPtr& msg)
{
		voltage_ = msg->voltage;
}

void hexacopter::gpsFIXCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
		globalGPS_ = *msg;
}

void hexacopter::markposeCallback(const mark_follower::markPoseStampedPtr& msg){

	// Store data
	targetStamp_ = msg->stamp;
	targetRef_.x = msg->x;
	targetRef_.y = msg->y;
	budgetResidual_ = msg->budgetResidual;
	refVariance_ = msg->variance;

}

void hexacopter::altitudeCallback(const std_msgs::Float64Ptr &msg)
{
	altitude_ = msg->data;
}

bool hexacopter::str2Guide_Mode(const std::string str, uint8_t& mode){


		if (!strcmp(str.c_str(), "ALT_HOLD")){
				mode = ALT_HOLD;
				return true;        
		}

		if (!strcmp(str.c_str(), "LOITER")){
				mode = LOITER;
				return true;
		}


		if (!strcmp(str.c_str(), "GUIDED")){
				mode = GUIDED;
				return true;
		}

		if (!strcmp(str.c_str(), "STABILIZE")){
				mode = STABILIZE;
				return true;
		}

		// Set Stabilize as DEFAULT value
		mode = STABILIZE;

		return false;
		
}

bool hexacopter::Guide_Mode2str(const uint8_t mode, std::string& str){

		switch(mode){
				case ALT_HOLD:
						str = "ALT_HOLD"; 
						break;
				case LOITER:
						str = "LOITER";
						break;
				case GUIDED:
						str = "GUIDED";
						break;
				case STABILIZE:
						str = "STABILIZE";
						break;
				default:{
						// Set Stabilize as DEFAULT value
						str = "STABILIZE";
						return false;
						}
		}
		return true;
}


void hexacopter::init(){

		if (verbose_ == true)
				ROS_INFO("Init...");

		while ( ((mode_ == NOT_INIT) || (globalGPS_.status.status == NOT_INIT) || (voltage_ == NOT_INIT)) && ros::ok());

}

int hexacopter::getch()
{
		static struct termios oldt, newt;

		tcgetattr( STDIN_FILENO, &oldt);           // save old settings
		newt = oldt;
		newt.c_lflag &= ~(ICANON);                 // disable buffering      
		tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

		int c = getchar();  // read character (non-blocking)

		tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings

		return c;
}


bool hexacopter::setlocaltwist(geometry_msgs::Twist twist){

		ros::Publisher hexa_pub = n_->advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

		geometry_msgs::TwistStamped msg;

		msg.header.stamp = ros::Time::now();
		msg.header.seq = seq_++;
		msg.header.frame_id = "mavsetp";

		msg.twist = twist;

		hexa_pub.publish(msg);

		return true;
}

void hexacopter::control_rule(){

	int Roll, Pitch, Yaw, Throttle;

	Roll = BASERC;
	Pitch = BASERC;
	Throttle = MINRC;
	Yaw = BASERC;

	ros::Rate r(15);

	double err_x, err_y;

	while(ros::ok()){

		err_x = (1280 / 2) - targetRef_.x;
		err_y = (720 /  2) - targetRef_.y;


		// Calculate Roll and Pitch depending on the mode
		if (mode_ == LOITER && budgetResidual_ > 0){
			Roll = BASERC - err_x * FACTOR;
			Pitch = BASERC - err_y * FACTOR;

			// Throttle = BASERC - (altitude_ - 1) * 100;
			if (refVariance_ > 100 )
					Throttle = BASERC  - 100;   
			else
					Throttle = BASERC;  
		}else{
			Roll = BASERC;
			Pitch = BASERC;
			Throttle = BASERC;  
		}  

		set_ORCIn(Roll, Pitch, Yaw, Throttle);

		r.sleep();
	}	
}

// Set SYSID_MYGCS and checks setting

bool hexacopter::setSYSID_MYGCS()
{
	ros::ServiceClient client = n_->serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
	mavros_msgs::StreamRate srv;
	srv.request.stream_id = 0;
	srv.request.message_rate = 100;
	srv.request.on_off = 1;

	if (client.call(srv))
		ROS_INFO("Send OK");
	else
		ROS_INFO("Failed to call service");
	
	ros::ServiceClient cl_get = n_->serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
	
	mavros_msgs::ParamGet paramget;
	paramget.request.param_id = "SYSID_MYGCS";

	ros::ServiceClient cl = n_->serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
	
	uint16_t err_count = 1000;

	while(err_count > 0) {
		if (cl_get.call(paramget)) {
			ROS_INFO("Send OK %d Value: %ld", paramget.response.success, paramget.response.value.integer);
				
			if (paramget.response.value.integer == 0){

				// Create new params to set
				mavros_msgs::ParamSet paramset_new;
				paramset_new.request.param_id = "SYSID_MYGCS";
				paramset_new.request.value.integer = 1;
				paramset_new.request.value.real = 1;
				
				// Call paramset service 

				if (cl.call(paramset_new))
					ROS_INFO("SYSIS_MYGCS: send ok");
				else
					ROS_INFO("Failed to call service SYSIS_MYGCS");
					
			}
			else
				return true;
		} else
			ROS_ERROR("Faciled GET PARAMETER");
	}
	
	return false;

}



void hexacopter::spin(){

	// Init functions
	init();

	// Set Ground Control Station ID
	if (!setSYSID_MYGCS()){
		ROS_ERROR("Impossible set SYSIS_MYGCS after 1000 iterations.");
		return;
	}

	// Set MODE

	set_Mode(GUIDED);

	if (verbose_ == true)
			ROS_INFO("Arming...");
	
	while (!set_arm(ARM) && ros::ok());

	// ATTENTION MOTOR ARM
	takeoff(25);
	// char dir;
	ros::Rate r(10);
	bool set_loiter = true;
		
	while(ros::ok()){

		if (altitude_ > 24.5 && set_loiter){
			set_Mode(LOITER);
			set_loiter = false;
		}

		r.sleep();
	}


	// ATTENTION MOTOR ARM
	land();

	if (verbose_ == true)
			ROS_INFO("Disarming...");
	
	while (!set_arm(DISARM) && ros::ok());

}

