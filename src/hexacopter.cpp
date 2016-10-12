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

	// set OFF 
	lbstate_ = BUTTON_OFF;

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
	mavros_rcIn_sub_ = n_->subscribe("/mavros/rc/in", 1, &hexacopter::rcINCallback, this);
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
	
	// set OFF 
	lbstate_ = BUTTON_OFF;

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

bool hexacopter::set_ORCIn(int ch1, int ch2, int ch3, int ch4, int ch5, int ch6, int ch7, int ch8){

	mavros_msgs::OverrideRCIn msg;

	// Limit the ch1
	if (ch1 > MAXRC)
		ch1 = MAXRC;
	else 
		if (ch1 < MINRC)
			ch1 = MINRC;

	// Limit the ch2
	if (ch2 > MAXRC)
		ch2 = MAXRC;
	else 
		if (ch2 < MINRC)
			ch2 = MINRC;

	// Limit the ch4
	if (ch4 > MAXRC)
		ch4 = MAXRC;
	else 
		if (ch4 < MINRC)
			ch4 = MINRC;

	// Limit the ch4
	if (ch3 > MAXRC)
		ch3 = MAXRC;
	else 
		if (ch3 < MINRC)
			ch3 = MINRC;

	// Compose Message	    

	msg.channels[0] = ch1;     	//Roll
	msg.channels[1] = ch2;    	//Pitch
	msg.channels[2] = ch4;   	//Throttle
	msg.channels[3] = ch3;        	//Yaw
	msg.channels[4] = ch5;
	msg.channels[5] = ch6;
	msg.channels[6] = ch7;
	msg.channels[7] = ch8;

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

void hexacopter::rcINCallback(const mavros_msgs::RCIn::ConstPtr& msg){

	// Store data

	rcIn_ = *msg;

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
	double sum_err_x = 0, sum_err_y = 0;

	double dt = 0.067;
	

	while(ros::ok()){

		if (mode_ == GUIDED)
			continue;

		// Operator takes control

		if (rcIn_.channels[4] > BASERC){ // swith ON from transmitter
			if (!lbstate_) // laste stete was OFF
				set_ORCIn(RELEASE_RC, RELEASE_RC, RELEASE_RC, RELEASE_RC, RELEASE_RC, RELEASE_RC, RELEASE_RC, RELEASE_RC);

			lbstate_ = BUTTON_ON;

			continue;

		}else{ // switch OFF from transmitter
			if (lbstate_){ // last state was ON

				// Init control vars
				targetRef_.x = 0;
				targetRef_.x = 0;

				sum_err_x = 0;
				sum_err_y = 0;
			}
			
			lbstate_ = BUTTON_OFF;
		}

			

		err_x = (1280 / 2) - targetRef_.x;
		err_y = (720 /  2) - targetRef_.y;

		sum_err_x += err_x * dt;
		sum_err_y += err_y * dt;

		// std::cout << targetRef_.x << " " << targetRef_.y << " "<< sum_err_x << " " << sum_err_y<<  std::endl ;

		// Anti wind-UP

		if (sum_err_x > 100)
			sum_err_x = 100;
		
		if (sum_err_y > 100)
			sum_err_y = 100;

		if (sum_err_x < -100)
			sum_err_x = -100;
		
		if (sum_err_y < -100)
			sum_err_y = -100;

		// Calculate Roll and Pitch depending on the mode
		if (mode_ == LOITER && budgetResidual_ > 0 ){
			
			Roll = BASERC - err_x * K_P + K_I * sum_err_x;
			Pitch = BASERC - err_y * K_P + K_I * sum_err_y;

			// Throttle = BASERC - (altitude_ - 1) * 100;
			if (refVariance_ > 100 && altitude_ > 0.5){
				// Throttle = BASERC  - 100; 	
				if (altitude_ > 0.5)
					Throttle = BASERC - ((altitude_ - 0.5) * 20 + 50);
				else
					Throttle = BASERC  - 100; 			
			}
			else
				Throttle = BASERC;  
		}else{
			Roll = BASERC;
			Pitch = BASERC;
			Throttle = BASERC;  
		}  

		std::cout << Roll << " " << Pitch << " " << Throttle << std::endl;

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
	takeoff(5);
	// char dir;
	ros::Rate r(10);
	bool set_loiter = true;
		
	while(ros::ok()){

		if (altitude_ > 4.5 && set_loiter){
			set_Mode(LOITER);
			// // usleep(10000000);
			// break;
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

