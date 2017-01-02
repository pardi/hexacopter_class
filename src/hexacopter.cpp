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

	// Service to get params

	ros::ServiceClient client = n_->serviceClient<ids_viewer::IDSparams>("ids_viewer/params");

	ids_viewer::IDSparams srv;

	if (client.call(srv)){

		width_ = srv.response.width;
		height_ = srv.response.height;

	}else{
		width_ = STD_VID_WIDTH;
		height_ = STD_VID_HEIGHT;		
	}


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

	// Wait first  RC command

	rcStart_ = false;

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

	std::vector<int> chs;

	// Limit the ch1
	if (ch1 > MAXRC)
		ch1 = MAXRC;
	else 
		if (ch1 < MINRC)
			ch1 = MINRC;

	chs.push_back(ch1);

	// Limit the ch2
	if (ch2 > MAXRC)
		ch2 = MAXRC;
	else 
		if (ch2 < MINRC)
			ch2 = MINRC;

	chs.push_back(ch2);

	// Limit the ch4
	if (ch4 > MAXRC)
		ch4 = MAXRC;
	else 
		if (ch4 < MINRC)
			ch4 = MINRC;

	chs.push_back(ch3);

	// Limit the ch4
	if (ch3 > MAXRC)
		ch3 = MAXRC;
	else 
		if (ch3 < MINRC)
			ch3 = MINRC;

	chs.push_back(ch4);

	// Other channels
	chs.push_back(ch5);
	chs.push_back(ch6);
	chs.push_back(ch7);	
	chs.push_back(ch8);

	return send_RCmsg(chs);

}

bool hexacopter::send_RCmsg(const std::vector<int> chs){

	if (chs.size() != 8)
		return false;

	// Define a message
	mavros_msgs::OverrideRCIn msg;

	// Fill the message

	msg.channels[0] = chs[0];
	msg.channels[1] = chs[1];
	msg.channels[2] = chs[2];
	msg.channels[3] = chs[3];
	msg.channels[4] = chs[4];
	msg.channels[5] = chs[5];
	msg.channels[6] = chs[6];
	msg.channels[7] = chs[7];

	// Publish the message

	mavros_overrideIN_pub_.publish(msg);

	return true;
}


bool hexacopter::reset_ORCIn(){

	std::vector<int> chs;

	chs.push_back(RELEASE_RC);
	chs.push_back(RELEASE_RC);
	chs.push_back(RELEASE_RC);	
	chs.push_back(RELEASE_RC);
	chs.push_back(RELEASE_RC);
	chs.push_back(RELEASE_RC);
	chs.push_back(RELEASE_RC);	
	chs.push_back(RELEASE_RC);

	return send_RCmsg(chs);

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

	rcStart_ = true;

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

		if (!strcmp(str.c_str(), "LAND")){
				mode = LAND;
				return true;
		}

		if (!strcmp(str.c_str(), "AUTO")){
				mode = AUTO;
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
				case LAND:
						str = "LAND";
						break;
				case AUTO:
						str = "AUTO";
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

		while ( ((mode_ == NOT_INIT) || (globalGPS_.status.status == NOT_INIT)) && ros::ok()); //|| (voltage_ == NOT_INIT)) && ros::ok());

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

	int iter = 0;

	ros::Rate r(15);

	double err_x, err_y, err_yaw;
	double sum_err_x = 0, sum_err_y = 0, sum_err_yaw = 0;

	double x1[3] = {0, 0, 0}, x2[3] = {0, 0, 0};

	double dt = 0.067;
	

	while(ros::ok()){

		// if (!armed_){
		// 	reset_ORCIn();
		// 	continue;
		// }

		// if (!rcStart_)
		// 	continue;

		if (mode_ != LOITER) // XXX alt_hold option
			continue;

		// Operator takes control

		// if (rcIn_.channels[7] > BASERC){ // switch ON from transmitter
		// 	if (!lbstate_) // last  state was OFF
		// 		reset_ORCIn();

		// 	lbstate_ = BUTTON_ON;

		// 	// ROS_INFO("BUTTON_ON - RC control");
		// 	continue;

		// }else{ // switch OFF from transmitter
		// 	if (lbstate_){ // last state was ON

		// 		// Init control vars
		// 		targetRef_.x = 0;
		// 		targetRef_.x = 0;

		// 		sum_err_x = 0;
		// 		sum_err_y = 0;
		// 	}
			
		// 	lbstate_ = BUTTON_OFF;
		// 	// ROS_INFO("BUTTON_OFF - ROS control");
		// }

		
		err_x = (width_ / 2) - targetRef_.x; // 1280
		err_y = (height_/  2) - targetRef_.y;  //720
		// err_yaw = yaw_ - targetRef_.yaw;

		sum_err_x += err_x * dt;
		sum_err_y += err_y * dt;	
		// sum_err_yaw += err_yaw * dt;

		// Anti wind-UP

		if (sum_err_x > 100)
			sum_err_x = 100;
		
		if (sum_err_y > 100)
			sum_err_y = 100;

		if (sum_err_yaw > 100)
			sum_err_yaw = 100;

		if (sum_err_x < -100)
			sum_err_x = -100;
		
		if (sum_err_y < -100)
			sum_err_y = -100;

		if (sum_err_yaw < -100)
			sum_err_yaw = -100;

		// Calculate Roll and Pitch depending on the mode
		if (mode_ == LOITER && budgetResidual_ > 0 ){

			// Roll = BASERC - C(err_x, &x1[0], &x2[0]);
			// Pitch = BASERC - C(err_x, &x1[1], &x2[1]);
			
			Roll = BASERC - K_P * err_x + K_I * sum_err_x;
			Pitch = BASERC - K_P * err_y + K_I * sum_err_y;
			// Yaw = BASERC - K_P * err_yaw  + K_I * sum_err_yaw;

			// Throttle = BASERC - (altitude_ - 1) * 100;
			if (refVariance_ > 100){// && altitude_ > 0.5){
				//if (altitude_ < 1.5 && iter > 1000)
				
				Throttle = BASERC  - 150;//100;
				
				// else{
				// 	Throttle = BASERC;
				// 	iter++;
				// }

				//if (altitude_ > 0.5)
					// Throttle = BASERC - ((altitude_ - 0.2) * 10 + 50);
				//else
			}
			else
				Throttle = BASERC;  
		}else{
			Roll = BASERC;
			Pitch = BASERC;
			Yaw = BASERC; 

			if (( altitude_ < 25 ) && ( budgetResidual_ < -50 ))
				Throttle = BASERC + 80;
			 else
			 	Throttle = BASERC;
		}  

		// std::cout << Roll << " " << Pitch << " " << Throttle << std::endl;

		set_ORCIn(Roll, Pitch, Throttle, Yaw);

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

double hexacopter::C(const double x, double* z1, double* z2){

	double y = 0;

	y = b1 * x + *z1; // y(k) = b(1) x(k) + z1(k-1)
	*z1 = b2 * x + *z2  - a2 * y; // z1(k) = b(2) x(k) + z2(k-1) - a(2) y(k)
	*z2 = b3 * x  - a3 * y; // z2(k) = b(3) x(k)  - a(3) y(k)

	return y;
}

mavros_msgs::Waypoint hexacopter::generateWP( float latitude, float longitude, float alt, int command, int frame){

	mavros_msgs::Waypoint wp;

	wp.frame = frame; // Default Global Frame
	wp. command = command;
	wp.is_current = false;
	wp.autocontinue = false;
	wp.param1 = 0;
	wp.param2 = 0;
	wp.param3 = 0;
	wp.param4 = 0;
	wp.x_lat = latitude;
	wp.y_long = longitude;
	wp.z_alt = alt;

	return wp;


}

void hexacopter::setWPMission(){

	// Load WPs

	sendWP(generateWP(-35.3654, 149.1456, 650));

}

bool hexacopter::sendWP(mavros_msgs::Waypoint wp){


	mavros_msgs::WaypointPush srv_wp;

	// Load WPs

	srv_wp.request.waypoints.push_back(wp);

	// Send wp mission procedure

	ros::ServiceClient waypointMission_cl = n_->serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

	if(waypointMission_cl.call(srv_wp)){
		if (verbose_ == true)
			ROS_ERROR("srv_wp send ok %d", srv_wp.response.success);
		return true;
	}else{
		if (verbose_ == true)
			ROS_ERROR("Failed push WP");
		
		return false;
	}

}

double hexacopter::distWPs(mavros_msgs::Waypoint wp1, mavros_msgs::Waypoint wp2){

	const double EARTH_RADIUS_KM = 6371;

	double lat1_rad = wp1.x_lat * M_PI / 180.0;
	double long1_rad = wp1.y_long * M_PI / 180.0;
	
	double lat2_rad = wp2.x_lat * M_PI / 180.0;
	double long2_rad = wp2.y_long * M_PI / 180.0;

	double dlat = lat2_rad - lat1_rad;
	double dlon = long2_rad - long1_rad;

	double a = pow(sin(dlat / 2), 2) + cos(lat1_rad) * cos(lat2_rad) * pow(sin(dlon / 2), 2);

	return   2 * EARTH_RADIUS_KM * atan2(sqrt(a), sqrt(1 - a));

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

	// set_Mode(STABILIZE);

	// Free RC from previously override

	reset_ORCIn();

	// if (verbose_ == true)
	// 	ROS_INFO("Arming...");
	
	// while (!set_arm(ARM) && ros::ok());

	// ATTENTION MOTOR ARM
	// takeoff(10);
	char dir;
	ros::Rate r(10);
	bool set_loiter = true;
		
	//---> Seek and hunt <---

	// setPMission();

	// set_mode(AUTO);
	
	//-----------------------------------

	while(ros::ok()){

		//---> Seek and hunt <---

		// if (budgetResidual_ > 0)
		// 	set_mode(LOITER);
		// else 
		// 	if (budgetResidual_ < -300 && mode_ == LOITER)
		// 		set_mode(AUTO);				

		//-----------------------------------


		if (altitude_ > 9.5 && set_loiter){


			// set_Mode(LOITER);
			// // First Trial
			// usleep(10000 * 1000);
			// break;
			set_loiter = false;

		}

		if (!set_loiter && altitude_ < 0.6)
			break;

		r.sleep();
	}


	// ATTENTION MOTOR ARM
	land();

	while(altitude_ > 0.1);

	if (verbose_ == true)
		ROS_INFO("Disarming...");
	
	// while (!set_arm(DISARM) && ros::ok());

}

