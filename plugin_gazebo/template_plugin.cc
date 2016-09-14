#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

// Circle radius
#define RADIUS 17

// Define shapes
#define CIRCLE 0
#define SQUARE 1
#define STOP   2
#define DOT    2 // DOT == STOP
#define EIGHT_SIMPLE 3

#define STEP_TIME   0.000997



namespace gazebo
{
	enum status_type{
		RIGTH_C,
		LEFT_C,
		TO_RIGTH,
		TO_LEFT,
		TO_HOME
	};

	class template_plugin : public ModelPlugin
	{
		// Pointer to the model
		private: physics::ModelPtr model;

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

		//private: math::Pose initialPose;
		private: double initialX, initialY, initialZ;
		private: double initialRoll, initialPitch, initialYaw;
		private: float angle_;
		private: float old_X, old_Y;
		private: int shape;
		private: float speed_;

		private: float X, Y;

		private: float step_, step_angle_;

		private: int status_;
		private: bool flag_12t;

    		private: struct timeval time_after, time_before;

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			// Store the pointer to the model
			this->model = _parent;

			angle_ = 0;

			math::Pose initialPose = this->model->GetWorldPose();
			initialX      = initialPose.pos[0];
			initialY      = initialPose.pos[1];
			initialZ      = initialPose.pos[2];
			initialRoll   = initialPose.pos[3];
			initialPitch  = initialPose.pos[4];
			initialYaw    = initialPose.pos[5];

			std::string shapeParam = _sdf->Get<std::string>("shape");
			std::transform(shapeParam.begin(), shapeParam.end(), shapeParam.begin(), ::tolower);
 
			if ( shapeParam == "circle")
				shape = CIRCLE;
			else 
				if ( shapeParam == "stop")
					shape = STOP;
				else 
					if ( shapeParam == "eight_simple")
						shape = EIGHT_SIMPLE;  
					else 
						shape = STOP;

			// step_ = 0.0005; //old step_ defined by Erle
			// speed_ = 4.167; // m/s, max speed
			speed_ = 2.167; // m/s, max speed

			step_ = STEP_TIME * speed_;  
 			step_angle_ = (step_  * 360) / (2 * M_PI *  RADIUS);

			
			// Initialize
			X = initialX;
			Y = initialY;

			status_ = TO_RIGTH;

			// get current time  
    			gettimeofday(&time_after, NULL);
    			time_before = time_after;
    			flag_12t = false;

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
					boost::bind(&template_plugin::OnUpdate, this, _1));
		}

		// Called by the world update start event
		public: void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			// printf("%f %f\n", step_angle_, step_ );
	        /// Time manager

	      	gettimeofday(&time_after, NULL);

	      	float t = ((time_after.tv_usec + (time_after.tv_sec - time_before.tv_sec) * 1000000) - time_before.tv_usec) / 1000.0;

			if (t >= 360000 ) { // 6 [min]

				time_before = time_after;

				// after 20 [min]
				if (speed_ <= 0)
					speed_ = 0;
				
				// challenge 1-20 [min]
				if (speed_ >  1.389 || flag_12t == true)
					speed_ -= 1.389;
				else
					flag_12t = true;

				// Set new steps
				
				step_ = STEP_TIME * speed_;  
				step_angle_ = (step_  * 360)  / (2 * M_PI *  RADIUS);

			}

			// --- 

			switch (shape)
			{
				case EIGHT_SIMPLE:{ // Simple without rotation and random position

					switch(status_){
						case TO_RIGTH:{
							X += step_ * 1.2;
							Y += step_ ;

							if (X > (17.4 + initialX)){
								status_ = RIGTH_C;

								angle_ = 120;

								old_X = RADIUS * cos(angle_  * M_PI / 180) ;
								old_Y = RADIUS * sin(angle_  * M_PI / 180);
								
							}
						} break;
						case RIGTH_C:{

							X += (RADIUS * cos(angle_  * M_PI / 180) - old_X);
							Y += (RADIUS * sin(angle_  * M_PI / 180) - old_Y);

							old_X = RADIUS * cos(angle_  * M_PI / 180) ;
							old_Y = RADIUS * sin(angle_  * M_PI / 180);
							
							angle_ -= step_angle_ ;

							if  ( X <= (17.4 + initialX))
								status_ = TO_LEFT;
						} break;
						case TO_LEFT:{
							X += step_ *  - 1.2;
							Y += step_ ;

							if (X < (-17.4 + initialX)){
								
								angle_ = 60;
								
								status_ = LEFT_C;

								old_X = RADIUS * cos(angle_  * M_PI / 180) ;
								old_Y = RADIUS * sin(angle_  * M_PI / 180);

							}
						}break;
						case LEFT_C:{

							X += (RADIUS * cos(angle_  *M_PI / 180) - old_X);
							Y += (RADIUS * sin(angle_  *M_PI / 180) - old_Y);

							old_X = RADIUS * cos(angle_  * M_PI / 180) ;
							old_Y = RADIUS * sin(angle_  * M_PI / 180);
							
							angle_ += step_angle_ ;

							if  ( X >= (-17.4 + initialX))
								status_ = TO_HOME;
						}break;
						case TO_HOME:{
							X += step_ * (X / Y);
							Y += step_ ;
							if (X > (0 + initialX))
								status_ = TO_RIGTH;
						} break;
						default: 
							break;
					}

					math::Pose pose = math::Pose(math::Vector3(X, Y, initialZ), 
										 math::Quaternion(math::Vector3(initialRoll, initialPitch, initialYaw)));
					this->model->SetWorldPose(pose);
				} break;

				case CIRCLE:
				{
					angle_ = angle_ + 0.0004;
					X = initialX + sin(angle_) * RADIUS;
					Y = initialY + cos(angle_) * RADIUS;
					
					math::Pose pose = math::Pose(math::Vector3(X,Y,initialZ), 
										 math::Quaternion(math::Vector3(initialRoll,initialPitch,initialYaw)));
					this->model->SetWorldPose(pose);
					
				} break; // end case CIRCLE
					
				default: // STOP
					break;  // end case STOP

			} // end switch shape

		} // end void OnUpdate

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(template_plugin)
} // end namespace GAZEBO
