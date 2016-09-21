#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>

using namespace std;

namespace gazebo
{

	class vacuum_gripper : public ModelPlugin
	{
		// Pointer to the model
		private: physics::ModelPtr parent_;
		private: physics::WorldPtr world_;
		private:  physics::LinkPtr link_;

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

		private: bool flag_touch;

		public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			// Store the pointer to the model
		  	parent_ = _model;

		  	// Get the world name.
		  	world_ = _model->GetWorld();

			math::Pose initialPose = parent_->GetWorldPose();
			initialX      = initialPose.pos[0];
			initialY      = initialPose.pos[1];
			initialZ      = initialPose.pos[2];
			initialRoll   = initialPose.pos[3];
			initialPitch  = initialPose.pos[4];
			initialYaw    = initialPose.pos[5];

			std::string str = "gripper_vacuum";
			flag_touch = false;

			link_ = _model->GetLink(str);
			
			if (!link_)
			{
				std::string found;
				physics::Link_V links = _model->GetLinks();
				
				for (size_t i = 0; i < links.size(); i++) 
					found += std::string(" ") + links[i]->GetName();
				
				cout << "gazebo_ros_vacuum_gripper plugin error: link named: %s does not exist" << str.c_str() << endl;
				cout << "gazebo_ros_vacuum_gripper plugin error: You should check it exists and is not connected with fixed joint" << endl;
				cout << "gazebo_ros_vacuum_gripper plugin error: Found links are: %s" <<  found.c_str() << endl;
				 return;
			}

			// if (!ros::isInitialized())
			// {
			// 	ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
			// 		<< "Load the Gazebo system plugin 'libgazebo_ros_api_+plugin.so' in the gazebo_ros package)");
			// 	return;
			// }

			 
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&vacuum_gripper::OnUpdate, this, _1));
		}
			
		

		// Called by the world update start event
		public: void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			cout << "alive" << endl;

			math::Pose parent_pose = link_->GetWorldPose();
			physics::Model_V models = world_->GetModels();
		  	
		  	for (size_t i = 0; i < models.size(); i++) {

		    		if (models[i]->GetName() == link_->GetName() || models[i]->GetName() == parent_->GetName())		
		      		continue;
		   
			    	physics::Link_V links = models[i]->GetLinks();

			    	for (size_t j = 0; j < links.size(); j++) {
			
			     		math::Pose link_pose = links[j]->GetWorldPose();
			      	math::Pose diff = parent_pose - link_pose;

			      	double norm = diff.pos.GetLength(); 

			      	if (norm < 2) {
			        		links[j]->SetLinearAccel(link_->GetWorldLinearAccel());
			        		links[j]->SetAngularAccel(link_->GetWorldAngularAccel());
			        		links[j]->SetLinearVel(link_->GetWorldLinearVel());
			        		links[j]->SetAngularVel(link_->GetWorldAngularVel());
			        		

			        		double norm_force = 1 / norm;

			        		if (norm < 0.15||  flag_touch) {
			        			flag_touch = true;
					          	link_pose.Set(parent_pose.pos, link_pose.rot);
					          	link_pose.pos.z -= 0.15;
					          	links[j]->SetWorldPose(link_pose);
					     	}
					     	else{
							if (norm_force > 20) 
								norm_force = 20;  // max_force
					        
							math::Vector3 force = norm_force * diff.pos.Normalize();
							links[j]->AddForce(force);
			      			
						}
			      	}else{
	
						math::Vector3 force(0,0,0);

						links[j]->AddForce(force);
					}
			      }
			}
		} // end void OnUpdate
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(vacuum_gripper)
} // end namespace GAZEBO
