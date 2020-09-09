#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Code for velodyne plugin turn around movement
namespace gazebo
{
	class VelodynePlugin : public ModelPlugin
	{
		public: VelodynePlugin() {}
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			if(_model->GetJointCount() == 0)
			{
				std::cerr << "\n Invalid joint count, Velodyne plugin not loaded \n ";
				return;
			}

			this->model = _model;
			this->joint = _model->GetJoints()[2];
			this->pid = common::PID(0.1,0,0);

			this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(),this->pid);
			this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(),10.0);

		}

		private: physics::ModelPtr model;
		private: physics::JointPtr joint;
		private: common::PID pid;
	};


	GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin);

}



#endif
