#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "A_star/A_star.h"
#include "A_star/load_map.h"

namespace gazebo
{
  class KinematicPointNavigation : public ModelPlugin
  {

    public:

      //Vehicle chasis.
      std::string chassis = "chassis";

      //Keep track of iteration and time.
      unsigned long long int i = 0;
      unsigned int t = 0;

      //Velocity cap.
      float v_cap = 4.0;

      //Path to follow.
      int x [10] = {1,2,4,6,8,5,5,4,3,1};
      int y [10] = {1,6,8,5,5,4,3,1,4,3};
      math::Vector3 target;
      math::Vector3 v_to_target;
      // std::vector<std::pair<int,int>> path;

      //Keep track of current position.
      // double x_pose;
      // double y_pose;
      math::Vector3 position;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&KinematicPointNavigation::OnUpdate, this, _1));

      //path planner

    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // x_pose = pose.pos.x;
      // y_pose = pose.pos.y;
      // if(i==0){
      //   x_pose += 10;
      //   this->model->SetLinearVel(math::Vector3(x_pose, y_pose, 0));
      // }

      if(i % 100){

        //Acquire position and set direction towards current target.
        math::Pose pose = this->model->GetWorldPose();
        position = pose.pos;
        target = math::Vector3(x[t],y[t],position.z);
        v_to_target = target - position;

        //Normalize, cap and set velocity towards target.
        v_to_target = v_cap * v_to_target.Normalize();

        this->model->SetLinearVel(v_to_target);

        if(position.Distance(target) < 1)
          t+=1;
      }

      i+=1;
        
    }


    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(KinematicPointNavigation)
}