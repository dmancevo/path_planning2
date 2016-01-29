#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "A_star/A_star.h"
#include "A_star/load_map.h"

namespace gazebo
{
  class DynamicPointNavigation : public ModelPlugin
  {

    public:

      //Vehicle chassis.
      std::string chassis = "chassis";

      //Keep track of iteration and path step.
      // unsigned long long int i = 0;
      unsigned int t = 1;

      //Acceleration cap.
      float a_cap = 5.0;

      //Path to follow.
      int x [5] = {1,5,5,1,1};
      int y [5] = {1,1,5,5,1};
      // std::vector<std::pair<int,int>> path;
      math::Vector3 target;
      math::Vector3 p_target;
      math::Vector3 v_to_target;
      float d_to_target;

      //Keep track of current position.
      math::Vector3 position;

      //Keep track of current velocity.
      math::Vector3 velocity;

      //Keep track of acceleration.
      math::Vector3 acceleration;
      float alpha;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&DynamicPointNavigation::OnUpdate, this, _1));

      //path planner

    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {

      //Acquire position.
      math::Pose pose = this->model->GetWorldPose();
      position = pose.pos;

      //Set direction towards current target.
      p_target = math::Vector3(x[t-1],y[t-1],position.z);
      target = math::Vector3(x[t],y[t],position.z);
      v_to_target = target - position;
      d_to_target = position.Distance(target);

      //Update alpha
      alpha = -1 + 2*d_to_target/p_target.Distance(target);

      //Normalize, cap and set acceleration towards target.
      acceleration = a_cap * alpha * v_to_target.Normalize();

      //Acquire velocity.
      velocity = this->model->GetWorldLinearVel();

      //Update velicity according to acceleration.
      velocity = velocity + 0.0001*acceleration;
      this->model->SetLinearVel(velocity);

      //Update target when reached.
      if(d_to_target < 0.00001){
        t+=1;
        t = t % 5;
        if(t==0)
          t+=1;
      }
        
    }


    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DynamicPointNavigation)
}