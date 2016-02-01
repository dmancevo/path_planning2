#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "V_graph/V_graph.h"

namespace gazebo
{
  class KinematicPointNavigation : public ModelPlugin
  {

    public:

      //Vehicle chassis.
      std::string chassis = "chassis";

      //Keep track of iteration and path step.
      unsigned int t=0;
      double alpha=0;

      //Velocity cap.
      float v_cap = 4.0;

      //Path to follow.
      std::vector<std::pair<double,double> > path;
      math::Vector3 target;
      math::Vector3 departure;

      //Keep track of current position.
      math::Vector3 position;

      //Keep track of current velocity.
      math::Vector3 velocity;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&KinematicPointNavigation::OnUpdate, this, _1));

      //path planner
      V_graph v_graph("Maps/polyObst.txt", 0.1);
      path = v_graph.shortest_path();
      t=path.size()-1;
      departure = math::Vector3(path[t].first,path[t].second,0.05);
      target = math::Vector3(path[t-1].first,path[t-1].second,0.05);
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {

      if(position.Distance(target) < 0.01 && t>0){
        departure = math::Vector3(path[t].first,path[t].second,0.05);
        target = math::Vector3(path[t-1].first,path[t-1].second,0.05);
        t--;
        alpha=0;
      }
      
      if(position.Distance(target) > 0.01){
        //Update position.
        position = departure * (1-alpha/2000) + target * (alpha/2000);
        this->model->SetLinkWorldPose(math::Pose(position.x,
            position.y,position.z,0,0,0), chassis);

        alpha +=1;
      }
      
    }


    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(KinematicPointNavigation)
}