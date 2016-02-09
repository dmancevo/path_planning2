#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "V_graph/V_graph.h"

namespace gazebo
{
  class KinematicPointNavigation : public ModelPlugin
  {

    public:

      //Vehicle chassis.
      std::string chassis = "chassis";

      //Keep track of path step and time.
      unsigned int t=0;
      double elapsed_time=0.0;

      //Seconds per frame.
      double delta=0.001;

      //Path to follow.
      std::vector<std::pair<double,double> > path;

      //Keep track of current position and current waypoint.
      math::Vector3 position;
      math::Vector3 waypoint;

      //Keep track of current velocity.
      math::Vector3 velocity;
      double vx, vy, norm;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&KinematicPointNavigation::OnUpdate, this, _1));

      //path planner
      V_graph v_graph("Maps/polygObstTest.txt", 0.1);
      path = v_graph.shortest_path();
      t=path.size()-2;

      position = math::Vector3(v_graph.start.first,v_graph.start.second,0.05);
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {

      if(std::sqrt(std::pow(path[t].first - position.x,2)+std::pow(path[t].second - position.y,2))>0.1){

          //Update velocity
          vx = path[t].first - path[t+1].first;
          vy = path[t].second - path[t+1].second;
          norm = std::sqrt(std::pow(vx,2)+std::pow(vy,2));

          vx = 1.0 * vx/norm;
          vy = 1.0 * vy/norm;

          velocity = math::Vector3(vx,vy,0);

          //Update position.
          position = math::Vector3(position.x + delta * velocity.x,
              position.y + delta * velocity.y, position.z);

          this->model->SetLinkWorldPose(math::Pose(position.x,
              position.y,position.z,0,0,0), chassis);

          elapsed_time += delta;

        } else if(t>0)
          t--;
          else
            std::cout<<"kinPoint Time: "<<elapsed_time<<"\n";
          
    }


    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(KinematicPointNavigation)
}