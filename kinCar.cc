#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>
#define PI 3.14159265
#include "V_graph/V_graph.h"

namespace gazebo
{
  class KinematicCarNavigation : public ModelPlugin
  {

    public:

      //Vehicle chassis.
      std::string chassis = "chassis";

      //Keep track of path step.
      unsigned int t=0;

      //Car length
      double L = 0.1;

      //Initial orientation and target orientation
      double theta = 0.0;
      double target_theta;

      //Initial phi and bound
      double phi, bound;

      //Speed cap and start speed;
      float v_cap = 1.0;
      float v = 0.1;

      //Seconds per frame
      double delta = 0.001;

      //Path to follow.
      std::vector<std::pair<double,double> > path;

      //Current waypoint coordinates
      math::Vector3 waypoint;

      //Keep track of current position.
      math::Vector3 position;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&KinematicCarNavigation::OnUpdate, this, _1));

      //path planner
      V_graph v_graph("Maps/polygObst.txt", 1);
      path = v_graph.shortest_path();
      t = path.size()-2;

      position = math::Vector3(v_graph.start.first,v_graph.start.second,0.05);
      waypoint = math::Vector3(path[t].first,path[t].second,0.05);

    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {

      if(std::sqrt(std::pow(waypoint.x,2)+std::pow(waypoint.y,2)) < 0.2 && t>0){
        t--;
        waypoint = math::Vector3(path[t].first,path[t].second,0.05);
      }
      
      if(std::sqrt(std::pow(waypoint.x,2)+std::pow(waypoint.y,2))>0.2){
        //Update waypoint
        waypoint = math::Vector3(path[t].first-position.x,
          path[t].second-position.y,0.05);

        //Target orientation
        target_theta = std::atan(waypoint.y/waypoint.x);
        if(waypoint.x<0)
          target_theta -= PI;

        //Find optimal phi to match target orientation
        phi = 0;
        bound = 0.5;
        while(bound > 0.01){
          if(pow(theta+(v/L)*std::tan(phi+bound)-target_theta,2)<
            pow(theta+(v/L)*std::tan(phi)-target_theta,2))
            phi = phi+bound;
          else if(pow(theta+(v/L)*std::tan(phi-bound)-target_theta,2)<
            pow(theta+(v/L)*std::tan(phi)-target_theta,2))
            phi = phi-bound;

          bound /= 2.0;
        }

        //Update theta
        theta += delta * (v/L) * std::tan(phi);

        //Update speed
        v = 1-0.8*std::abs(phi);

        //Update position.
        position = math::Vector3(position.x + delta * v * std::cos(theta),
            position.y + delta * v * std::sin(theta),position.z);

        this->model->SetLinkWorldPose(math::Pose(position.x,
            position.y,position.z,0,0,theta), chassis);

      }
      
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(KinematicCarNavigation)
}