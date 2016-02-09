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
  class DynamicCarNavigation : public ModelPlugin
  {

    public:

      //Vehicle chassis.
      std::string chassis = "chassis";

      //Keep track of path step and time.
      unsigned int t=0;
      double elapsed_time=0.0;

      //Car length
      double L = 5.0;

      //Initial orientation and target orientation
      double theta = 0.0;
      double target_theta, dist_target;

      //Initial phi and bound
      double phi, bound;
      double phiMax = PI/4;

      //Initial speed and acceleration;
      float v = 0.0;
      float a = 2.0;

      //Seconds per frame
      double delta = 0.001;

      //Path to follow.
      std::vector<std::pair<double,double> > path;

      //Current waypoint coordinates
      math::Vector3 waypoint;

      //Keep track of current position.
      math::Vector3 position;

      //Visibility graph
      V_graph *v_graph = new V_graph("Maps/polygObstTest.txt", 2.5);

      //Currently back-tracking
      unsigned int back = 0;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&DynamicCarNavigation::OnUpdate, this, _1));

      //path planner
      path = v_graph->shortest_path();
      t = path.size()-2;

      position = math::Vector3(v_graph->start.first,v_graph->start.second,0.1);
      waypoint = math::Vector3(path[t].first,path[t].second,0.1);
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      
      if(std::sqrt(std::pow(waypoint.x,2)+std::pow(waypoint.y,2))>0.1){
        //Update waypoint
        waypoint = math::Vector3(path[t].first-position.x,
          path[t].second-position.y,0.1);

        //Target orientation
        target_theta = std::atan(waypoint.y/waypoint.x);
        if(waypoint.x<0)
          target_theta -= PI;

        //Find optimal phi to match target orientation
        phi = 0;
        bound = phiMax/2;
        while(bound > 0.01){
          if(pow(theta+(v/L)*std::tan(phi+bound)-target_theta,2)<
            pow(theta+(v/L)*std::tan(phi)-target_theta,2))
            phi = phi+bound;
          else if(pow(theta+(v/L)*std::tan(phi-bound)-target_theta,2)<
            pow(theta+(v/L)*std::tan(phi)-target_theta,2))
            phi = phi-bound;

          bound /= 2.0;
        }

        //Update acceleration
        dist_target = std::sqrt(std::sqrt(std::pow(waypoint.x,2)+std::pow(waypoint.y,2)));
        if(t>1){
          if(v=1.0)
            a = 0;
          else
            a = delta * 2.0;
        } else if(dist_target<2.0)
          a = delta * -2.0;

        //Backtrack to avoid obstacles
        if(back==0 && v_graph->validPath(std::make_pair(position.x,position.y),
         std::make_pair(position.x+0.3*v*std::cos(theta), position.y+0.3*v*std::sin(theta)))==1){
          back += 5000;
        }

        if (back>0){
          a *= -1;
          phi = 0;
          back--;
        }

        if(back>0 && v_graph->validPath(std::make_pair(position.x,position.y),
         std::make_pair(position.x+0.3*v*std::cos(theta), position.y+0.3*v*std::sin(theta)))==1){
          back=0;
        }

        //Update speed
        v += delta * a;

        //Update theta
        theta += delta * (v/L) * std::tan(phi);

        //Update position.
        position = math::Vector3(position.x + delta * v * std::cos(theta),
            position.y + delta * v * std::sin(theta),position.z);

        this->model->SetLinkWorldPose(math::Pose(position.x,
            position.y,position.z,0,0,theta), chassis);

        elapsed_time += delta;

      } else if(t>0){
          t--;
          waypoint = math::Vector3(path[t].first,path[t].second,0.1);
      } else
          std::cout<<"dynCar Time: "<<elapsed_time<<"\n";

      //Check if we can make a run for the next waypoint
      for(int j=0;j<t;j++){
        if(v_graph->validPath(std::make_pair(position.x,position.y),
           std::make_pair(path[j].first, path[j].second))==0){
           t=j;
           waypoint = math::Vector3(path[t].first,path[t].second,0.1);
           break;
        }
      }
      
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DynamicCarNavigation)
}