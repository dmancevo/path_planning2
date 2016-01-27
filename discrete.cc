#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "A_star/A_star.h"
#include "A_star/load_map.h"

namespace gazebo
{
  class DiscreteNavigation : public ModelPlugin
  {

    public:
      std::string chassis = "chassis";
      unsigned long long int i = 0;
      unsigned int t = 0;
      std::vector<std::pair<int,int>> path;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&DiscreteNavigation::OnUpdate, this, _1));

      //path planner
      std::string file_path_to_map = "Maps/dicsObst.txt";
      std::vector<std::vector<bool>> map = loadObstacleMapFromTxt(file_path_to_map);
      A_star* path_planner = new A_star(map);
      path = path_planner->findPath(1,1,14,19,4);
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      
      i += 1;
      if(i % 1000 == 0 && t<path.size()){
        this->model->SetLinkWorldPose(math::Pose(path[t].first,
          path[t].second,0.25,0,0,0), chassis);
        t += 1;
      }
    }


    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DiscreteNavigation)
}