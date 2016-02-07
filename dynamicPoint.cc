#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "DIMT_RRT/dmit.h"
#include "DIMT_RRT/load_waypoint.h"
#include "DIMT_RRT/WaypointDynamicFollower.h"
#include "V_graph/V_graph.h"

namespace gazebo
{
    class DynamicPointNavigation : public ModelPlugin
    {

    public:
        std::string chassis = "chassis"; //Vehicle chasis.
        dynamicWaypoint wp;
        WaypointDynamicFollower wpf;
        V_graph *vis_graph = new V_graph("Maps/polyObst.txt", 0.1);
        std::pair<double, double> start;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Store the pointer to the model
            this->model = _parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&DynamicPointNavigation::OnUpdate, this, _1));


            std::pair<double,double> start = this->vis_graph->start;
            std::vector<std::pair<double,double> > shortest_path = this->vis_graph->shortest_path();
            std::reverse(shortest_path.begin(), shortest_path.end());


            //set position to the first point
            this->model->SetLinkWorldPose(math::Pose(start.first, start.second,0.25,0,0,0), chassis);

            //get best trajectory
            std::pair<std::vector<segment>,std::vector<segment>> segmentTrajectory =
                    waypointFollowingWithStops2(shortest_path, {0,0}, {0,0}, 0.1, 0.01);

            this->wpf = WaypointDynamicFollower(start, {0,0}, segmentTrajectory, 0.001);
        }

        // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
        {
            std::pair<double,double> new_coord = wpf.getNextCoordinates();
            this->model->SetLinkWorldPose(math::Pose(new_coord.first,new_coord.second,0.25,0,0,0), chassis);
        }

        // Pointer to the model
    private: physics::ModelPtr model;

        // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(DynamicPointNavigation)
}