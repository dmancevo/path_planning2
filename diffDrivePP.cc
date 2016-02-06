#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "DIFF_DRIVE/DDWPFollower.h"
#include "DIFF_DRIVE/load_dd_waypoint.h"
#include "DIFF_DRIVE/dd_structs.h"
#include "V_graph/V_graph.h"

namespace gazebo
{
    class DiffDrivePPNavigation : public ModelPlugin
    {

    public:
        std::string chassis = "chassis"; //Vehicle chasis.
        diffDriveWP wp;
        DDWPFollower ddwpFollower;
        double height = 0.16;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Store the pointer to the model
            this->model = _parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&DiffDrivePPNavigation::OnUpdate, this, _1));

            V_graph vis_graph = V_graph("Maps/polyObst.txt", 0.0001);
            //load waypoints
            //this->wp = loadDiffDriveWaipoint("Maps/waypoint_dd.txt");


            //set position to the first point
            std::pair<double,double> start = vis_graph.start;

            std::vector<std::pair<double,double> > shortest_path = vis_graph.shortest_path();
            std::reverse(shortest_path.begin(), shortest_path.end());

            this->model->SetLinkWorldPose(math::Pose(start.first, start.second, height,0,0,0), chassis);

            this->ddwpFollower = DDWPFollower(0.001, shortest_path, 0, 0.1, 1, true, {0,10}, {0,10});

        }

        // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
        {
            diffDrivePose pose = ddwpFollower.getNextPose();
            this->model->SetLinkWorldPose(math::Pose(pose.x, pose.y, height,0,0,pose.theta), chassis);
        }

        // Pointer to the model
    private: physics::ModelPtr model;

        // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(DiffDrivePPNavigation)
}