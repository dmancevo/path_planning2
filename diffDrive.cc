#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "DIFF_DRIVE/DDWPFollower.h"
#include "DIFF_DRIVE/load_dd_waypoint.h"
#include "DIFF_DRIVE/dd_structs.h"

namespace gazebo
{
    class DiffDriveNavigation : public ModelPlugin
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
                    boost::bind(&DiffDriveNavigation::OnUpdate, this, _1));

            //load waypoints
            this->wp = loadDiffDriveWaipoint("Maps/waypoint_dd.txt");


            //set position to the first point
            this->model->SetLinkWorldPose(math::Pose(this->wp.points[0].first, this->wp.points[0].second,height,0,0,0), chassis);

            this->ddwpFollower = DDWPFollower(0.001, wp.points, wp.theta_init, wp.vel_max, wp.w_max,
                                              true, {-1,11}, {-1,11});

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
    GZ_REGISTER_MODEL_PLUGIN(DiffDriveNavigation)
}