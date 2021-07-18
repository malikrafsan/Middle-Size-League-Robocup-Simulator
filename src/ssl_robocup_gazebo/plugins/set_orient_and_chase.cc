#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/GetModelState.h> 
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "gazebo/physics/physics.hh"
#include <ignition/math/Vector3.hh>
#include <math.h>
#include <cmath>
#include "tf/transform_datatypes.h"
#include "LinearMath/btMatrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include "ssl_robocup_gazebo/SetOrient.h"
#include "ssl_robocup_gazebo/SetOrientRequest.h"
#include "ssl_robocup_gazebo/SetOrientResponse.h"

namespace gazebo
{
class Chase : public ModelPlugin
{
  public: void OnUpdate()
  {
      // Call service to chase the ball
      ssl_robocup_gazebo::SetOrient body;
      body.request.origin_model_name = this->model->GetName().c_str();
      body.request.origin_link_name = "rack";
      body.request.target_model_name = "ball";
      this->rosChaserSrv.call(body);
  }

  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store model pointer and world pointer
    this->model = _parent;
    this->world = _parent->GetWorld();

    // Initialize ROS node
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "pass_ball",
        ros::init_options::NoSigintHandler);
    }
    this->rosNode.reset(new ros::NodeHandle("pass_ball"));

    // Client to services needed of this plugin
    this->rosPositionSrv = this->rosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    this->rosChaserSrv = this->rosNode->serviceClient<ssl_robocup_gazebo::SetOrient>("/kinetics/set_orient_n_chase");

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Chase::OnUpdate, this));
  }

  // ATTRIBUTES
  private: std::unique_ptr<ros::NodeHandle> rosNode;
  private: ros::ServiceClient rosPositionSrv;
  private: ros::ServiceClient rosChaserSrv;
  private: event::ConnectionPtr updateConnection;
  private: physics::ModelPtr model;
  private: physics::WorldPtr world;
  private: physics::ModelPtr ball;
  private: physics::ModelPtr ally;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Chase)
}
