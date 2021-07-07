#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/GetModelState.h> 
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "gazebo/physics/physics.hh"
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

namespace gazebo
{
class ResetWorld : public WorldPlugin
{
  // Called by the world update start event
  public: void OnUpdate()
  {
      gazebo_msgs::GetModelState ball_position ;  
      ball_position.request.model_name = "ball";
      this->rosPositionSrv.call(ball_position);

      if (ball_position.response.pose.position.x > 4.5 || ball_position.response.pose.position.x < -4.5 || 
      ball_position.response.pose.position.y > 3 || ball_position.response.pose.position.y < -3)
      {
          std_srvs::Empty resetWorldSrv;
          ros::service::call("/gazebo/reset_world", resetWorldSrv);
      }
  }

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store the pointer to the model
    this->world = _parent;

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "reset_world",
        ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("reset_world"));

    this->rosPositionSrv = this->rosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ResetWorld::OnUpdate, this));
  }
  /// \brief A node use for ROS transport
  private: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS service client
  private: ros::ServiceClient rosPositionSrv;
  private: ros::ServiceClient resetWorldSrv;

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

  private: physics::WorldPtr world;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ResetWorld)
}
