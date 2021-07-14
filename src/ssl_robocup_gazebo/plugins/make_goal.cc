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
#include "ssl_robocup_gazebo/MoveBall.h"
#include "ssl_robocup_gazebo/MoveBallRequest.h"
#include "ssl_robocup_gazebo/MoveBallResponse.h"

namespace gazebo
{
class MakeGoal : public ModelPlugin
{
  // Called by the world update start event
  public: void OnUpdate()
  {
      gazebo_msgs::GetModelState ball_position ;  
      ball_position.request.model_name = "ball";
      this->rosPositionSrv.call(ball_position);

      gazebo_msgs::GetModelState self_position;  
      self_position.request.model_name = this->model->GetName().c_str();
      this->rosPositionSrv.call(self_position);

      double distanceX = ball_position.response.pose.position.x - self_position.response.pose.position.x;
      double distanceY = ball_position.response.pose.position.y - self_position.response.pose.position.y;

      double absDistance = std::sqrt(std::pow(distanceX, 2) + std::pow(distanceY, 2) * 1.0);

      if (absDistance < 0.075) {
        ssl_robocup_gazebo::MoveBall move_ball;
        move_ball.request.target_model_name = "robocup_ssl_left_goal"; 
        move_ball.request.origin_model_name = this->model->GetName().c_str();  
        this->rosBallMoverSrv.call(move_ball);
      }
  }

  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store the pointer to the model
    this->model = _parent;
    this->world = _parent->GetWorld();
    this->ball = world->ModelByName("ball");
    this->ally = world->ModelByName("robocup_ssl_left_goal");

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "make_goal",
        ros::init_options::NoSigintHandler);
    }

    this->rosNode.reset(new ros::NodeHandle("make_goal"));

    this->rosPositionSrv = this->rosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    this->rosBallMoverSrv= this->rosNode->serviceClient<ssl_robocup_gazebo::MoveBall>("/kinetics/move_ball");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MakeGoal::OnUpdate, this));
  }
  /// \brief A node use for ROS transport
  private: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief ROS service client
  private: ros::ServiceClient rosPositionSrv;
  private: ros::ServiceClient rosBallMoverSrv;

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

  private: physics::ModelPtr model;

  private: physics::WorldPtr world;
  
  private: physics::ModelPtr ball;

  private: physics::ModelPtr ally;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MakeGoal)
}

