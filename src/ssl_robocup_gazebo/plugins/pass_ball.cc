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
class KickBall : public ModelPlugin
{
  // Called by the world update start event
  public: void OnUpdate()
  {
      gazebo_msgs::GetModelState ball_position ;  
      ball_position.request.model_name = "ball";
      this->rosPositionSrv.call(ball_position);

      gazebo_msgs::GetModelState ally_position ;  
      ally_position.request.model_name = "turtlebot3";
      this->rosPositionSrv.call(ally_position);

      gazebo_msgs::GetModelState self_position;  
      self_position.request.model_name = this->model->GetName().c_str();
      this->rosPositionSrv.call(self_position);

      double ally_ball_X = ally_position.response.pose.position.x - ball_position.response.pose.position.x;
      double ally_ball_Y = ally_position.response.pose.position.y - ball_position.response.pose.position.y;
      double gradien = ally_ball_Y / ally_ball_X;

      double self_ball_X = 0.3 / std::sqrt(1 + gradien*gradien);

      double posX;

      if (ally_ball_X < 0) {
          posX = ball_position.response.pose.position.x + self_ball_X;
      } else {
          posX = ball_position.response.pose.position.x - self_ball_X;
      }

      double posY = ball_position.response.pose.position.y - gradien * (ball_position.response.pose.position.x - posX);

      this->model->SetLinearVel(ignition::math::Vector3d(posX - self_position.response.pose.position.x, posY - self_position.response.pose.position.y, 0));

      double distanceX = posX - self_position.response.pose.position.x;
      double distanceY = posY - self_position.response.pose.position.y;

      double absDistance = std::sqrt(std::pow(distanceX, 2) + std::pow(distanceY, 2) * 1.0);

      double velX = (ally_position.response.pose.position.x - ball_position.response.pose.position.x);
      double velY = (ally_position.response.pose.position.y - ball_position.response.pose.position.y);

      if (absDistance < 0.075) {
        ssl_robocup_gazebo::MoveBall move_ball;
        move_ball.request.target_model_name = "turtlebot3"; 
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
    this->ally = world->ModelByName("turtlebot3");

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "pass_ball",
        ros::init_options::NoSigintHandler);
    }

    this->rosNode.reset(new ros::NodeHandle("pass_ball"));

    this->rosPositionSrv = this->rosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    this->rosBallMoverSrv= this->rosNode->serviceClient<ssl_robocup_gazebo::MoveBall>("/kinetics/move_ball");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&KickBall::OnUpdate, this));
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
GZ_REGISTER_MODEL_PLUGIN(KickBall)
}
