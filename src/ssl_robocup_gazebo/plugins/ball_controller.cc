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
#include "ssl_robocup_gazebo/MoveBall.h"
#include "ssl_robocup_gazebo/MoveBallRequest.h"
#include "ssl_robocup_gazebo/MoveBallResponse.h"
#include <std_srvs/Empty.h>

namespace gazebo
{
class BallController : public ModelPlugin
{
  public : BallController() :
    nh("kinetics")
  {
  }

  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store model pointer
    this->model = _parent;

    // Initialize ROS node
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "ball_controller",
        ros::init_options::NoSigintHandler);
    }

    // Advertise "move_ball" and "stop_ball" service 
    this->moveBallServicePublisher = this->nh.advertiseService("move_ball", &BallController::move_ball_callback, this);
    this->stopBallServicePublisher = this->nh.advertiseService("stop_ball", &BallController::stop_ball_callback, this);

    // Client to get_model_state service
    this->rosPositionSrvClient = this->nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  }

    public: bool stop_ball_callback(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res){
      ROS_INFO_STREAM("Received request to stop ball");

      // Set velocity of the ball to 0 in order to stop it
      this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
      return true;
    }

    public: bool move_ball_callback(ssl_robocup_gazebo::MoveBall::Request &req,
                                             ssl_robocup_gazebo::MoveBall::Response &res){
      ROS_INFO_STREAM("Received request to move ball  from model: '" << req.target_model_name 
                        << "' to model: '" <<  req.origin_model_name << "'");

      // Get position state of the ball
      gazebo_msgs::GetModelState ball_position ;  
      ball_position.request.model_name = "ball";
      this->rosPositionSrvClient.call(ball_position);

      // Get position state of the target
      gazebo_msgs::GetModelState target_position;  
      target_position.request.model_name = req.target_model_name;
      this->rosPositionSrvClient.call(target_position);

      // Get position state of the origin (kicker)
      gazebo_msgs::GetModelState origin_position;  
      origin_position.request.model_name = req.origin_model_name;
      this->rosPositionSrvClient.call(origin_position);

      // Calculate velocity needed for the ball to get to target
      double velX = (target_position.response.pose.position.x - ball_position.response.pose.position.x);
      double velY = (target_position.response.pose.position.y - ball_position.response.pose.position.y);

      // Set velocity of the ball to kick it
      this->model->SetLinearVel(ignition::math::Vector3d(velX*2, velY*2, 0));

      // Return response
      res.ok = true;
      return true;
  }

  // Attributes of this class
  private: ros::NodeHandle nh; 
  private: ros::ServiceClient rosPositionSrvClient;
  private: physics::ModelPtr model;
  private: ros::ServiceServer moveBallServicePublisher;
  private: ros::ServiceServer stopBallServicePublisher;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(BallController)
}
