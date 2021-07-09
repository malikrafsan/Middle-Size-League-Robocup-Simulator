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

namespace gazebo
{
class MoveBall : public WorldPlugin
{

  public : MoveBall() :
    nh("kinetics")
  {
  }

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    this->world = _parent;
    this->ball = world->ModelByName("ball");

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "move_ball",
        ros::init_options::NoSigintHandler);
    }

    this->shootingServicePublisher = this->nh.advertiseService("move_ball", &MoveBall::move_ball_callback, this);

    this->rosPositionSrvClient = this->nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  }

    public: bool move_ball_callback(ssl_robocup_gazebo::MoveBall::Request &req,
                                             ssl_robocup_gazebo::MoveBall::Response &res){
      ROS_INFO_STREAM("Received request to move ball  from model: '" << req.target_model_name 
                        << "' to model: '" <<  req.origin_model_name << "'");

      gazebo_msgs::GetModelState ball_position ;  
      ball_position.request.model_name = "ball";
      this->rosPositionSrvClient.call(ball_position);

      gazebo_msgs::GetModelState target_position;  
      target_position.request.model_name = req.target_model_name;
      this->rosPositionSrvClient.call(target_position);

      gazebo_msgs::GetModelState origin_position;  
      origin_position.request.model_name = req.origin_model_name;
      this->rosPositionSrvClient.call(origin_position);

      double velX = (target_position.response.pose.position.x - ball_position.response.pose.position.x);
      double velY = (target_position.response.pose.position.y - ball_position.response.pose.position.y);

      this->ball->SetLinearVel(ignition::math::Vector3d(velX, velY, 1.5));

      res.ok = true;

      return true;
  }


  private: ros::NodeHandle nh; 

  private: ros::ServiceClient rosPositionSrvClient;
  private: ros::ServiceClient resetWorldSrv;

  private: physics::WorldPtr world;
  private: physics::ModelPtr ball;

  private: ros::ServiceServer shootingServicePublisher;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(MoveBall)
}
