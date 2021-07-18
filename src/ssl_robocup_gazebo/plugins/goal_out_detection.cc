#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/GetModelState.h> 
#include <gazebo_msgs/SetModelState.h> 
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
#include <gazebo/common/common.hh>
#include "ssl_robocup_gazebo/GameMessage.h"

namespace gazebo
{
class ResetWorld : public WorldPlugin
{

  

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

    this->rosGetPositionSrv = this->rosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    this->rosSetPositionSrv = this->rosNode->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::SubscribeOptions so =
    ros::SubscribeOptions::create<ssl_robocup_gazebo::GameMessage>(
        "/game_plugin/game_info",
        1,
        boost::bind(&ResetWorld::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->rosGamePluginSub = this->rosNode->subscribe(so);

    this->rosQueueThread =
    std::thread(std::bind(&ResetWorld::QueueThread, this));
  }

  public: void OnRosMsg(const ssl_robocup_gazebo::GameMessageConstPtr &_msg){
      gazebo_msgs::GetModelState ball_position ;  
      ball_position.request.model_name = "ball";
      this->rosGetPositionSrv.call(ball_position);

      if (ball_position.response.pose.position.x > 4.5 || ball_position.response.pose.position.x < -4.5 || 
      ball_position.response.pose.position.y > 3 || ball_position.response.pose.position.y < -3)
      {
          this->previous_time = world->RealTime();
          std_srvs::Empty rosEmptySrv;
          ros::service::call("/gazebo/reset_world", rosEmptySrv);
          if(_msg->ball_holder.find("A") != std::string::npos){
            while(true){
              if ((world->RealTime() - this->previous_time) > 0.1)
              {
                ros::service::call("/gazebo/pause_physics", rosEmptySrv);
                gazebo_msgs::SetModelState new_position; 
                new_position.request.model_state.model_name = "A_robot1";
                new_position.request.model_state.pose.position.x = -2;
                new_position.request.model_state.pose.orientation.z = 3.14;
                this->rosSetPositionSrv.call(new_position);
                new_position.request.model_state.model_name = "B_robot1";
                new_position.request.model_state.pose.position.x = -0.5;
                new_position.request.model_state.pose.orientation.z = 3.14;
                this->rosSetPositionSrv.call(new_position);
                std::cout << new_position.response.success << std::endl;
                this->previous_time = world->RealTime();          
                ros::service::call("/gazebo/unpause_physics", rosEmptySrv);
              }
            }
           } else {
              while(true){
                if ((world->RealTime() - this->previous_time) > 0.1)
                {
                  ros::service::call("/gazebo/pause_physics", rosEmptySrv);
                  gazebo_msgs::SetModelState new_position; 
                  new_position.request.model_state.model_name = "B_robot1";
                  new_position.request.model_state.pose.position.x = 2;
                  this->rosSetPositionSrv.call(new_position);
                  new_position.request.model_state.model_name = "A_robot1";
                  new_position.request.model_state.pose.position.x = 0.5;
                  this->rosSetPositionSrv.call(new_position);
                  std::cout << new_position.response.success << std::endl;
                  this->previous_time = world->RealTime();          
                  ros::service::call("/gazebo/unpause_physics", rosEmptySrv);
                  break;
                }
              }
         }
      }
  
  }

  private: void QueueThread()
  {
  static const double timeout = 20;
  while (this->rosNode->ok())
    {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  private: std::string ballHolder;

  /// \brief A node use for ROS transport
  private: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS service client
  private: ros::ServiceClient rosGetPositionSrv;
  private: ros::ServiceClient rosSetPositionSrv;
  private: ros::ServiceClient rosEmptySrv;
  private: common::Time previous_time;


  private: ros::Subscriber rosGamePluginSub;

  /// \brief A ROS callbackqueue that helps process messages
  private: ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
  private: std::thread rosQueueThread;

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

  private: physics::WorldPtr world;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ResetWorld)
}
