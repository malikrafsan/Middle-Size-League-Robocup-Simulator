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
#include "ssl_robocup_gazebo/GameMessage.h"
#include "ssl_robocup_gazebo/FindDistance.h"
#include "ssl_robocup_gazebo/FindDistanceRequest.h"
#include "ssl_robocup_gazebo/FindDistanceResponse.h"

namespace gazebo
{
class GamePlugin : public WorldPlugin 
{
  public : GamePlugin() :
    nh("game_plugin")
  {
    this->ball_holder = "None";
  }

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store the pointer to the model
    this->world = _parent;

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "game_plugin",
        ros::init_options::NoSigintHandler);
    }

    this->rosPub = this->nh.advertise<ssl_robocup_gazebo::GameMessage>("game_info",100);
    this->rosDistanceSrv = this->nh.serviceClient<ssl_robocup_gazebo::FindDistance>("/kinetics/calculateDist");

    this->rosQueueThread =
    std::thread(std::bind(&GamePlugin::QueueThread, this));
  }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->nh.ok())
        {
            ssl_robocup_gazebo::GameMessage game_message;
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
            game_message.ball_holder= this->ball_holder.c_str();
            game_message.model_at_goal_radius.A_robot1 = this->checkRadiusToGoal("turtlebot1");
            game_message.model_at_goal_radius.A_robot2 = this->checkRadiusToGoal("turtlebot2");
            game_message.model_at_goal_radius.A_robot3 = this->checkRadiusToGoal("turtlebot3");
            this->rosPub.publish(game_message);
        }
    }

    private: bool checkRadiusToGoal(std::string model_name){
      ssl_robocup_gazebo::FindDistance distance_msg; 
      distance_msg.request.target_model_name = "robocup_ssl_left_goal";
      distance_msg.request.origin_model_name = model_name;
      double distanceToGoal = this->rosDistanceSrv.call(distance_msg);
      if(distance_msg.response.distance < 1.75){
        return true;
      }
      return false;
    }
  

  /// \brief A node use for ROS transport
  private: ros::NodeHandle nh; 

  private: ros::ServiceClient rosDistanceSrv;

  private: ros::Publisher rosPub;

  /// \brief A ROS callbackqueue that helps process messages
  private: ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
  private: std::thread rosQueueThread;


  /// \brief A ROS service publisher 
  private: ros::ServiceServer setOrientPublisher;

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

  private: physics::ModelPtr origin;

  private: physics::WorldPtr world;
  
  private: std::string ball_holder;

  private: physics::ModelPtr ally;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GamePlugin)
}

