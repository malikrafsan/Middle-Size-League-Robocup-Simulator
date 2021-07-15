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
#include "ssl_robocup_gazebo/TestMsg.h"

namespace gazebo
{
class GamePlugin : public WorldPlugin 
{
  public : GamePlugin() :
    nh("game_plugin")
  {
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

    this->rosPub = this->nh.advertise<ssl_robocup_gazebo::TestMsg>("test",100);

    this->rosQueueThread =
    std::thread(std::bind(&GamePlugin::QueueThread, this));
  }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->nh.ok())
        {
            ssl_robocup_gazebo::TestMsg test_msg;
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
            test_msg.payload = "hello";
            this->rosPub.publish(test_msg);
        }
    }
  

  /// \brief A node use for ROS transport
  private: ros::NodeHandle nh; 

  /// \brief A ROS subscriber
  private: ros::Publisher rosPub;

  /// \brief A ROS callbackqueue that helps process messages
  private: ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
  private: std::thread rosQueueThread;

  /// \brief A ROS service client
  private: ros::ServiceClient rosPositionSrv;

  /// \brief A ROS service publisher 
  private: ros::ServiceServer setOrientPublisher;

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

  private: physics::ModelPtr origin;

  private: physics::WorldPtr world;
  
  private: physics::ModelPtr ball;

  private: physics::ModelPtr ally;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GamePlugin)
}

