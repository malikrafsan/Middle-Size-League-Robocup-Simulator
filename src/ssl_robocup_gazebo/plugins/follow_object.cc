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
class SetOrient : public WorldPlugin 
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store the pointer to the model
    this->world = _parent;

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "kinetics",
        ros::init_options::NoSigintHandler);
    }

    this->rosNode.reset(new ros::NodeHandle("kinetics"));

    this->rosPositionSrv = this->rosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    this->setOrientPublisher = this->rosNode->advertiseService("set_orient_n_chase", &SetOrient::set_orient_callback, this);
  }

    public: bool set_orient_callback(ssl_robocup_gazebo::SetOrient::Request &req, ssl_robocup_gazebo::SetOrient::Response &res){
      gazebo_msgs::GetModelState target_state;  
      target_state.request.model_name = req.target_model_name;
      this->rosPositionSrv.call(target_state);

      gazebo_msgs::GetModelState self_state;  
      self_state.request.model_name = req.origin_model_name;
      this->rosPositionSrv.call(self_state);
      this->origin = world->ModelByName(req.origin_model_name);

      double x_difference = target_state.response.pose.position.x-self_state.response.pose.position.x; 
      double y_difference = target_state.response.pose.position.y-self_state.response.pose.position.y;
      double magnitudeDist = std::sqrt(std::pow(x_difference,2) + std::pow(y_difference,2));

      double velBotX = self_state.response.twist.linear.x;
      double velBotY = self_state.response.twist.linear.y;
      double magnitudeVel = std::sqrt(std::pow(velBotX,2) + std::pow(velBotY,2));

      double angle = std::atan2(-y_difference, -x_difference);

      tf::Quaternion q(
          self_state.response.pose.orientation.x,
          self_state.response.pose.orientation.y,
          self_state.response.pose.orientation.z,
          self_state.response.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
  
      double difOrient = angle - yaw;

      if (difOrient > 0.03) {
          this->origin->SetAngularVel(ignition::math::Vector3d(0, 0, 1));
      }
      else if(difOrient < -0.03){
          this->origin->SetAngularVel(ignition::math::Vector3d(0, 0, -1));
      } else {
          this->origin->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
      }
      if (magnitudeDist <= magnitudeVel) {
          this->origin->SetLinearVel(ignition::math::Vector3d(x_difference, y_difference, 0));
      } else {
          double forceStrength = 3.0;
          double normalization = forceStrength / magnitudeDist;
          this->origin->GetLink(req.origin_link_name)->SetForce(ignition::math::Vector3d(100*normalization*x_difference, 100*normalization*y_difference, 0));  
      }
      res.ok = true;
      return true;
    }

  /// \brief A node use for ROS transport
  private: std::unique_ptr<ros::NodeHandle> rosNode;

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
GZ_REGISTER_WORLD_PLUGIN(SetOrient)
}

