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

namespace gazebo
{
class SetOrient : public ModelPlugin
{
  // Called by the world update start event
  public: void OnUpdate()
  {
      gazebo_msgs::GetModelState ball_state ;  
      ball_state.request.model_name = "ball";
      this->rosPositionSrv.call(ball_state);

      gazebo_msgs::GetModelState self_state;  
      self_state.request.model_name = this->model->GetName().c_str();
      this->rosPositionSrv.call(self_state);

      double x_difference = ball_state.response.pose.position.x-self_state.response.pose.position.x; 
      double y_difference = ball_state.response.pose.position.y-self_state.response.pose.position.y;
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

      if (difOrient > 0.03 || difOrient < -0.03) {
          this->model->SetAngularVel(ignition::math::Vector3d(0, 0, difOrient));
      } else {
          if (magnitudeDist <= magnitudeVel) {
              this->model->SetLinearVel(ignition::math::Vector3d(x_difference, y_difference, 0));
          } else {
              double forceStrength = 3.0;
              double normalization = forceStrength / magnitudeDist;
              this->model->GetLink("rack")->SetForce(ignition::math::Vector3d(normalization * x_difference, normalization * y_difference, 0));  
          }
      }
  }

  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store the pointer to the model
    this->model = _parent;
    this->world = _parent->GetWorld();
    this->ball = world->ModelByName("ball");

    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "pass_ball",
        ros::init_options::NoSigintHandler);
    }

    this->rosNode.reset(new ros::NodeHandle("pass_ball"));

    this->rosPositionSrv = this->rosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SetOrient::OnUpdate, this));
  }
  /// \brief A node use for ROS transport
  private: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS service client
  private: ros::ServiceClient rosPositionSrv;

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

  private: physics::ModelPtr model;

  private: physics::WorldPtr world;
  
  private: physics::ModelPtr ball;

  private: physics::ModelPtr ally;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SetOrient)
}
