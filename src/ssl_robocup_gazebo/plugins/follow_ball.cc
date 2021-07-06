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


namespace gazebo
{
  class FollowBall : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "follow_ball",
            ros::init_options::NoSigintHandler);
        }

        // Create our ROS node. This acts in a similar manner to
        // the Gazebo node
        this->rosNode.reset(new ros::NodeHandle("follow_ball"));

        this->rosPositionSrv = this->rosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&FollowBall::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
        gazebo_msgs::GetModelState ball_position ;  
        ball_position.request.model_name = "ball";
        this->rosPositionSrv.call(ball_position);

        gazebo_msgs::GetModelState self_position;  
        self_position.request.model_name = this->model->GetName().c_str();
        this->rosPositionSrv.call(self_position);

        float x_difference = ball_position.response.pose.position.x-self_position.response.pose.position.x; 
        float y_difference = ball_position.response.pose.position.y-self_position.response.pose.position.y; 
        float z_difference = ball_position.response.pose.position.z-self_position.response.pose.position.z; 
        this->model->SetLinearVel(ignition::math::Vector3d(x_difference, y_difference, 0));
   }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS service client
    private: ros::ServiceClient rosPositionSrv;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(FollowBall)
}
