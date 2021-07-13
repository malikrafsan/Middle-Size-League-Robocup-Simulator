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
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <iostream>
#include <math.h>


namespace gazebo
{
  class FollowBall : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->checkSame1 =true;

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
        geometry_msgs::Quaternion current_rotation = self_position.response.pose.orientation;
        tf2::Quaternion quat_tf_ball;
        geometry_msgs::Quaternion quat_msg_ball;


        tf2::Quaternion quat_tf_self;
        tf2::convert(current_rotation, quat_tf_self);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat_tf_self).getRPY(roll,pitch,yaw);
        quat_tf_ball.setRPY(0, 0, atan(x_difference/y_difference)+yaw);
        tf2::convert(quat_tf_ball, quat_msg_ball);
        std::cout << "[KONDISI]" << std::endl;
        std::cout << current_rotation.x << std::endl <<
        current_rotation.y << std::endl << current_rotation.z <<
        std::endl << current_rotation.w << std::endl << std::endl;


        std::cout << "[TUJUAN]" << std::endl;
        std::cout << quat_msg_ball.x << std::endl <<
        quat_msg_ball.y << std::endl << quat_msg_ball.z <<
        std::endl << quat_msg_ball.w << std::endl << std::endl;

        double error = 0.000435;

        if((std::fabs(quat_msg_ball.w-current_rotation.w) < error) && (std::fabs(quat_msg_ball.z-current_rotation.z) < error)){
          this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
          this->checkSame1 = false;
        } else if(this->checkSame1) {
          this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 1));
        }
   }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS service client
    private: ros::ServiceClient rosPositionSrv;

    private: bool checkSame1;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(FollowBall)
}