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
#include <math.h>
#include "ssl_robocup_gazebo/MoveBall.h"
#include "ssl_robocup_gazebo/MoveBallRequest.h"
#include "ssl_robocup_gazebo/MoveBallResponse.h"
#include "ssl_robocup_gazebo/Attach.h"
#include "ssl_robocup_gazebo/AttachRequest.h"
#include "ssl_robocup_gazebo/AttachResponse.h"


// ini buat otomatisasi attach detach, bagian yang stop di certain radius dari gawang

namespace gazebo{
  class MoveToGoal : public ModelPlugin{
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/){
      // Store the pointer to the model
      this->model = _parent;

        if (!ros::isInitialized()){
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "move_to_goal",
            ros::init_options::NoSigintHandler);
        }

        // Create ROS node
        this->rosNode.reset(new ros::NodeHandle("move_to_goal"));
        this->rosPositionSrv = this->rosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        this->rosBallMoverSrv= this->rosNode->serviceClient<ssl_robocup_gazebo::MoveBall>("/kinetics/move_ball");
        this->rosLinkAttachSrv = this->rosNode->serviceClient<ssl_robocup_gazebo::Attach>("/link_attacher_node/attach");
        this->rosLinkDetachSrv = this->rosNode->serviceClient<ssl_robocup_gazebo::Attach>("/link_attacher_node/detach");

        
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MoveToGoal::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate(){
        // goal position
        gazebo_msgs::GetModelState ball_pos;
        ball_pos.request.model_name = "ball";
        this->rosPositionSrv.call(ball_pos);
  
        gazebo_msgs::GetModelState goal_pos;  
        goal_pos.request.model_name = "robocup_ssl_left_goal"; // ini masih asumsi dia goalnya gawang yang kiri
        this->rosPositionSrv.call(goal_pos);

        // robot position
        gazebo_msgs::GetModelState self_pos;  
        self_pos.request.model_name = this->model->GetName().c_str();
        this->rosPositionSrv.call(self_pos);

        // x and y difference between goal and robot position
        double x_diff = goal_pos.response.pose.position.x - self_pos.response.pose.position.x;
        double y_diff = goal_pos.response.pose.position.y - self_pos.response.pose.position.y;

        // distance between goal and robot
        double dist = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));

        double dist_x = self_pos.response.pose.position.x - ball_pos.response.pose.position.x;
        double dist_y = self_pos.response.pose.position.y - ball_pos.response.pose.position.y;
        
        double dist_xy = std::sqrt(std::pow(dist_x,2) + std::pow(dist_y,2));

        bool have_ball = false;

        if(dist_xy < 0.5){
            ssl_robocup_gazebo::Attach attach;
            attach.request.model_name_1 = "ball";
            attach.request.link_name_1 = "ball";
            attach.request.model_name_2 = this->model->GetName().c_str();
            attach.request.link_name_2 = "rack";
            this->rosLinkAttachSrv.call(attach);

            have_ball = true;
        }

        // stop at certain distance
        if (dist < 1.75){
          this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));

          if(have_ball){
            ssl_robocup_gazebo::Attach detach;
            detach.request.model_name_1 = "ball";
            detach.request.link_name_1 = "ball";
            detach.request.model_name_2 = this->model->GetName().c_str();
            detach.request.link_name_2 = "rack";
            this->rosLinkDetachSrv.call(detach);

            ssl_robocup_gazebo::MoveBall move_ball;
            move_ball.request.target_model_name = "robocup_ssl_left_goal"; 
            move_ball.request.origin_model_name = this->model->GetName().c_str();  
            this->rosBallMoverSrv.call(move_ball);
          }

        }
        else{
          this->model->SetLinearVel(ignition::math::Vector3d(x_diff, y_diff, 0));

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
    private: ros::ServiceClient rosBallMoverSrv;
    private: ros::ServiceClient rosLinkAttachSrv;
    private: ros::ServiceClient rosLinkDetachSrv;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MoveToGoal)
}