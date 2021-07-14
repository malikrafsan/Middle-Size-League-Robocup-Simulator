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

namespace gazebo
{
class AttachBall : public ModelPlugin 
{
    // Called by the world update start event
    public: void OnUpdate()
    {
        gazebo_msgs::GetModelState ball_position ;  
        ball_position.request.model_name = "ball";
        this->rosPositionSrv.call(ball_position);

        gazebo_msgs::GetModelState self_position;  
        self_position.request.model_name = this->model->GetName().c_str();
        this->rosPositionSrv.call(self_position);

        //find distance between ball and robot
        double dist_x = ally_position.response.pose.position.x - ball_position.response.pose.position.x;
        double dist_y = ally_position.response.pose.position.y - ball_position.response.pose.position.y;
        
        double dist_xy = std::sqrt(std::pow(dist_x,2) + std::pow(dist_y,2));

        if (dist_xy < 1) {
            
        }
    }
    
}
}
