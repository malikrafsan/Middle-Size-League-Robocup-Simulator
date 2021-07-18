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
#include "ssl_robocup_gazebo/FindDistance.h"
#include "ssl_robocup_gazebo/FindDistanceRequest.h"
#include "ssl_robocup_gazebo/FindDistanceResponse.h"

namespace gazebo
{
    class FindDistance: public WorldPlugin
    {
        // ATTRIBUTES
        private: physics::WorldPtr world;
        private: physics::ModelPtr origin;
        private: physics::ModelPtr target;
        private: std::unique_ptr<ros::NodeHandle> rosNode;
        private: ros::ServiceClient rosPositionSrv;
        private: ros::ServiceServer distancePublisher;

        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr)
        {
            // Store world pointer
            this->world = _parent;

            // Initialize ROS node
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "kinetics",
                ros::init_options::NoSigintHandler);
            }
            this->rosNode.reset(new ros::NodeHandle("kinetics"));
            
            // Client to get_model_state service
            this->rosPositionSrv = this->rosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
            
            // Advertise calculateDist service
            this->distancePublisher = this->rosNode->advertiseService("calculateDist", &FindDistance::calculateDist, this);
        }

        public: bool calculateDist(ssl_robocup_gazebo::FindDistance::Request &req, ssl_robocup_gazebo::FindDistance::Response &res)
        {
            // Get position state of the target
            gazebo_msgs::GetModelState target_state;
            target_state.request.model_name = req.target_model_name;
            this->rosPositionSrv.call(target_state);

            // Get position state of the origin
            gazebo_msgs::GetModelState self_state;  
            self_state.request.model_name = req.origin_model_name;
            this->rosPositionSrv.call(self_state);
            
            // Calculate the distance
            double x_difference = target_state.response.pose.position.x-self_state.response.pose.position.x; 
            double y_difference = target_state.response.pose.position.y-self_state.response.pose.position.y;
            double magnitudeDist = std::sqrt(std::pow(x_difference,2) + std::pow(y_difference,2));

            // Return distance value
            res.distance = magnitudeDist;
            return true;
        }
    };
    // Register this plugin
    GZ_REGISTER_WORLD_PLUGIN(FindDistance)
}