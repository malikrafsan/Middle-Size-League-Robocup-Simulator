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
#include "ssl_robocup_gazebo/SetOrient.h"
#include "ssl_robocup_gazebo/SetOrientRequest.h"
#include "ssl_robocup_gazebo/SetOrientResponse.h"
#include "ssl_robocup_gazebo/MoveBall.h"
#include "ssl_robocup_gazebo/MoveBallRequest.h"
#include "ssl_robocup_gazebo/MoveBallResponse.h"
#include "ssl_robocup_gazebo/FindDistance.h"
#include "ssl_robocup_gazebo/FindDistanceRequest.h"
#include "ssl_robocup_gazebo/FindDistanceResponse.h"
#include "ssl_robocup_gazebo/Attach.h"
#include "ssl_robocup_gazebo/AttachRequest.h"
#include "ssl_robocup_gazebo/AttachResponse.h"
#include <string>

namespace gazebo
{
    class BehaviorTree: public ModelPlugin
    {
        private: std::unique_ptr<ros::NodeHandle> rosNode;
        private: ros::ServiceClient rosChaserSrv;
        private: ros::ServiceClient rosDistSrv;
        private: ros::ServiceClient rosAttachSrv;
        private: physics::ModelPtr model;
        private: event::ConnectionPtr updateConnection;
        private: std::string modelName;
        private: std::string ballHolder;
        private: std::string enemyGoal;
        // private: int specificLoc[2];

        private: void whichGoal(std::string modelName)
        {
            if (modelName[0] == 'A') { this->enemyGoal = "robocup_ssl_right_goal"; } 
            else { this->enemyGoal = "robocup_ssl_left_goal"; }
        }

        // private: void setSpecLoc()
        // {
        //     if (modelName[0] == 'A') 
        //     { 
        //         this->specificLoc[0] = 0; 
        //         this->specificLoc[1] = 0;
        //     }
        //     else
        //     {
        //         this->specificLoc[0] = 0; 
        //         this->specificLoc[1] = 0;
        //     }
        // }

        private: void chase(std::string chased)
        {
            // Call service to move this robot to chased object
            ssl_robocup_gazebo::SetOrient chasing;
            chasing.request.origin_model_name = this->modelName;
            chasing.request.origin_link_name = "rack";
            chasing.request.target_model_name = chased;
            this->rosChaserSrv.call(chasing);

            ROS_INFO("%s is chased", chased.c_str());
        }

        private: std::string whoHoldBall()
        {
            std::string inputHolder = "None"; // REMOVE NONE LATER
            // Call service to check who hold the ball
            return inputHolder;
        }

        private: int isMeAllyEnemy(std::string inputHolder)
        {
            std::string modelName = this->model->GetName().c_str();
            if (modelName[0] == inputHolder[0])
            {
                if (modelName == inputHolder) { return 1; }
                else { return 2; }
            }
            else
            {
                if (inputHolder == "None") { return 0; } 
                else { return 3; }
            }
        }

        private: bool modelInGoalRadius(std::string modelName)
        {
            // Call service to check whether the model is in goal radius
            return false;
        }

        private: void kickBall(std::string targetName)
        {
            // Call service to kick the ball to target object
        }

        private: std::string whoCloseToGoal()
        {   
            std::string arrRobotName[3];
            if (modelName[0] == 'A') 
            { 
                arrRobotName[0] = "A_robot1"; 
                arrRobotName[1] = "A_robot2";
                arrRobotName[2] = "A_robot3"; 
            }
            else 
            {
                arrRobotName[0] = "B_robot1"; 
                arrRobotName[1] = "B_robot2";
                arrRobotName[2] = "B_robot3"; 
            }
            
            double distGoalRobot[3];
            for (int i=0;i<3;i++)
            {
                ssl_robocup_gazebo::FindDistance dist;
                dist.request.origin_model_name = this->enemyGoal;
                dist.request.target_model_name = arrRobotName[i];
                this->rosDistSrv.call(dist);
                distGoalRobot[i] = dist.response.distance;
            }
            int minIndex = 2;
            for (int j=0;j<2;j++)
            {
                if (distGoalRobot[j] < distGoalRobot[minIndex]) { minIndex = j; }
            }
            return arrRobotName[minIndex];
        }

        public: void OnUpdate()
        {
            this->ballHolder = whoHoldBall();
            int switching = isMeAllyEnemy(this->ballHolder);
            switch (switching)
            {
                // There is no ball holder
                case 0:
                    chase("ball");
                    break;

                // This robot hold the ball
                case 1:
                {
                    bool inGoalRadia = modelInGoalRadius(this->modelName);
                    if (inGoalRadia) { kickBall(this->enemyGoal); }
                    else 
                    {
                        std::string modelCloserToGoal = whoCloseToGoal();
                        if (modelCloserToGoal == this->modelName) { chase(this->enemyGoal); }
                        else { kickBall(modelCloserToGoal); }
                    }
                    break;
                }

                // Ally of this robot hold the ball
                case 2:
                    chase(this->enemyGoal); 
                    // TEMPORARY, TRY MOVING TO SPECIFIC LOCATION
                    // BUT WE MUST CREATE NEW MECHANISM TO DO IT
                    break;

                // Enemy of this robot hold the ball
                default:
                    chase("ball");
            }
        }

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr)
        {
            this->model = _parent;

            if (!ros::isInitialized)
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "behavior_tree", 
                ros::init_options::NoSigintHandler);
            }
            this->rosNode.reset(new ros::NodeHandle("behavior_tree"));
            this->rosChaserSrv = this->rosNode->serviceClient<ssl_robocup_gazebo::SetOrient>("/kinetics/set_orient_n_chase");
            this->rosDistSrv = this->rosNode->serviceClient<ssl_robocup_gazebo::FindDistance>("/kinetics/calculateDist");

            this->modelName = this->model->GetName().c_str();
            whichGoal(this->modelName);
            // setSpecLoc();

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&BehaviorTree::OnUpdate, this));
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(BehaviorTree)
}
