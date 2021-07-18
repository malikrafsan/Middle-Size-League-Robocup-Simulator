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
#include "ssl_robocup_gazebo/MoveToCoord.h"
#include "ssl_robocup_gazebo/MoveToCoordRequest.h"
#include "ssl_robocup_gazebo/MoveToCoordResponse.h"
#include "ssl_robocup_gazebo/GameMessage.h"
#include "ssl_robocup_gazebo/BallHolder.h"
#include "ssl_robocup_gazebo/BallHolderRequest.h"
#include "ssl_robocup_gazebo/BallHolderResponse.h"
#include "ros/subscribe_options.h"
#include <string>
#include <unistd.h>


namespace gazebo
{
    class BehaviorTree: public ModelPlugin
    {
        private: std::unique_ptr<ros::NodeHandle> rosNode;
        private: ros::ServiceClient rosChaserSrv;
        private: ros::ServiceClient rosDistSrv;
        private: ros::ServiceClient rosAttachSrv;
        private: ros::ServiceClient rosDetachSrv;
        private: ros::ServiceClient rosKickSrv;
        private: ros::ServiceClient rosMoveSrv;        
        private: ros::ServiceClient rosBallHandlerSrv;        
        private: physics::ModelPtr model;
        private: event::ConnectionPtr updateConnection;
        private: std::string modelName;
        private: std::string ballHolder;
        private: std::string enemyGoal;
        private: ros::Subscriber rosGamePluginSub;
        private: ros::CallbackQueue rosQueue;
        private: std::thread rosQueueThread;
        private: bool freezeState;
  
        private: double specificLoc[2];

        private: void whichGoal(std::string modelName)
        {
            if (modelName[0] == 'A') { this->enemyGoal = "robocup_ssl_right_goal"; } 
            else { this->enemyGoal = "robocup_ssl_left_goal"; }
        }

        private: void setSpecLoc()
        {
            if (this->modelName[7] == '1') 
            { 
                this->specificLoc[0] = -3.5;
                this->specificLoc[1] = 1;
            }
            else if (this->modelName[7] == '2')
            {
                this->specificLoc[0] = -3;
                this->specificLoc[1] = 0;
            }
            else
            {
                this->specificLoc[0] = -3.5;
                this->specificLoc[1] = -1;
            }

            if (this->modelName[0] == 'B') { this->specificLoc[0] = - this->specificLoc[0]; }
            // ROS_INFO("SPECIFIC LOCATION: %lf", this->specificLoc[0]);
        }

        private: void chase(std::string chased)
        {
            // Call service to move this robot to chased object
            ssl_robocup_gazebo::SetOrient chasing;
            chasing.request.origin_model_name = this->modelName;
            chasing.request.origin_link_name = "rack";
            chasing.request.target_model_name = chased;
            this->rosChaserSrv.call(chasing);
            if(this->ballHolder == this->modelName) return;
            else if(this->ballHolder != "None"){
                //Case Bola dah dipegang sama musuh -> Rebut 
                this->handlerDefault();
            }
            else if(this->ballHolder == "None"){
                this->handlerNone();
            }
            // ROS_INFO("%s is chased", chased.c_str());
        }

        private: void handlerDefault(){
            double current_distance = calculateDist(this->modelName, this->ballHolder);

            if(current_distance < 0.25){
                ssl_robocup_gazebo::Attach detach;
                detach.request.model_name_1 = this->ballHolder;
                detach.request.link_name_1 = "rack";
                detach.request.model_name_2 = "ball";
                detach.request.link_name_2 = "ball";
                this->rosDetachSrv.call(detach);

                usleep(3000000);

                ssl_robocup_gazebo::Attach attach;
                attach.request.model_name_1 = this->modelName;
                attach.request.link_name_1 = "rack";
                attach.request.model_name_2 = "ball";
                attach.request.link_name_2 = "ball";
                this->rosAttachSrv.call(attach);

                ssl_robocup_gazebo::BallHolder ball_handler;
                ball_handler.request.model_name = this->modelName;
                this->rosBallHandlerSrv.call(ball_handler);
            }
        }

        private: void handlerNone(){
            double current_distance = calculateDist(this->modelName, "ball");

            if(current_distance <= 0.2){
                ssl_robocup_gazebo::Attach attach;
                attach.request.model_name_1 = this->modelName;
                attach.request.link_name_1 = "rack";
                attach.request.model_name_2 = "ball";
                attach.request.link_name_2 = "ball";
                this->rosAttachSrv.call(attach);
                
                ssl_robocup_gazebo::BallHolder ball_handler;
                ball_handler.request.model_name = this->modelName;
                this->rosBallHandlerSrv.call(ball_handler);
            }
        }

        // private: std::string whoHoldBall()
        // {
        //     std::string inputHolder = "None"; // REMOVE NONE LATER
        //     // Call service to check who hold the ball
        //     return inputHolder;
        // }

        private: int isMeAllyEnemy(std::string inputHolder)
        {
            std::string modelName = this->model->GetName().c_str();
            if (modelName[0] == inputHolder[0])
            {
                if (modelName == inputHolder) { return 1; } //self
                else { return 2; } // team
            }
            else
            {
                if (inputHolder == "None") { return 0; } // none
                else { return 3; }
            }
        }

        private: bool modelInGoalRadius(std::string modelName, const ssl_robocup_gazebo::GameMessageConstPtr &_msg)
        {
            // Subscribe to messages to check whether the model is in goal radius
            bool arrayTrueFalse[6];

            arrayTrueFalse[0] = _msg->model_at_goal_radius.A_robot1;
            arrayTrueFalse[1] = _msg->model_at_goal_radius.A_robot2;
            arrayTrueFalse[2] = _msg->model_at_goal_radius.A_robot3;
            arrayTrueFalse[3] = _msg->model_at_goal_radius.B_robot1;
            arrayTrueFalse[4] = _msg->model_at_goal_radius.B_robot2;
            arrayTrueFalse[5] = _msg->model_at_goal_radius.B_robot3;

            int index = this->modelName[7] - 49;
            if (this->modelName[0] == 'B') { index += 3; }

            return arrayTrueFalse[index];
        }

        private: void kickBall(std::string targetName)
        {
            // Call service to detach the ball from this
            ssl_robocup_gazebo::Attach detach;
            detach.request.model_name_1 = this->ballHolder;
            detach.request.link_name_1 = "rack";
            detach.request.model_name_2 = "ball";
            detach.request.link_name_2 = "ball";
            this->rosDetachSrv.call(detach);

            ssl_robocup_gazebo::BallHolder ball_handler;
            ball_handler.request.model_name = "None";
            this->rosBallHandlerSrv.call(ball_handler);

            // Call service to kick the ball to target object
            ssl_robocup_gazebo::MoveBall kicking;
            kicking.request.origin_model_name = this->modelName;
            kicking.request.target_model_name = targetName;
            this->rosKickSrv.call(kicking);

            // ROS_INFO("ball is kicked to %s", targetName.c_str());
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
                distGoalRobot[i] = calculateDist(this->enemyGoal,arrRobotName[i]);
            }
            int minIndex = 2;
            for (int j=0;j<2;j++)
            {
                if (distGoalRobot[j] < distGoalRobot[minIndex]) { minIndex = j; }
            }
            return arrRobotName[minIndex];
        }

        private: double calculateDist(std::string origin, std::string target)
        {
            ssl_robocup_gazebo::FindDistance dist;
            dist.request.origin_model_name = origin;
            dist.request.target_model_name = target;
            this->rosDistSrv.call(dist);
            return dist.response.distance;
        }

        private: void moveSpecLoc()
        {
            // Call service to move this robot to spesific location

            ssl_robocup_gazebo::MoveToCoord move;
            move.request.origin_model_name = this->modelName;
            move.request.origin_link_name = "rack";
            move.request.target_x_coordinate = this->specificLoc[0];
            move.request.target_y_coordinate = this->specificLoc[1];
            this->rosMoveSrv.call(move);

        }

        private: bool checkModelFreeze(const ssl_robocup_gazebo::GameMessageConstPtr &_msg){
            if(this->modelName == "A_robot1") return (bool)_msg->freeze.A_robot1;
            if(this->modelName == "A_robot2") return (bool)_msg->freeze.A_robot2;
            if(this->modelName == "A_robot3") return (bool)_msg->freeze.A_robot3;
            if(this->modelName == "B_robot1") return (bool)_msg->freeze.B_robot1;
            if(this->modelName == "B_robot2") return (bool)_msg->freeze.B_robot2;
            if(this->modelName == "B_robot3") return (bool)_msg->freeze.B_robot3;
        }
        

        public: void OnRosMsg(const ssl_robocup_gazebo::GameMessageConstPtr &_msg)
        {
            this->freezeState = checkModelFreeze(_msg);
            if(this->freezeState) return;
            this->ballHolder = _msg->ball_holder;
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
                    int inGoalRadia = modelInGoalRadius(this->modelName, _msg);
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
                    moveSpecLoc();
                    break;

                // Enemy of this robot hold the ball
                default:
                    chase("ball");
            }
        }

        private: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
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
            this->rosKickSrv = this->rosNode->serviceClient<ssl_robocup_gazebo::MoveBall>("/kinetics/move_ball");
            this->rosMoveSrv = this->rosNode->serviceClient<ssl_robocup_gazebo::MoveToCoord>("/kinetics/move_to_coord");
            this->rosAttachSrv = this->rosNode->serviceClient<ssl_robocup_gazebo::Attach>("/link_attacher_node/attach");
            this->rosDetachSrv = this->rosNode->serviceClient<ssl_robocup_gazebo::Attach>("/link_attacher_node/detach");
            this->rosBallHandlerSrv = this->rosNode->serviceClient<ssl_robocup_gazebo::BallHolder>("/game_plugin/ball_holder");

            ros::SubscribeOptions so =
                ros::SubscribeOptions::create<ssl_robocup_gazebo::GameMessage>(
                    "/game_plugin/game_info",
                    1,
                    boost::bind(&BehaviorTree::OnRosMsg, this, _1),
                    ros::VoidPtr(), &this->rosQueue);
            this->rosGamePluginSub = this->rosNode->subscribe(so);
            this->rosQueueThread = std::thread(std::bind(&BehaviorTree::QueueThread,this));

            this->modelName = this->model->GetName().c_str();
            whichGoal(this->modelName);
            setSpecLoc();

            // this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            //     std::bind(&BehaviorTree::OnUpdate, this));
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(BehaviorTree)
}
