#include <thread>
#include <ros/init.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/ModelInterface.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/TransformStamped.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <std_srvs/Empty.h>
#include <xbot_msgs/JointCommand.h>
using namespace XBot::Cartesian;
bool start_searching_bool = false;
bool tagDetected = false;
bool start_searching(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    start_searching_bool = !start_searching_bool;
    return true;
};

void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    // std::cout << "msg->detections.size() = " << msg->detections.size() << std::endl;\

    if (msg->detections.size() > 0) {
        tagDetected = true;
    }else{
        tagDetected = false;
    }

    // if (tagDetected) {
    //     ROS_INFO("A tag has been detected.");
    // } else {
    //     ROS_INFO("No tags detected. tagDetected = %d", tagDetected);
    // }

}

void SearchingTag(ros::NodeHandle nodeHandle){
        std::vector<std::string> joint_names{
                                            "torso_yaw",
                                            };
        std::vector<float> joint_positions (1);
        joint_positions[0] = 0.5;
        xbot_msgs::JointCommand joint_command;
        ros::Publisher jointCmdPublisher_ = nodeHandle.advertise<xbot_msgs::JointCommand>("/xbotcore/command", 10);
        joint_command.header.stamp = ros::Time::now();
        joint_command.name = joint_names; 
        joint_command.position = joint_positions;
        joint_command.ctrl_mode = std::vector<uint8_t>(joint_positions.size(), 1);
        jointCmdPublisher_.publish(joint_command);
}

void TurnToTag(Eigen::Vector6d E, int &searching_num, double K_yaw, XBot::Cartesian::CartesianTask* car_cartesian){
        E[0] = 0;
        E[1] = 0;
        E[2] = 0;
        E[3] = 0;
        E[4] = 0;
        double yaw_e = 3.14 * searching_num/200;
        E[5] = K_yaw * yaw_e;
        car_cartesian->setVelocityReference(E); 
                std::cout << "searching_num = " << searching_num << std::endl;

        searching_num ++ ;
        if (searching_num == 100)
        {
            searching_num = 0;
        }
}



// Turn_yaw(E, int &tun_num, double K_yaw, XBot::Cartesian::CartesianTask* car_cartesian){

// }


int main(int argc, char **argv)
{
    const std::string robotName = "centauro";
    // Initialize ros node
    ros::init(argc, argv, robotName);
    ros::NodeHandle nodeHandle("");
    // opendrawer
    // opendoor
    ros::ServiceClient client = nodeHandle.serviceClient<std_srvs::Empty>("opendoor");
    std_srvs::Empty srv;



    // Create a Buffer and a TransformListener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    auto cfg = XBot::ConfigOptionsFromParamServer();


    // and we can make the model class
    auto model = XBot::ModelInterface::getModel(cfg);
    auto robot = XBot::RobotInterface::getRobot(cfg);
    // initialize to a homing configuration
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    std::cout << "Homing configuration: " << std::endl << qhome.size() << std::endl;
    std::cout << "qhome = " << qhome << std::endl;

    model->setJointPosition(qhome);
    model->update();
    XBot::Cartesian::Utils::RobotStatePublisher rspub (model);

    std::cout << "pppp" << std::endl;

    robot->setControlMode(
        {
            {"j_wheel_1", XBot::ControlMode::Velocity()},
            {"j_wheel_2", XBot::ControlMode::Velocity()},
            {"j_wheel_3", XBot::ControlMode::Velocity()},
            {"j_wheel_4", XBot::ControlMode::Velocity()}
        }
    );

    int homing_num = 100;
    int searching_num = 0; bool search_flag = false, turn_num = 20;
    bool home_flag = false;

    // before constructing the problem description, let us build a
    // context object which stores some information, such as
    // the control period
    const double dt = 0.01;
    double time = 0, plan_time = 0;
    auto ctx = std::make_shared<XBot::Cartesian::Context>(
                std::make_shared<XBot::Cartesian::Parameters>(dt),
                model
            );

    // load the ik problem given a yaml file
    std::string problem_description_string;
    nodeHandle.getParam("problem_description", problem_description_string);

    auto ik_pb_yaml = YAML::Load(problem_description_string);
    XBot::Cartesian::ProblemDescription ik_pb(ik_pb_yaml, ctx);

    // we are finally ready to make the CartesIO solver "OpenSot"
    auto solver = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                       ik_pb, ctx
                                                       );

    
    ros::Subscriber sub = nodeHandle.subscribe("/tag_detections", 1000, tagDetectionsCallback);
    


    Eigen::VectorXd q, qdot, qddot;
    auto car_task = solver->getTask("base_link");
    auto car_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(car_task);    

  


    double det_shift_x = 0.5, det_shift_y = 0.5;

    ros::Rate r(10);
    ros::ServiceServer service = nodeHandle.advertiseService("start_searching", start_searching);

    geometry_msgs::TransformStamped tag_base_T; 
    geometry_msgs::TransformStamped wheelFL_base_T; 
    double roll_e, pitch_e, yaw_e;
    Eigen::Vector6d E;
    Eigen::Vector6d E_Zero;
    E_Zero.setZero();
    double K_x = 0.1, K_y = 0.1, K_roll = 0.1, K_pitch = 0.1 , K_yaw = 0.1;
    bool reach_goal = false;

    int nodes_num = 100;

    int off_num = 100; // off num to start tracking
    double yaw_l = -0.5;
    double yaw_h = 0.5;
    bool reach_yaw = false;

    while (ros::ok())
    {
        while (!start_searching_bool)
        {
            // std::cout << "start_searching_bool: " << start_searching_bool << std::endl;
            ros::spinOnce();
            r.sleep();
        }


        /**
         * Detection
        */
        std::string parent_frame = "base_link";
        std::string child_frame = "tag_0";
        if (tagDetected && off_num ){
            E[0] = 0;
            E[1] = 0;
            E[2] = 0;
            E[3] = 0;
            E[4] = 0;
            double yaw_e = 3.14 * (100 - off_num)/200;
            E[5] = K_yaw * yaw_e;
            car_cartesian->setVelocityReference(E); 
            off_num -- ;
            std::cout << "tagDetected but should turn it to tag off num = " << off_num << std::endl;
        }

        if (tagDetected && !off_num )
        {

            std::cout << "tagDetected = " << tagDetected << std::endl;
            search_flag = true;
            tag_base_T = tfBuffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
            /**
             * Error Calculate
            */
            double x_e = tag_base_T.transform.translation.x;
            double y_e = tag_base_T.transform.translation.y ;
            // std::cout << "y_e = " << y_e <<  std::endl;
            double z_e = tag_base_T.transform.translation.z;

            double x_ee = tag_base_T.transform.translation.x * tag_base_T.transform.translation.x;
            double y_ee = tag_base_T.transform.translation.y * tag_base_T.transform.translation.y;
            double z_ee = tag_base_T.transform.translation.z * tag_base_T.transform.translation.z;
            double e = sqrt(x_ee + y_ee);

            tf2::Quaternion q_;
            q_.setW(tag_base_T.transform.rotation.w);
            q_.setX(tag_base_T.transform.rotation.x);
            q_.setY(tag_base_T.transform.rotation.y);
            q_.setZ(tag_base_T.transform.rotation.z);
            
            tf2::Matrix3x3 m(q_);
            m.getRPY(roll_e, pitch_e, yaw_e);
            yaw_e = yaw_e + 1.6;
            /**
             * Velocity Controller
            */
            
            
            E[0] = K_x * x_e;
            E[1] = K_y * y_e;
            E[2] = 0;
            E[3] = 0;
            E[4] = 0;
            E[5] = K_yaw * 0;
            // std::cout << "yaw_error = " << E[5] << std::endl;
            // E = K * E * e;


            // std::cout << "tag_base_T.transform.translation.x = " << tag_base_T.transform.translation.x << std::endl;
            // std::cout << "tag_base_T.transform.translation.y = " << tag_base_T.transform.translation.y << std::endl;
            // std::cout << "tag_base_T.transform.translation.z = " << tag_base_T.transform.translation.z << std::endl;
            // std::cout << "yaw = " << yaw_e << std::endl;

            if ((abs(x_e) > 1 || abs(y_e) > 0.4 ) && !reach_goal)
            {                
                std::cout << "x_e = " << x_e << std::endl;
                std::cout << "y_e = " << y_e << std::endl;
                // std::cout << "yaw_e = " << yaw_e << std::endl;

                if (search_flag)
                {

                }
                car_cartesian->setVelocityReference(E);
                
            }
            }else{
                reach_goal = false;
            }

            
            if (!tagDetected && !search_flag){
                std::cout << "no tag" << std::endl;
                TurnToTag(E, searching_num, K_yaw, car_cartesian.get());
                // std::cout << "no tag detected! " << std::endl;
                // E[0] = 0;
                // E[1] = 0;
                // E[2] = 0;
                // E[3] = 0;
                // E[4] = 0;
                // yaw_e = -3.14 * searching_num/100;
                // E[5] = K_yaw * yaw_e;
                // car_cartesian->setVelocityReference(E); 
                // searching_num ++ ;
                // if (searching_num == 100)
                // {
                //     searching_num = 0;
                // }
            } 
            solver->update(time, dt);
            model->getJointPosition(q);
            model->getJointVelocity(qdot);
            model->getJointAcceleration(qddot);
            q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
            qdot += dt * qddot;
            model->setJointPosition(q);
            model->setJointVelocity(qdot);
            model->update();
            robot->setPositionReference(q.tail(robot->getJointNum()));
            robot->setVelocityReference(qdot.tail(robot->getJointNum()));
            robot->move();
            time += dt;
            rspub.publishTransforms(ros::Time::now(), "");
            /**
             * Move Robot
            */
            ros::spinOnce();
            r.sleep();
        }        
}





