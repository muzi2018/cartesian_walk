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

using namespace XBot::Cartesian;


bool start_walking_bool = false;
bool tagDetected = false;
bool start_walking(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    start_walking_bool = !start_walking_bool;
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






int main(int argc, char **argv)
{
    const std::string robotName = "centauro";
    // Initialize ros node
    ros::init(argc, argv, robotName);
    ros::NodeHandle nodeHandle("");


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
    model->setJointPosition(qhome);
    model->update();
    XBot::Cartesian::Utils::RobotStatePublisher rspub (model);

    

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

    ros::Rate r(100);
    ros::ServiceServer service = nodeHandle.advertiseService("start_walking", start_walking);

    // // D435_head_camera_color_optical_frame
    // // tag_0
    // ros::Publisher pub_x_d = nodeHandle.advertise<std_msgs::Float64>("x_desire", 1000);
    // std_msgs::Float64 msg_xd;
    // ros::Publisher pub_x_c = nodeHandle.advertise<std_msgs::Float64>("x_actual", 1000);
    // std_msgs::Float64 msg_xc;

    // ros::Publisher pub_y_d = nodeHandle.advertise<std_msgs::Float64>("y_desire", 1000);
    // std_msgs::Float64 msg_yd;
    // ros::Publisher pub_y_c = nodeHandle.advertise<std_msgs::Float64>("y_actual", 1000);
    // std_msgs::Float64 msg_yc;
    

    geometry_msgs::TransformStamped tag_base_T; 
    double roll_e, pitch_e, yaw_e;
    Eigen::Vector6d E;
    Eigen::Vector6d E_Zero;
    E_Zero.setZero();
    double K_x = 0.1, K_y = 0.1, K_yaw = 0.1;

    bool reach_goal = false;
    while (ros::ok())
    {
        while (!start_walking_bool)
        {
            ros::spinOnce();
            r.sleep();
        }


        /**
         * Detection
        */
        std::string parent_frame = "base_link";
        std::string child_frame = "tag_0";

        if (tagDetected)
        {
            // std::cout << "tagDetected = " << tagDetected << std::endl;
            tag_base_T = tfBuffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
            /**
             * Error Calculate
            */
            double x_e = tag_base_T.transform.translation.x;
            double y_e = tag_base_T.transform.translation.y - 0.3;
            double z_e = tag_base_T.transform.translation.z;

            double x_ee = tag_base_T.transform.translation.x * tag_base_T.transform.translation.x;
            double y_ee = tag_base_T.transform.translation.y * tag_base_T.transform.translation.y;
            double z_ee = tag_base_T.transform.translation.z * tag_base_T.transform.translation.z;
            double e = sqrt(x_ee + y_ee);

            tf2::Quaternion q;
            q.setW(tag_base_T.transform.rotation.w);
            q.setX(tag_base_T.transform.rotation.x);
            q.setY(tag_base_T.transform.rotation.y);
            q.setZ(tag_base_T.transform.rotation.z);
            
            tf2::Matrix3x3 m(q);
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
            E[5] = K_yaw * yaw_e;
            // std::cout << "yaw_error = " << E[5] << std::endl;
            // E = K * E * e;


            std::cout << "tag_base_T.transform.translation.x = " << tag_base_T.transform.translation.x << std::endl;
            std::cout << "tag_base_T.transform.translation.y = " << tag_base_T.transform.translation.y << std::endl;
            std::cout << "tag_base_T.transform.translation.z = " << tag_base_T.transform.translation.z << std::endl;
            std::cout << "yaw = " << yaw_e << std::endl;

            if ((abs(x_e) > 1 || abs(y_e) > 0.05 || abs(yaw_e) > 0.1) && !reach_goal)
            {                
                
                car_cartesian->setVelocityReference(E);
                car_cartesian->setVelocityLimits(-0.05, 0.05);
            }

            if (abs(x_e) < 1 )
            {
                reach_goal = true;
                car_cartesian->setVelocityReference(E_Zero);
                car_cartesian->setVelocityLimits(-0.05, 0.05);
            }else{
                reach_goal = false;
            }

        }
        if (!tagDetected){
            car_cartesian->setVelocityReference(E_Zero);
            car_cartesian->setVelocityLimits(-0.05, 0.05);
        } 

        /**
         * Move Robot
        */

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
        ros::spinOnce();
        r.sleep();
        
    }
}




