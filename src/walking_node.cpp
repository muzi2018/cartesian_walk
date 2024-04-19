#include <thread>
#include <ros/init.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <XBotInterface/ModelInterface.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>

#include <Eigen/Dense>

using namespace XBot::Cartesian;

int main(int argc, char **argv)
{
    const std::string robotName = "centauro";
    // Initialize ros node
    ros::init(argc, argv, robotName);
    ros::NodeHandle nodeHandle;
    // Get node parameters
    std::string URDF_PATH, SRDF_PATH;
    nodeHandle.getParam("/urdf_path", URDF_PATH);
    nodeHandle.getParam("/srdf_path", SRDF_PATH);


    // an option structure which is needed to make a model
    XBot::ConfigOptions xbot_cfg;

    // set the urdf and srdf path..
    xbot_cfg.set_urdf_path(URDF_PATH);
    xbot_cfg.set_srdf_path(SRDF_PATH);

    // the following call is needed to generate some default joint IDs
    xbot_cfg.generate_jidmap();

    // some additional parameters..
    xbot_cfg.set_parameter("is_model_floating_base", true);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");

    // and we can make the model class
    auto model = XBot::ModelInterface::getModel(xbot_cfg);


    // initialize to a homing configuration
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    // qhome.setZero(); 


    model->setJointPosition(qhome);
    model->update();

    XBot::Cartesian::Utils::RobotStatePublisher rspub (model);


    // before constructing the problem description, let us build a
    // context object which stores some information, such as
    // the control period
    const double dt = 0.01;
    double time = 0;
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

    /**arm task*/
    int current_state = 0;
    Eigen::VectorXd q, qdot, qddot;
    auto right_arm_task = solver->getTask("arm2_8");
    auto rarm_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(right_arm_task);


    /**leg task*/
    auto leg1_task = solver->getTask("wheel_1");
    auto leg1_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(leg1_task);


    // // // get pose reference from task
    // // // ...
    Eigen::Affine3d RightArm_T_ref;
    Eigen::Affine3d Leg1_T_ref;
    Eigen::Affine3d Torso_T_ref;
    
    int leg_state = 1;
    double long_x = 0.3;
    double target_time = 3.0;
    double dx = long_x /target_time; // 0.1 / 3 = 0.03 m / s
    double x, z;

    // Trajectory::WayPointVector wp;
    // Eigen::Affine3d w_T_f1 ;
    // w_T_f1.setIdentity();

    // w_T_f1.pretranslate(Eigen::Vector3d(0,0,0.2));
    // wp.emplace_back(w_T_f1, 2);
    // w_T_f1.setIdentity();
    // wp.emplace_back(w_T_f1, 4);

    ros::Rate r(100);
    while (ros::ok())
    {

            // command leg 1, leg_state ++

            // command leg 2

            // command leg 3

            // command leg 4

        // if (current_state == 0)     
        // {
        //     leg1_cartesian->setWayPoints(wp);
        //     current_state++;

        // }
        


        // if(current_state == 0) // here we command a reaching motion
        // {
        //     std::cout << "Commanding left hand forward 0.3m in 3.0 secs" << std::endl;

        //     rarm_cartesian->getPoseReference(RightArm_T_ref);
        //     RightArm_T_ref.pretranslate(Eigen::Vector3d(0.2,0,-0.2));
        //     rarm_cartesian->setPoseTarget(RightArm_T_ref, target_time);

        //     leg1_cartesian->getPoseReference(Leg1_T_ref);
        //     Leg1_T_ref.pretranslate(Eigen::Vector3d(0,0,0.2));
        //     leg1_cartesian->setPoseTarget(Leg1_T_ref, target_time);
        //     // leg1_cartesian->setWayPoints(wp);
        //     current_state++;
        // }

        // if(current_state == 1) // here we check that the reaching started
        // {
        //     if(rarm_cartesian->getTaskState() == State::Reaching)
        //     {
        //         std::cout << "Motion started!" << std::endl;

        //         current_state++;
        //     }
        // }

        // if(current_state == 2) // here we wait for it to be completed
        // {
        //     if(rarm_cartesian->getTaskState() == State::Online)
        //     {
        //         Eigen::Affine3d T;
        //         rarm_cartesian->getCurrentPose(T);

        //         std::cout << "Motion completed, final error is " <<
        //                     (T.inverse()*RightArm_T_ref).translation().norm() << std::endl;

        //         current_state++;
        //     }
        // }

        // if(current_state == 3) // here we wait the robot to come to a stop
        // {
        //     std::cout << "qdot norm is " << qdot.norm() << std::endl;
        //     if(qdot.norm() < 1e-3)
        //     {
        //         std::cout << "Robot came to a stop, press ENTER to exit.. \n";
        //         std::cin.ignore();
        //         current_state++;
        //     }

        // }

    // while task state is Reaching

        if (time <= target_time)
        {
            if (current_state == 0)
            {
                leg1_cartesian->reset();
                x = dx * time;
                z = 3*sin(3.14*x/long_x);
                leg1_cartesian->getPoseReference(Leg1_T_ref);
                Leg1_T_ref.pretranslate(Eigen::Vector3d(x,0,z));
                leg1_cartesian->setPoseTarget(Leg1_T_ref, target_time);
                ROS_INFO_STREAM("time");
                ROS_INFO_STREAM(time);
                ROS_INFO_STREAM("x");
                ROS_INFO_STREAM(x);
                ROS_INFO_STREAM("z");
                ROS_INFO_STREAM(z);
                ROS_INFO_STREAM("current_state");
                ROS_INFO_STREAM(current_state);
                current_state++;
            }
            if(leg1_cartesian->getTaskState() == State::Reaching)
            {
                std::cout << "Motion started!" << std::endl;

                current_state--;
            }

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
        time += dt;

        rspub.publishTransforms(ros::Time::now(), "");
        r.sleep();
    }


}



    // // Print position
    // std::stringstream ss;
    // ss << "qhome: [";
    // for (int i = 0; i < qhome.size(); ++i)
    // {
        
    //     if (i < qhome.size() - 1){
    //         ss << "q" << i  << ": ";
    //         ss << qhome[i];
    //         ss << " ";
    //     }

    // }
    // ss << "]";
    // ROS_INFO_STREAM(ss.str());



