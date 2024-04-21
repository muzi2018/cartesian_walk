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

    /** task*/
    int current_state1 = 0;
    int current_state2 = 0;
    int current_state3 = 0;
    int current_state4 = 0;

    Eigen::VectorXd q, qdot, qddot;
    // auto right_arm_task = solver->getTask("arm2_8");
    // auto rarm_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(right_arm_task);


    /**leg task*/
    auto leg1_task = solver->getTask("wheel_1");
    auto leg1_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(leg1_task);

    /**leg task*/
    auto leg2_task = solver->getTask("wheel_2");
    auto leg2_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(leg2_task);

    /**leg task*/
    auto leg3_task = solver->getTask("wheel_3");
    auto leg3_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(leg3_task);

    /**leg task*/
    auto leg4_task = solver->getTask("wheel_4");
    auto leg4_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(leg4_task);

    // // // get pose reference from task
    // // // ...
    Eigen::Affine3d RightArm_T_ref;
    Eigen::Affine3d Leg1_T_ref;
    Eigen::Affine3d Leg2_T_ref;
    Eigen::Affine3d Leg3_T_ref;
    Eigen::Affine3d Leg4_T_ref;
    Eigen::Affine3d Torso_T_ref;
    
    int leg_state = 1,num_leg = 2, segment = 0;
    
    double phase_time = 3, target_time = num_leg*phase_time;
    double x, z;
    bool reset = true;

    // Trajectory::WayPointVector wp;
    // Eigen::Affine3d w_T_f1 ;
    // w_T_f1.setIdentity();

    // w_T_f1.pretranslate(Eigen::Vector3d(0.2,0,0));
    // wp.emplace_back(w_T_f1,10);
    // leg1_cartesian->setWayPoints(wp);
    

    int i=1;
    double seg_num = 100;
    double long_x = 0.2;
    double seg_time = phase_time / seg_num;
    double seg_dis = long_x / seg_num;
    double leg_height = 0.15;

    ros::Rate r(100);
    while (ros::ok())
    {
        if (leg_state == 1)
        {
            if (current_state1 == 0)
            {
                x = seg_dis;
                z = leg_height*sin(3.14*i/seg_num)-leg_height*sin(3.14*(i-1)/seg_num);
                leg1_cartesian->getPoseReference(Leg1_T_ref);
                Leg1_T_ref.pretranslate(Eigen::Vector3d(x,0,z));
                leg1_cartesian->setPoseTarget(Leg1_T_ref, seg_time);
                current_state1++;
                i++;
            }
            if(current_state1 == 1)
            {
                if (leg1_cartesian->getTaskState() == State::Reaching)
                {
                    {
                        std::cout << "Motion started!" << std::endl;
                        current_state1++;
                    }
                }

            }
            if(current_state1 == 2) // here we wait for it to be completed
            {
                if(leg1_cartesian->getTaskState() == State::Online)
                {
                    Eigen::Affine3d T;
                    leg1_cartesian->getCurrentPose(T);

                    std::cout << "Motion completed, final error is " <<
                                (T.inverse()*Leg1_T_ref).translation().norm() << std::endl;

                    
                    if (i != seg_num+1)
                    {
                        current_state1=0;

                    }else if (i == seg_num+1){
                        i = 0;
                        leg_state++;
                    }
                }
            }

        }
        



        if (leg_state == 2)
        {
            
            if (current_state2 == 0)
            {
                x = seg_dis;
                z = leg_height*sin(3.14*i/seg_num)-leg_height*sin(3.14*(i-1)/seg_num);
                leg2_cartesian->getPoseReference(Leg2_T_ref);
                Leg2_T_ref.pretranslate(Eigen::Vector3d(x,0,z));
                leg2_cartesian->setPoseTarget(Leg2_T_ref, seg_time);
                current_state2++;
                i++;
            }
            if(current_state2 == 1)
            {
                if (leg2_cartesian->getTaskState() == State::Reaching)
                {
                    {
                        std::cout << "Motion started!" << std::endl;
                        current_state2++;
                    }
                }

            }
            if(current_state2 == 2) // here we wait for it to be completed
            {
                if(leg2_cartesian->getTaskState() == State::Online)
                {
                    Eigen::Affine3d T;
                    leg1_cartesian->getCurrentPose(T);

                    std::cout << "Motion completed, final error is " <<
                                (T.inverse()*Leg2_T_ref).translation().norm() << std::endl;

                    
                    if (i != seg_num+1)
                    {
                        current_state2=0;

                    }else if (i == seg_num+1)
                    {
                        i=0;
                        leg_state++;
                    }
                }
            }
        }




        if (leg_state == 3)
        {
            
            if (current_state3 == 0)
            {
                x = seg_dis;
                z = leg_height*sin(3.14*i/seg_num)-leg_height*sin(3.14*(i-1)/seg_num);
                leg3_cartesian->getPoseReference(Leg3_T_ref);
                Leg3_T_ref.pretranslate(Eigen::Vector3d(x,0,z));
                leg3_cartesian->setPoseTarget(Leg3_T_ref, seg_time);
                current_state3++;
                i++;
            }
            if(current_state3 == 1)
            {
                if (leg3_cartesian->getTaskState() == State::Reaching)
                {
                    {
                        std::cout << "Motion started!" << std::endl;
                        current_state3++;
                    }
                }

            }
            if(current_state3 == 2) // here we wait for it to be completed
            {
                if(leg3_cartesian->getTaskState() == State::Online)
                {
                    Eigen::Affine3d T;
                    leg1_cartesian->getCurrentPose(T);

                    std::cout << "Motion completed, final error is " <<
                                (T.inverse()*Leg3_T_ref).translation().norm() << std::endl;

                    
                    if (i != seg_num+1)
                    {
                        current_state3=0;

                    }else if (i == seg_num+1)
                    {
                        i=0;
                        leg_state++;
                    }
                }
            }
        }




        if (leg_state == 4)
        {
            
            if (current_state4 == 0)
            {
                x = seg_dis;
                z = leg_height*sin(3.14*i/seg_num)-leg_height*sin(3.14*(i-1)/seg_num);
                leg4_cartesian->getPoseReference(Leg4_T_ref);
                Leg4_T_ref.pretranslate(Eigen::Vector3d(x,0,z));
                leg4_cartesian->setPoseTarget(Leg4_T_ref, seg_time);
                current_state4++;
                i++;
            }
            if(current_state4 == 1)
            {
                if (leg4_cartesian->getTaskState() == State::Reaching)
                {
                    {
                        std::cout << "Motion started!" << std::endl;
                        current_state4++;
                    }
                }

            }
            if(current_state4 == 2) // here we wait for it to be completed
            {
                if(leg4_cartesian->getTaskState() == State::Online)
                {
                    Eigen::Affine3d T;
                    leg1_cartesian->getCurrentPose(T);

                    std::cout << "Motion completed, final error is " <<
                                (T.inverse()*Leg4_T_ref).translation().norm() << std::endl;
                    
                    if (i != seg_num+1)
                    {
                        current_state4=0;

                    }else if (i == seg_num+1)
                    {
                        i=0;
                        leg_state=1;
                    }
                }
            }
        }

            // std::cout << "Motion started!" << std::endl;
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



