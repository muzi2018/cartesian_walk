#include <thread>
#include <ros/init.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <XBotInterface/ModelInterface.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>

#include <Eigen/Dense>


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
    model->setJointPosition(qhome);
    model->update();

    // XBot::Cartesian::Utils::RobotStatePublisher rspub (model);


    // // before constructing the problem description, let us build a
    // // context object which stores some information, such as
    // // the control period
    // const double dt = 0.01;
    // auto ctx = std::make_shared<XBot::Cartesian::Context>(
    //             std::make_shared<XBot::Cartesian::Parameters>(dt),
    //             model
    //         );

    // // load the ik problem given a yaml file
    //  std::string problem_description_string;
    // nodeHandle.getParam("problem_description", problem_description_string);

    // auto ik_pb_yaml = YAML::Load(problem_description_string);
    // XBot::Cartesian::ProblemDescription ik_pb(ik_pb_yaml, ctx);

    // // we are finally ready to make the CartesIO solver "OpenSot"
    // auto solver = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
    //                                                    ik_pb, ctx
    //                                                    );

    // auto right_arm_task = solver->getTask("arm1_8");
    // auto task_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(right_arm_task);

    // // get pose reference from task
    // // ...

    // // set new pose reference to task
    // // ...


    ros::Rate r(100);
    while (ros::ok())
    {

        // while task state is Reaching
        // while (...)
        // {
            // solver->update(time, dt);

    // model->getJointPosition(q);
    // model->getJointVelocity(qdot);
    // model->getJointAcceleration(qddot);

    // q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
    // qdot += dt * qddot;

    // model->setJointPosition(q);
    // model->setJointVelocity(qdot);
    // model->update();
        // }

        // rspub.publishTransforms(ros::Time::now(), "");
        r.sleep();
    }


}
