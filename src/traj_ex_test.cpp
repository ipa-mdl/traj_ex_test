#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include "counter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_ex_test");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit_msgs::RobotTrajectory traj;

    ros::NodeHandle priv("~");

    if (!priv.getParam("joints", traj.joint_trajectory.joint_names)){
        ROS_ERROR("please set joints param");
        return 1;
    }

    traj.joint_trajectory.points.resize(1);
    traj.joint_trajectory.points[0].positions.resize(traj.joint_trajectory.joint_names.size());
    traj.joint_trajectory.points[0].time_from_start = ros::Duration(1.0);

    RateCounter rate_counter_;

    robot_model_loader::RobotModelLoader rml;
    trajectory_execution_manager::TrajectoryExecutionManager tem(rml.getModel(), true);

    double rate = priv.param("rate", 0.0);
    ros::Rate r(ros::Duration( rate ? 1.0/rate : 0.0));

    while(ros::ok()){
        rate_counter_.count(traj.joint_trajectory.header.stamp = ros::Time::now());
        tem.push(traj);
        tem.executeAndWait();
        r.sleep();
    }

    std::cout << rate_counter_ << std::endl;

    return 0;
}