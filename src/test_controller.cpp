#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <boost/scoped_ptr.hpp>

#include <pluginlib/class_list_macros.h>

#include "counter.h"

namespace traj_ex_test {

class TestJointTrajectoryController: public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
  {
    std::vector<std::string> joint_names;
    // List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    if(joint_names.size() == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }
    for(unsigned int i=0; i<joint_names.size(); i++)
    {
      try
      {
        joints_.push_back(hw->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }
    }
    as_.reset(new ActionServer(n, "follow_joint_trajectory", boost::bind(&TestJointTrajectoryController::execute, this, _1), false));
    as_->start();
    return true;
  }

  void starting(const ros::Time& time) { latency_counter_.reset(); }
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {}
  void stopping(const ros::Time& time) { }


private:
  std::vector< hardware_interface::JointHandle > joints_;
  typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> ActionServer;
  boost::scoped_ptr<ActionServer> as_;

  LatencyCounter latency_counter_;

  void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal){
      latency_counter_.count(ros::Time::now(), goal->trajectory.header.stamp);
      as_->setSucceeded();
  }
};

} // namespace traj_ex_test

PLUGINLIB_EXPORT_CLASS(traj_ex_test::TestJointTrajectoryController, controller_interface::ControllerBase);
