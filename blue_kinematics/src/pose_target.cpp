#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include "blue_controllers/pseudoinverse.h"

KDL::Tree kdl_tree;
KDL::Chain kdl_chain;

ros::Publisher joint_position_pub;
ros::Subscriber joint_state_sub;
ros::Subscriber command_sub;

Eigen::Matrix<double, 3, Eigen::Dynamic> target_position;
KDL::Rotation target_rotation;

bool started = false;

std::vector<std::string> joint_names;
std::vector<double> posture_target;
double posture_gain;

void jointStateCallback(const sensor_msgs::JointState msg)
{
  if (!started)
    return;

  int nj = kdl_tree.getNrOfJoints();

  // Load joint positions into KDL
  KDL::JntArray joint_positions = KDL::JntArray(nj);
  for (int i = 0; i < nj; i++) {
    for (int j = 0; j < nj; j++) {
      if (msg.name[j].compare(joint_names[i]) == 0) {
        joint_positions(i) = msg.position[j];
        break;
      }
      if (j == nj - 1) {
         ROS_ERROR("Could not find %s in joint_states message", msg.name[i].c_str());
      }
    }
  }

  // Build KDL solvers
  KDL::ChainJntToJacSolver jac_solver(kdl_chain);
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

  // Iteratively solve for a (regularized) IK solution
  KDL::JntArray joint_positions_ik = KDL::JntArray(joint_positions);
  for (int i = 0; i < 50; i++) {

    // Compute jacobian via KDL
    KDL::Jacobian jacobian(nj);
    if (!jac_solver.JntToJac(joint_positions_ik, jacobian, -1))
      ROS_ERROR("Jacobian solver failed");

    // Load it into Eigen
    Eigen::Matrix<double,6,Eigen::Dynamic> jacobian_eigen(6,nj);
    for (int joint = 0; joint < nj; joint++) {
      for (int dim = 0; dim < 6; dim ++) {
        jacobian_eigen(dim, joint) = jacobian(dim, joint);
      }
    }

    // Compute cartesian position via KDL
    KDL::Frame cartesian_pose;
    if (!fk_solver.JntToCart(joint_positions_ik, cartesian_pose))
      ROS_ERROR("Forward kinematics failed");

    Eigen::Matrix<double,3, Eigen::Dynamic> current_position(3,1);
    for (int dim = 0; dim < 3; dim ++) {
      current_position(dim, 0) = cartesian_pose.p.data[dim];
    }

    KDL::Rotation rotation_difference = target_rotation * cartesian_pose.M.Inverse();
    KDL::Vector rotation_difference_vec = rotation_difference.GetRot();
    if(i == 0){
      ROS_DEBUG_THROTTLE(1, "%f rot_difference", rotation_difference_vec[0]);
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> position_difference = target_position - current_position;
    Eigen::Matrix<double, 6, Eigen::Dynamic> deltaX(6,1);
    deltaX(0, 0) = position_difference(0, 0);
    deltaX(1, 0) = position_difference(1, 0);
    deltaX(2, 0) = position_difference(2, 0);
    deltaX(3, 0) = rotation_difference_vec(0);
    deltaX(4, 0) = rotation_difference_vec(1);
    deltaX(5, 0) = rotation_difference_vec(2);

    Eigen::MatrixXd delta_joint = jacobian_eigen.transpose() * deltaX;

    double alpha;
    if (i > 20){
      alpha = 0.05;
    } else {
      alpha = 0.2;
    }
    for (int j = 0; j < nj; j++) {
      joint_positions_ik(j) = alpha * delta_joint(j, 0) + joint_positions_ik(j);
    }
  }

  std_msgs::Float64MultiArray joint_command_msg;

  for (int i = 0; i < nj; i++) {
    joint_command_msg.data.push_back(joint_positions_ik(i));
    ROS_ERROR_THROTTLE(1, "published command %d, %f", i, joint_command_msg.data[i]);
  }
  // ROS_ERROR("published command %d, %f", i, commandMsg.data[i]);
  joint_position_pub.publish(joint_command_msg);
}

void commandCallback(const geometry_msgs::PoseStamped msg)
{
  target_position(0, 0) = msg.pose.position.x;
  target_position(1, 0) = msg.pose.position.y;
  target_position(2, 0) = msg.pose.position.z;
  target_rotation = KDL::Rotation::Quaternion(
      msg.pose.orientation.x,
      msg.pose.orientation.y,
      msg.pose.orientation.z,
      msg.pose.orientation.w
  );

  started = true;
}

template <typename TParam>
void getRequiredParam(ros::NodeHandle &nh, const std::string name, TParam &dest) {
  if(!nh.getParam(name, dest)) {
    ROS_FATAL("Could not find %s parameter in namespace %s", name.c_str(), nh.getNamespace().c_str());
    ros::shutdown();
    exit(1);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "inverse_kin_target");
  ros::NodeHandle node;

  // Parameters
  std::string robot_desc_string;
  std::string endlink;
  getRequiredParam(node, "/robot_description", robot_desc_string);
  getRequiredParam(node, "blue_hardware/endlink",  endlink);
  getRequiredParam(node, "blue_hardware/joint_names", joint_names);

  // KDL setup
  if(!kdl_parser::treeFromString(robot_desc_string, kdl_tree)){
    ROS_ERROR("Failed to contruct kdl tree");
    return false;
  }
  if (!kdl_tree.getChain("base_link", endlink, kdl_chain)) {
    ROS_ERROR("Could not get KDL chain!");
    return 1;
  }

  // Subscriber/publisher setup
  joint_state_sub = node.subscribe("/joint_states", 1, &jointStateCallback);
  command_sub = node.subscribe("pose_target/command", 1, &commandCallback);
  joint_position_pub = node.advertise<std_msgs::Float64MultiArray>("blue_controllers/joint_position_controller/command", 1);

  // Start pose control
  ros::spin();

  return 0;
}
