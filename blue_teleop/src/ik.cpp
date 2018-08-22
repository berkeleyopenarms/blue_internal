#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>
#include <cmath>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

KDL::Tree kdl_tree;
KDL::Chain kdl_chain;

KDL::ChainJntToJacSolver* jac_solver;
KDL::ChainFkSolverPos_recursive* fk_solver;

ros::Publisher joint_position_pub;
ros::Subscriber joint_state_sub;
ros::Subscriber command_sub;

std::string base_link;
tf2_ros::Buffer* tf_buffer;
tf2_ros::TransformListener* tf_listener;

Eigen::Matrix<double, 3, Eigen::Dynamic> target_position(3, 1);
KDL::Rotation target_rotation;

bool started = false;
float lock_threshhold = 0.4;

std::vector<urdf::JointConstSharedPtr> joint_urdfs;

std::vector<std::string> joint_names;
std::vector<double> posture_target;
std::vector<double> posture_weights;

int nj;

Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd &mat, double tolerance) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::MatrixXd &singular_values = svd.singularValues();
    Eigen::Matrix<double, Eigen::MatrixXd::ColsAtCompileTime, Eigen::MatrixXd::RowsAtCompileTime> singular_values_inv(mat.cols(), mat.rows());
    singular_values_inv.setZero();
    for (unsigned int i = 0; i < singular_values.size(); i++) {
        if (singular_values(i) > tolerance)
            singular_values_inv(i, i) = 1 / singular_values(i);
        else
            singular_values_inv(i, i) = 0;
    }
    return svd.matrixV() * singular_values_inv * svd.matrixU().adjoint();
}

Eigen::Matrix<double, 6, Eigen::Dynamic> get_jacobian_eigen(KDL::JntArray joint_pos) {
    // Compute jacobian via KDL
    KDL::Jacobian jacobian(nj);
    if (jac_solver->JntToJac(joint_pos, jacobian, -1) < 0)
      ROS_ERROR("Jacobian solver failed");

    // Load it into Eigen
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_eigen(6, nj);
    for (int joint = 0; joint < nj; joint++) {
      for (int dim = 0; dim < 6; dim ++) {
        jacobian_eigen(dim, joint) = jacobian(dim, joint);
      }
    }
    return jacobian_eigen;
}

void solve_ik(KDL::JntArray &joint_positions_curr, Eigen::Matrix<double, 3, Eigen::Dynamic> target_pos, KDL::Rotation target_rot) {
  int iterations = 100;
  for (int i = 0; i < iterations; i++) {
    // get jacobian matrix
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_eigen = get_jacobian_eigen(joint_positions_curr);

    // Compute cartesian position via KDL
    KDL::Frame cartesian_pose;
    if (fk_solver->JntToCart(joint_positions_curr, cartesian_pose) < 0)
      ROS_ERROR("Forward kinematics failed");

    Eigen::Matrix<double, 3, Eigen::Dynamic> current_position(3, 1);
    for (int dim = 0; dim < 3; dim ++) {
      current_position(dim, 0) = cartesian_pose.p.data[dim];
    }

    KDL::Rotation rotation_difference = target_rot * cartesian_pose.M.Inverse();
    KDL::Vector rotation_difference_vec = rotation_difference.GetRot();

    Eigen::Matrix<double, 3, Eigen::Dynamic> position_difference = target_pos - current_position;
    Eigen::Matrix<double, 6, Eigen::Dynamic> deltaX(6,1);
    deltaX(0, 0) = position_difference(0, 0);
    deltaX(1, 0) = position_difference(1, 0);
    deltaX(2, 0) = position_difference(2, 0);
    // deltaX(3, 0) = rotation_difference_vec(0);
    // deltaX(4, 0) = rotation_difference_vec(1);
    // deltaX(5, 0) = rotation_difference_vec(2);
    deltaX(3, 0) = 0.0;
    deltaX(4, 0) = 0.0;
    deltaX(5, 0) = 0.0;

    double lambda = 5.0;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  I_nj(nj, nj);
    I_nj.setIdentity();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  I_6(6, 6);
    I_6.setIdentity();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> inv_reg = jacobian_eigen * jacobian_eigen.transpose() + lambda * lambda * I_6;
    inv_reg = inv_reg.inverse();
    Eigen::MatrixXd delta_joint = jacobian_eigen.transpose() * (inv_reg * deltaX);
    double alpha = 0.005;
    for (int j = 0; j < nj; j++) {
      joint_positions_curr(j) += alpha * delta_joint(j, 0);
    }

    // Null-space posture control

    // Eigen::Matrix<double, Eigen::Dynamic, 1>  posture_error(nj,1);
    // for (int j = 0; j < nj; j++) {
      // posture_error(j, 0) = posture_weights[j] * (posture_target[j] - joint_positions_curr(j));
    // }
    // Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian_pinv = pseudoinverse(jacobian_eigen, 0.000001);
    // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  nullspace_proj = I_nj - jacobian_pinv * jacobian_eigen;
    // Eigen::Matrix<double, Eigen::Dynamic, 1> posture_error_proj = nullspace_proj * posture_error;
    // for(int j = 0; j < nj; j++) {
      // joint_positions_curr(j) += posture_error_proj(j, 0);
    // }
//
    // Enforce joint limits
    for(int j = 0; j < nj; j++) {
      double &position = joint_positions_curr(j);
      // Check that this joint has applicable limits
      if (joint_urdfs[j]->type == urdf::Joint::REVOLUTE || joint_urdfs[j]->type == urdf::Joint::PRISMATIC)
      {
        if( position > joint_urdfs[j]->limits->upper ) // above upper limnit
          position = joint_urdfs[j]->limits->upper;
        else if( position < joint_urdfs[j]->limits->lower ) // below lower limit
          position = joint_urdfs[j]->limits->lower;
      }
    }

  }
}

void jointStateCallback(const sensor_msgs::JointState msg) {
  if (!started)
    return;

  // Load joint positions into KDL
  KDL::JntArray joint_positions = KDL::JntArray(nj);
  for (int i = 0; i < nj; i++) {
    for (int j = 0; j < msg.name.size(); j++) {
      if (msg.name[j].compare(joint_names[i]) == 0) {
        joint_positions(i) = msg.position[j];
        break;
      }
      if (j == msg.name.size() - 1) {
        return;
      }
    }
  }


  // Iteratively solve for a (regularized) IK solution
  KDL::JntArray joint_positions_ik = KDL::JntArray(nj);
  for (int i = 0; i < nj; i++)
    joint_positions_ik(i) = 1.0 * joint_positions(i) + 0.0 * posture_target[i];
    // joint_positions_ik(i) = 0.2 * joint_positions(i) + 0.8 * posture_target[i];
    // TODO: ^consider factoring posture weights into this expression


  Eigen::Matrix<double, 3, Eigen::Dynamic> target_position_local(3, 1);
  KDL::Rotation target_rotation_local;

  // ROS_ERROR("Joint test");
  // for(int i = 0; i < 7; i++){
    // ROS_ERROR("joint %f", joint_positions_ik(i));
  // }
  solve_ik(joint_positions_ik, target_position_local, target_rotation_local);


  std::vector<double> joint_command;

  for (int i = 0; i < nj; i++) {
    joint_command.push_back(joint_positions_ik(i));
  }

  // Publish the command
  std_msgs::Float64MultiArray joint_command_msg;
  for (int i = 0; i < nj; i++) {
    joint_command_msg.data.push_back(joint_command[i]);
  }
  // ROS_ERROR("published command %d, %f", i, commandMsg.data[i]);
  joint_position_pub.publish(joint_command_msg);
}

void commandCallback(geometry_msgs::PoseStamped msg) {
  geometry_msgs::TransformStamped t;
  t = tf_buffer->lookupTransform(base_link, msg.header.frame_id, ros::Time(0));
  tf2::doTransform(msg, msg, t);

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


  tf_buffer = new tf2_ros::Buffer();
  tf_listener = new tf2_ros::TransformListener(*tf_buffer);

  // Parameters
  std::string robot_desc_string;
  std::string end_link;
  getRequiredParam(node, "/robot_description", robot_desc_string);
  getRequiredParam(node, "blue_hardware/baselink", base_link);
  getRequiredParam(node, "blue_hardware/endlink", end_link);
  getRequiredParam(node, "blue_hardware/joint_names", joint_names);
  getRequiredParam(node, "blue_hardware/posture_target", posture_target);
  getRequiredParam(node, "blue_hardware/posture_weights", posture_weights);

  urdf::Model urdf;
  if (!urdf.initParam("robot_description")) {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }

  for(unsigned int i=0; i < joint_names.size(); i++) {
    urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names[i]);
    if (!joint_urdf)
    {
      ROS_ERROR("Could not find joint '%s' in urdf", joint_names[i].c_str());
      return false;
    }
    ROS_ERROR("joint '%s' in urdf", joint_names[i].c_str());
    joint_urdfs.push_back(joint_urdf);
  }

  // KDL setup
  if(!kdl_parser::treeFromString(robot_desc_string, kdl_tree)) {
    ROS_ERROR("Failed to contruct kdl tree");
    return false;
  }
  if (!kdl_tree.getChain(base_link, end_link, kdl_chain)) {
    ROS_ERROR("Could not get KDL chain!");
    return 1;
  }

  // Build KDL solvers
  jac_solver = new KDL::ChainJntToJacSolver(kdl_chain);
  fk_solver = new KDL::ChainFkSolverPos_recursive(kdl_chain);
  nj = kdl_chain.getNrOfJoints();

  // Subscriber/publisher setup
  joint_state_sub = node.subscribe("/joint_states", 1, &jointStateCallback);
  command_sub = node.subscribe("pose_target/command", 1, &commandCallback);
  joint_position_pub = node.advertise<std_msgs::Float64MultiArray>("blue_controllers/joint_position_controller/command", 1);

  // Start pose control
  ros::spin();

  return 0;
}
