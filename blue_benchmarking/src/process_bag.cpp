#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <blue_msgs/MotorState.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>


#include <rosbag/bag.h>
#include <iostream>
#include <fstream>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

KDL::Tree kdl_tree;
KDL::Chain kdl_chain;

std::string base_link;
tf2_ros::Buffer* tf_buffer;
tf2_ros::TransformListener* tf_listener;

Eigen::Matrix<double, 3, Eigen::Dynamic> target_position(3, 1);
KDL::Rotation target_rotation;

bool started = false;

std::vector<urdf::JointConstSharedPtr> joint_urdfs;

std::vector<std::string> joint_names;
std::vector<double> posture_target;
std::vector<double> posture_weights;

template <typename TParam>
void getRequiredParam(ros::NodeHandle &nh, const std::string name, TParam &dest) {
  if(!nh.getParam(name, dest)) {
    ROS_FATAL("Could not find %s parameter in namespace %s", name.c_str(), nh.getNamespace().c_str());
    ros::shutdown();
    exit(1);
  }
}

KDL::Frame get_ee_pose(const sensor_msgs::JointState msg) {
  int nj = kdl_chain.getNrOfJoints();

  // Load joint positions into KDL
  KDL::JntArray joint_positions = KDL::JntArray(nj);
  for (int i = 0; i < nj; i++) {
    for (int j = 0; j < msg.name.size(); j++) {
      if (msg.name[j].compare(joint_names[i]) == 0) {
        joint_positions(i) = msg.position[j];
        break;
      }
      if (j == msg.name.size() - 1) {
        ROS_ERROR("ERRORRORORORORO");
      }
    }
  }
  // Build KDL solvers
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

  // Compute cartesian position via KDL
  KDL::Frame cartesian_pose;
  if (fk_solver.JntToCart(joint_positions, cartesian_pose) < 0)
    ROS_ERROR("Forward kinematics failed");
  return cartesian_pose;
}

KDL::Frame get_ee_pose(const std_msgs::Float64MultiArray msg) {
  int nj = kdl_chain.getNrOfJoints();

  // Load joint positions into KDL
  KDL::JntArray joint_positions = KDL::JntArray(nj);
  for (int i = 0; i < nj; i++) {
      joint_positions(i) = msg.data[i];
  }
  // Build KDL solvers
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

  // Compute cartesian position via KDL
  KDL::Frame cartesian_pose;
  if (fk_solver.JntToCart(joint_positions, cartesian_pose) < 0)
    ROS_ERROR("Forward kinematics failed");
  return cartesian_pose;
}



void process_bag(char* file_name) {
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("joint_states"));
  topics.push_back(std::string("/joint_states"));
  topics.push_back(std::string("/right_arm/blue_controllers/joint_position_controller/command"));
  topics.push_back(std::string("/right_arm/motor_states"));
  topics.push_back(std::string("/right_arm/joint_imu"));
  topics.push_back(std::string("/forearm_tracker_pose"));
  topics.push_back(std::string("/Robot_1/pose"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::ofstream ee_file;
  ee_file.open ("ee.csv");
  ee_file << "time,x,y,z,qx,qy,qz,qw,j0,j1,j2,j3,j4,j5,j6" << std::endl;
  std::ofstream vive_file;
  vive_file.open ("vive.csv");
  vive_file << "time,x,y,z,qx,qy,qz,qw" << std::endl;
  std::ofstream cmd_file;
  cmd_file.open ("cmd.csv");
  cmd_file << "time,x,y,z,qx,qy,qz,qw,j0,j1,j2,j3,j4,j5,j6" << std::endl;
  std::ofstream motor_file;
  motor_file.open ("motor.csv");
  motor_file << "time,curr_cmd0,curr_cmd1,curr_cmd2,curr_cmd3,curr_cmd4,curr_cmd5,curr_cmd6" << std::endl;
  std::ofstream acc_file;
  acc_file.open ("acc.csv");
  acc_file << "time,ax0,ay0,az0,ax1,ay1,az1,ax2,ay2,az2,ax3,ay3,az3,ax4,ay4,az4,ax5,ay5,az5,ax6,ay6,az6" << std::endl;

  foreach(rosbag::MessageInstance const m, view) {
      ros::Time msg_time = m.getTime();

      sensor_msgs::JointState::ConstPtr js = m.instantiate<sensor_msgs::JointState>();
      if (js != NULL) {
        KDL::Frame ee_pose = get_ee_pose(*js);
        double qx;
        double qy;
        double qz;
        double qw;
        ee_pose.M.GetQuaternion(qx, qy, qz, qw);
        // Load joint positions into KDL
        int nj = kdl_chain.getNrOfJoints();
        KDL::JntArray jpositions = KDL::JntArray(nj);
        for (int i = 0; i < nj; i++) {
          for (int j = 0; j < js->name.size(); j++) {
            if (js->name[j].compare(joint_names[i]) == 0) {
              jpositions(i) = js->position[j];
              break;
            }
            if (j == js->name.size() - 1) {
              ROS_ERROR("ERRORRORORORORO");
            }
          }
        }

        ee_file << msg_time << ',';
        ee_file << ee_pose.p.x() << ',' << ee_pose.p.y() << ',' << ee_pose.p.z();
        ee_file << ',' << qx << ',' << qy << ',' << qz <<  ',' << qw;
        ee_file << ',' << jpositions(0) << ',' << jpositions(1) << ',' << jpositions(2) << ',' << jpositions(3) << ',' << jpositions(4) << ',' << jpositions(5) << ',' << jpositions(6) << std::endl;
      }

      geometry_msgs::Pose::ConstPtr vp = m.instantiate<geometry_msgs::Pose>();
      if (vp != NULL) {
        vive_file << msg_time << ',' << (*vp).position.x << ',' << (*vp).position.y << ',' << (*vp).position.z
                  << ',' << (*vp).orientation.x << ',' << (*vp).orientation.y << ',' << (*vp).orientation.z << ',' << (*vp).orientation.w << '\n';
      }
//
      std_msgs::Float64MultiArray::ConstPtr j_cmd_ptr = m.instantiate<std_msgs::Float64MultiArray>();
      if (j_cmd_ptr != NULL) {
        std::vector<double> j_cmd = j_cmd_ptr->data;
        KDL::Frame ee_pose = get_ee_pose(*j_cmd_ptr);

        double qx;
        double qy;
        double qz;
        double qw;
        ee_pose.M.GetQuaternion(qx, qy, qz, qw);
        cmd_file << msg_time << ',';
        cmd_file << ee_pose.p.x() << ',' << ee_pose.p.y() << ',' << ee_pose.p.z();
        cmd_file << ',' << qx << ',' << qy << ',' << qz <<  ',' << qw;
        cmd_file << ',' << j_cmd[0] << ',' << j_cmd[1] << ',' << j_cmd[2] << ',' << j_cmd[3] << ',' << j_cmd[4] << ',' << j_cmd[5] << ',' << j_cmd[6] << '\n';
      }


      blue_msgs::MotorState::ConstPtr m_ptr = m.instantiate<blue_msgs::MotorState>();
      if (m_ptr != NULL) {
        std::vector<float> cmd = m_ptr->command;
        motor_file << msg_time << ',';
        motor_file << cmd[0] << ',' << cmd[1] << ',' << cmd[2] << ',' << cmd[3] << ',' << cmd[4] << ',' << cmd[5] << ',' << cmd[6] << '\n';
      }

      // blue_msgs::ImuArray::ConstPtr a_ptr = m.instantiate<blue_msgs::ImuArray>();
      // if (a_ptr != NULL) {
        // std::vector<float> cmd = a_ptr->command;
        // motor_file << msg_time << ',';
        // motor_file << ',' << cmd[0] << ',' << cmd[1] << ',' << cmd[2] << ',' << cmd[3] << ',' << cmd[4] << ',' << cmd[5] << ',' << cmd[6] << '\n';
      // }
  }

  bag.close();
  ee_file.close();
  vive_file.close();
  cmd_file.close();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "inverse_kin_target");
  ros::NodeHandle node;
  if (argc != 2){
    ROS_ERROR("need bag file");
  }
  char* file_name = argv[1];
  ROS_ERROR("%s", file_name);

  tf_buffer = new tf2_ros::Buffer();
  tf_listener = new tf2_ros::TransformListener(*tf_buffer);

  // Parameters
  std::string robot_desc_string;
  std::string end_link;
  getRequiredParam(node, "/robot_description", robot_desc_string);
  getRequiredParam(node, "blue_hardware/baselink", base_link);
  getRequiredParam(node, "blue_hardware/endlink", end_link);
  getRequiredParam(node, "blue_hardware/joint_names", joint_names);

  urdf::Model urdf;
  if (!urdf.initParam("robot_description")) {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }

  for(unsigned int i = 0; i < joint_names.size(); i++) {
    urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names[i]);
    if (!joint_urdf)
    {
      ROS_ERROR("Could not find joint '%s' in urdf", joint_names[i].c_str());
      return false;
    }
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

  process_bag(file_name);
  return 0;
}
