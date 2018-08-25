#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <kdl_parser/kdl_parser.hpp> #include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>


#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

KDL::Tree kdl_tree;
KDL::Chain kdl_chain;

ros::Publisher joint_position_pub;
ros::Subscriber joint_state_sub;
ros::Subscriber command_sub;

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
        return;
      }
    }
  }
  // Build KDL solvers
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

  // Compute cartesian position via KDL
  KDL::Frame cartesian_pose;
  if (fk_solver.JntToCart(joint_positions_ik, cartesian_pose) < 0)
    ROS_ERROR("Forward kinematics failed");
  return cartesian_pose;
}

template <typename TParam>
void getRequiredParam(ros::NodeHandle &nh, const std::string name, TParam &dest) {
  if(!nh.getParam(name, dest)) {
    ROS_FATAL("Could not find %s parameter in namespace %s", name.c_str(), nh.getNamespace().c_str());
    ros::shutdown();
    exit(1);
  }
}


void process_bag(rosbag::Bag bag) {
  std::vector<std::string> topics;
  topics.push_back(std::string("/right_arm/joint_states"));
  topics.push_back(std::string("/right_arm/blue_controllers/joint_position_controller/command"));
  topics.push_back(std::string("/right_arm/motor_states"));
  topics.push_back(std::string("/base_tracker_link"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ofstream ee_file;
  ee_file.open ("ee.csv");
  ofstream vive_file;
  vive_file.open ("vive.csv");
  ofstream cmd_file;
  cmd_file.open ("vive.csv");

  foreach(rosbag::MessageInstance const m, view) {
      ros::Time msg_time = m.getTime();

      sensor_msgs::JointState::ConstPtr js = m.instantiate<sensor_msgs::JointState>();
      if (js != NULL) {
        ee_pose = get_ee_pose(js)
        double qx;
        double qy;
        double qz;
        double qw;
        ee_pose.M.GetQuaternion(qx, qy, qz, qw);
        ee_file << msg_time << ',' << ee_pose.p.x() << ',' << ee_pose.p.y() << ',' << ee_pose.p.z()
                << ',' << qx << ',' << qy << ',' << qz <<  ',' << qw
                << ',' << js[0] << ',' << js[1] << ',' << js[2] << ',' << js[3] << ',' << js[4] << ',' << js[5] << ',' << js[6] << '\n';
      }

      geometry_msgs::PoseStamped::ConstPtr vp = vp.minstantiate<geometry_msgs::PoseStamped>();
      if (vp != NULL) {
        vive_file << msg_time << ',' << vp.pose.position.x << ',' << vp.pose.position.y << ',' << vp.pose.position.z
                  << ',' << vp.pose.orientation.x << ',' << vp.pose.orientation.y << ',' << vp.pose.orientation.z << ',' << vp.pose.orientation.w << '\n';
      }

      std_msgs::Float64MultiArray::ConstPtr j_cmd;
      if (j_cmd != NULL) {
        cmd_file << msg_time << ',' << j_cmd[0] << ',' << j_cmd[1] << ',' << j_cmd[2] << ',' << j_cmd[3] << ',' << j_cmd[4] << ',' << j_cmd[5] << ',' << j_cmd[6] << '\n';
      }
  }

  bag.close();
  ee_file.close();
  vive_file.close();
  cmd_file.close();
}

int main(int argc, char** argv) {
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

  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Read);
  process_bag(bag);
  return 0;
}
