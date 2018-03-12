#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <std_msgs/Float64.h>
#include <string>
#include <vector>
#include <kdl/frames.hpp>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tf2/LinearMath/Transform.h> 
KDL::Tree my_tree;
KDL::Chain chain;
std::vector<KDL::Chain> chains;


//std::string visualizer = "simple_6dof_MOVE_ROTATE_3D"` ;
class CalibrateJointState
{
public:

  geometry_msgs::PoseStamped base_tracker_pose;
  geometry_msgs::PoseStamped upper_arm_tracker_pose;
  geometry_msgs::PoseStamped forearm_tracker_pose;
  geometry_msgs::PoseStamped ee_tracker_pose;

  KDL::JntArray jointCur;
  KDL::JntArray jointOut;
  KDL::Jacobian jacobian;
  std::vector<KDL::ChainFkSolverPos_recursive*> fkSolvers;
  std::vector<KDL::ChainJntToJacSolver*> jacSolvers;
  int seq;
  unsigned int nj;
  double error_threshold;
  bool base_tracker_read;
  bool upper_arm_tracker_read;
  bool forearm_tracker_read;
  bool ee_tracker_read;
  std::vector<double> calibration_initial_angles;


  int num_iterations;

  CalibrateJointState(std::vector<KDL::Chain> chains) { 
    for (int i = 0; i < chains.size(); i++) { 
      KDL::ChainFkSolverPos_recursive* fksolver_i = new KDL::ChainFkSolverPos_recursive(chains[i]);
      fkSolvers.push_back(fksolver_i);
      KDL::ChainJntToJacSolver* jacsolver_i = new KDL::ChainJntToJacSolver(chains[i]);
      jacSolvers.push_back(jacsolver_i);
    } 
    seq = 0;
    pub = n_.advertise<sensor_msgs::JointState>("/joint_state_tracker", 1000);
    pubDeltaPos = n_.advertise<visualization_msgs::Marker>("/deltaPos", 1000);
    pubDeltaRot = n_.advertise<visualization_msgs::Marker>("/deltaRot", 1000);
    base_tracker_read = false;
    upper_arm_tracker_read = false;
    forearm_tracker_read = false;
    ee_tracker_read = false;
    subBaseTracker = n_.subscribe("/base_tracker_pose", 2, &CalibrateJointState::baseTrackerCallback, this);
    subUpperarmTracker = n_.subscribe("/upper_arm_tracker_pose", 2, &CalibrateJointState::upperarmTrackerCallback, this);
    subForearmTracker = n_.subscribe("/forearm_tracker_pose", 2, &CalibrateJointState::forearmTrackerCallback, this);
    subEeTracker = n_.subscribe("/ee_tracker_pose", 2, &CalibrateJointState::eeTrackerCallback, this);
    ROS_ERROR("Subscribed");
    ros::NodeHandle node;
    if (!node.getParam("koko_hardware/joint_names", joint_names)) {
      ROS_ERROR("No joint_names given (namespace: %s)", node.getNamespace().c_str());
    } 
    if (!node.getParam("koko_hardware/calibration_initial_angles", calibration_initial_angles)) {
      ROS_ERROR("No calibration_initial_angles given (namespace: %s)", node.getNamespace().c_str());
    } 
    nj = chains[chains.size() - 1].getNrOfJoints();
    jointCur = KDL::JntArray(nj);
    for (int i = 0; i < calibration_initial_angles.size(); i++) {
      jointCur(i) = calibration_initial_angles[i];
    }


    jointOut = KDL::JntArray(nj);
    //jacobian = KDL::Jacobian(nj);
    num_iterations = 15000;
    counter = 0; 
    error_threshold = 0.01;

  }

  ~CalibrateJointState() {
    for (int i = fkSolvers.size(); i > 0; i--) {
      delete fkSolvers[i];
      delete jacSolvers[i];
    }
  }
  
  
  void baseTrackerCallback(const geometry_msgs::PoseStamped msg)
  {
    if (!base_tracker_read)
    {
      base_tracker_read = true;
    }
    base_tracker_pose = msg;
  }

  void upperarmTrackerCallback(const geometry_msgs::PoseStamped msg)
  {
    if (!upper_arm_tracker_read)
    {
      upper_arm_tracker_read = true;
    }
    upper_arm_tracker_pose = msg;
  }

  void forearmTrackerCallback(const geometry_msgs::PoseStamped msg)
  {
    if (!forearm_tracker_read)
    {
      forearm_tracker_read = true;
    }
    forearm_tracker_pose = msg;
  }

  void eeTrackerCallback(const geometry_msgs::PoseStamped msg)
  {
    if (!ee_tracker_read)
    {
      ee_tracker_read = true;
    }
    ee_tracker_pose = msg;
    update();
  }

  void update()
  {
    //ROS_ERROR("c1");

    if (!(base_tracker_read && upper_arm_tracker_read && forearm_tracker_read && ee_tracker_read)){
      ROS_ERROR("Have not received all of: base, upper_arm, forearm, and ee trackers. Cannot calibrate until receiving all.");
      return;
    }
    //ROS_INFO("HAVE received both base and forearm trackers, kin 6");

    // grab local copies
    // TODO: de-hardcode this
    geometry_msgs::PoseStamped base_tracker_pose_local = base_tracker_pose;
    geometry_msgs::PoseStamped upper_arm_tracker_pose_local = upper_arm_tracker_pose;
    geometry_msgs::PoseStamped forearm_tracker_pose_local = forearm_tracker_pose;
    geometry_msgs::PoseStamped ee_tracker_pose_local = ee_tracker_pose;


    std::vector<geometry_msgs::PoseStamped> fk_poses_desired;
    fk_poses_desired.push_back(upper_arm_tracker_pose_local);
    fk_poses_desired.push_back(forearm_tracker_pose_local);
    fk_poses_desired.push_back(ee_tracker_pose_local);

    std::vector<unsigned int> num_joints_to_calibrate;
    num_joints_to_calibrate.push_back(3);
    num_joints_to_calibrate.push_back(5);
    num_joints_to_calibrate.push_back(7);

    std::vector<unsigned int> num_new_joints_to_calibrate;
    num_new_joints_to_calibrate.push_back(3);
    num_new_joints_to_calibrate.push_back(2);
    num_new_joints_to_calibrate.push_back(2);

    //ROS_ERROR("c2");
    
    for(int calibration_index = 0; calibration_index < fk_poses_desired.size(); calibration_index++) {

      // desired ee_pose
      Eigen::Matrix<double,3, Eigen::Dynamic> ee_pos_desired(3,1);

      geometry_msgs::PoseStamped fk_pose_msg = fk_poses_desired[calibration_index];
   
      ee_pos_desired(0, 0) = fk_pose_msg.pose.position.x;
      ee_pos_desired(1, 0) = fk_pose_msg.pose.position.y;
      ee_pos_desired(2, 0) = fk_pose_msg.pose.position.z;
      //ee_pose_desired(3, 0) = forearm_tracker_pose_local.pose.orientation.x;
      //ee_pose_desired(4, 0) = forearm_tracker_pose_local.pose.orientation.y;
      //ee_pose_desired(5, 0) = forearm_tracker_pose_local.pose.orientation.z;
      //ee_pose_desired(6, 0) = forearm_tracker_pose_local.pose.orientation.w;
      KDL::Rotation desired_rotation = KDL::Rotation::Quaternion(fk_pose_msg.pose.orientation.x,
                                                                 fk_pose_msg.pose.orientation.y,
                                                                 fk_pose_msg.pose.orientation.z,
                                                                 fk_pose_msg.pose.orientation.w
                                                                 );


      unsigned int cur_new_nj = num_new_joints_to_calibrate[calibration_index];
      unsigned int cur_nj = num_joints_to_calibrate[calibration_index];
      //ROS_ERROR("c3");


      KDL::Jacobian cur_jacobian = KDL::Jacobian(cur_nj);
      KDL::JntArray cur_joints_calibration = KDL::JntArray(cur_nj);


      // initialize joints from previous calibration steps
      int previous_calibrated_joints = cur_nj - cur_new_nj;
        for (int i = 0; i < cur_nj; i++){
          cur_joints_calibration(i) = jointCur(i);
          //ROS_ERROR("prev_calibrated: %d, index: %d, cur_nj: %d, cur_new_nj: %d", 
                  //previous_calibrated_joints, calibration_index, cur_nj, cur_new_nj);
        }
  
      //ROS_INFO("ee_pos_desired: %f %f %f", ee_pos_desired(0), ee_pos_desired(1), ee_pos_desired(2));
       //ROS_ERROR("c4"); 
  
      // do inverse kinematics
      for(int iteration = 0; iteration < num_iterations; iteration++)
      {
        jacSolvers[calibration_index]->JntToJac(cur_joints_calibration, cur_jacobian, -1);
        Eigen::Matrix<double,6,Eigen::Dynamic> jacMat(6,cur_new_nj);
  
  
        for (unsigned int joint = 0; joint < cur_new_nj; joint++) {
          for (unsigned int index = 0; index < 6; index ++) {
            jacMat(index,joint) = cur_jacobian(index,joint + previous_calibrated_joints);
            //ROS_ERROR("jac %d, %d : %f", index, joint, jacMat(index, joint));
          }
        } 
  
        // forward kinematics on current joint
        KDL::Frame cartpos;
        //ROS_ERROR("c5"); 
        int status = fkSolvers[calibration_index]->JntToCart(cur_joints_calibration, cartpos);
  
  
        Eigen::Matrix<double,3, Eigen::Dynamic> ee_pos_cur(3,1);
  
        for (int i = 0; i < 3; i ++) {
          ee_pos_cur(i, 0) = cartpos.p.data[i];
        }
        //ROS_ERROR("c6"); 
 
  
  
        KDL::Rotation rotation_difference = desired_rotation * cartpos.M.Inverse();
  
        KDL::Vector rotation_difference_vec = rotation_difference.GetRot();
  
  
        Eigen::Matrix<double, 3, Eigen::Dynamic> pos_difference = ee_pos_desired - ee_pos_cur;
       
        Eigen::Matrix<double, 6, Eigen::Dynamic> deltaX(6,1);
        //deltaX(0, 0) = 0;
        //deltaX(1, 0) = 0;
        //deltaX(2, 0) = 0;
        deltaX(0, 0) = pos_difference(0, 0);
        deltaX(1, 0) = pos_difference(1, 0);
        deltaX(2, 0) = pos_difference(2, 0);
        
        // deltaX(3, 0) = 0;
        // deltaX(4, 0) = 0;
        // deltaX(5, 0) = 0;
        
        deltaX(3, 0) = 0.025 * rotation_difference_vec(0);
        deltaX(4, 0) = 0.025 * rotation_difference_vec(1);
        deltaX(5, 0) = 0.025 * rotation_difference_vec(2);
        
        //ROS_ERROR("iteration %d, pos_difference: (%f, %f, %f, %f, %f, %f)", counter, pos_difference(0, 0), pos_difference(1, 0), pos_difference(2, 0), rotation_difference_vec(0),rotation_difference_vec(2),rotation_difference_vec(2) );
        counter++;
        // change for 6 koko_hardware version
  
      
  
  
        Eigen::MatrixXd deltaJoint = jacMat.transpose() * deltaX;
        
        float alpha = 0.15;
        for(int j = previous_calibrated_joints; j < cur_nj; j++)
        {
          cur_joints_calibration(j) = cur_joints_calibration(j) + alpha * deltaJoint(j - previous_calibrated_joints, 0);
         // ROS_ERROR("deltaJoint: %f", deltaJoint(j,0));
        } 
  
        int below_threshold = 1; 
        for (int i = 0; i < 6; i++) {
          if (std::fabs(deltaX(i, 0)) > error_threshold) {
            below_threshold = 0;
          }
        } 
        if (below_threshold == 1) {
          //ROS_ERROR("below threshold break");
        }
        //{ ROS_ERROR("deltajoint: %f", deltaJoint(0,0)); }
      }
      
      for(int j = previous_calibrated_joints; j < cur_nj; j++)
      {
          jointCur(j) = cur_joints_calibration(j);
          // ROS_ERROR("deltaJoint: %f", deltaJoint(j,0));
      } 
    }


    //ROS_INFO("jointCur: %f", jointCur(0));



    // publish and increment and save
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.seq = seq;
    joint_state_msg.header.frame_id = "base_link";
    joint_state_msg.header.stamp = ros::Time::now();

    for (int i = 0; i < joint_names.size(); i++) {
      joint_state_msg.name.push_back(joint_names[i]);
      joint_state_msg.position.push_back(jointCur(i));
      joint_state_msg.velocity.push_back(0.0);
      joint_state_msg.effort.push_back(0.0);
    }

    pub.publish(joint_state_msg);
    //jointCur = jointOut;
    seq++; 
  }
  

private:
  ros::NodeHandle n_;
  ros::Publisher pub;
  ros::Publisher pubDeltaPos;
  ros::Publisher pubDeltaRot;
  ros::Subscriber subBaseTracker;
  ros::Subscriber subUpperarmTracker;
  ros::Subscriber subForearmTracker;
  ros::Subscriber subEeTracker;
  std::vector<std::string> joint_names;
  int counter;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_state_tracker");
  ros::NodeHandle node;
  std::string robot_desc_string;

  node.getParam("robot_description", robot_desc_string);

  if(!kdl_parser::treeFromString(robot_desc_string, my_tree)){
    ROS_ERROR("Failed to contruct kdl tree");
    return false; 
  }

  std::string end_tracker_link;


  //if (!node.getParam("/koko_hardware/endlink_tracker",  end_tracker_link)) {
   // ROS_ERROR("No /koko_hardware/endlink_tracker loaded in rosparam");
  //}

  // TODO: un hard code this
  std::vector<std::string> tracker_names;
  tracker_names.push_back("upper_arm_tracker_link");
  tracker_names.push_back("forearm_tracker_link");
  tracker_names.push_back("ee_tracker_link");

  for (int i = 0; i < tracker_names.size(); i++) {
    KDL::Chain cur_chain;
    bool exit_value = my_tree.getChain("base_tracker_link", tracker_names[i], cur_chain);
    chains.push_back(cur_chain);
  }
  //bool exit_value = my_tree.getChain("base_tracker_link", end_tracker_link, chain);
  ros::Duration(2.0).sleep();
  CalibrateJointState sp(chains); 


/*
  ros::Rate  loop_rate(90);
  while(ros::ok){
    
    ros::spinOnce();
    loop_rate.sleep();
  }
*/ ros::spin(); 

  return 0; 
}
