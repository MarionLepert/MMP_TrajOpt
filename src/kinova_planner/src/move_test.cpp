
/* Author: Marion Lepert */

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// msgs
#include <geometry_msgs/PoseArray.h>
#include "std_msgs/String.h"

// Conversions
#include <tf2_eigen/tf2_eigen.h>

// Transforms
#include <tf2_ros/transform_listener.h>

// C++
#include <string>
#include <algorithm>
#include <utility>
#include <vector>
#include <set>
#include <limits>
#include <sstream>

// Redis
#include "RedisClient.h"

static RedisClient redis_client;

const std::string INITIAL_POSE_KEY = "mmp::traj_initial_pose"; 
const std::string GOAL_STATE_KEY   = "mmp::traj_x_des";
const std::string ORI_DES_KEY      = "mmp::traj_ori_des"; 

#define KINOVA

#define STRINGS_EQUAL 0 

enum system_mode
{
  MMP,
  VEHICLE,
  ARM
};

enum system_mode sys_mode = ARM; // Default value: User needs to input sys_mode to run program 


int main(int argc, char** argv)
{
  /************************************** Initialize ROS *************************************/
  ros::init(argc, argv, "move_test");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1); 
  spinner.start(); 
  ros::Rate loop_rate(10); // Hz

  ros::Publisher traj_pub = node_handle.advertise<geometry_msgs::PoseArray>("trajectory_points", 1000);

  /***************************** Read command line arguments *********************************/
  std::istringstream ss(argv[1]); 
  int active_comp; 
  try {
    if (!(ss >> active_comp)) {
      throw std::runtime_error(std::string("ERROR: Invalid Number: ") + std::string(argv[1])); 
    } else if (!ss.eof()) {
      throw std::runtime_error(std::string("ERROR: Trailing characters after number")); 
    } else if ((active_comp < 0) || (active_comp > 3)) {
      throw std::runtime_error(std::string("ERROR: Input number out of range: ") + std::string(argv[1]));
    }
  } catch(const std::runtime_error& e) {
    std::cerr << e.what() << std::endl; 
    exit(0); 
  }

  if (std::string(argv[2]).compare("gen3") == STRINGS_EQUAL) {
    sys_mode = ARM; 
  } else if (std::string(argv[2]).compare("mmp") == STRINGS_EQUAL) {
    sys_mode = MMP; 
  } else {
    std::cerr << "Error: <SYS_MODE> parameter must be either arm or mmp" << std::endl; 
    exit(0); 
  } 

  std::string ip_address; 
  switch(active_comp)
  {
    case 0: 
      // ip_address = "172.24.69.155"; 
      ip_address = "192.168.1.25"; 
      break; 
    case 1: 
      ip_address = "172.24.69.149"; 
      break; 
    case 2: 
      ip_address = "172.24.69.150"; 
    case 3: 
      ip_address = "172.24.69.151"; 
  }

  /************************************ Initialize redis *************************************/
  redis_client = RedisClient();
  redis_client.connect(ip_address, 6379);
  redis_client.authenticate("bohg");

  std::string initial_joint_values_str; 
  // Set initial value for initial pose as placeholder 
#ifdef KINOVA
  if (sys_mode == MMP) {
    initial_joint_values_str = "0 0 0 0 0 0 1.57 0 0 0";
  } else {
    initial_joint_values_str = "0 0 0 1.57 0 0 0";
  }
#else
  // std::string initial_joint_values_str = "-0.52 -0.261 -0.261 -1.83 0.0 1.57 0.785";
  initial_joint_values_str = "-0.52 1.5 -0.261 0 0.0 1.57 0.785";
#endif 

  redis_client.set(INITIAL_POSE_KEY, initial_joint_values_str); 

  std::string initial_goal_state_str = "-0.5 0 0.5"; 
  redis_client.set(GOAL_STATE_KEY, initial_goal_state_str); 

  std::string initial_ori_des_str = "1 0 0 0"; 
  redis_client.set(ORI_DES_KEY, initial_ori_des_str); 

  /************************************ Initialize MoveIt ************************************/
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
#ifdef KINOVA
  static const std::string PLANNING_GROUP = "arm";
#else 
  static const std::string PLANNING_GROUP = "panda_arm";
#endif

  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Pointer to the robot model
  robot_model::RobotModelConstPtr robot_model = move_group.getRobotModel();

  // Pointer to the kinematic state of the robot 
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));

  int dof; 
  // Initial Values 
  if (sys_mode == MMP) { 
    dof = 10; 
  } else {
    dof = 7; 
  }

  /************************************** Begin loop *****************************************/
  while(ros::ok())
  {
    /**************************** Update joint values from redis ********************************/
    std::string initial_state = redis_client.get(INITIAL_POSE_KEY); 
    Eigen::VectorXd redis_joint_values = redis_client.decodeCPPRedisVector(initial_state);
    Eigen::VectorXd initial_joint_values = Eigen::VectorXd::Zero(dof); 

    // Account for potential disparity in dof between ROS and Sai2 models 
    if (redis_joint_values.size() < dof) {
      initial_joint_values.tail(7) = redis_joint_values; 
    } else if (redis_joint_values.size() > dof) {
      initial_joint_values = redis_joint_values.tail(7); 
    } else {
      initial_joint_values = redis_joint_values; 
    }

    ROS_INFO_STREAM("Initial pos: " << initial_joint_values.transpose());


    kinematic_state->setJointGroupPositions(joint_model_group, initial_joint_values); 
    move_group.setStartState(*kinematic_state);

    /************************* Update desired position and orientation *************************/
    // Position
    std::string x_des = redis_client.get(GOAL_STATE_KEY); 
    Eigen::Vector3d x_des_values = redis_client.decodeCPPRedisVector(x_des); 

    ROS_INFO_STREAM("Des pos: " << x_des_values.transpose()); 

    // Orientation 
    std::string ori_des = redis_client.get(ORI_DES_KEY); 
    Eigen::VectorXd ori_des_values = redis_client.decodeCPPRedisVector(ori_des); 

    ROS_INFO_STREAM("Des ori: " << ori_des_values.transpose()); 

    /********************************* Plan to a goal pose ************************************/
    // We can plan a motion for this group to a desired pose for the end-effector.
    geometry_msgs::Pose target_pose1;
    // Orientation
    target_pose1.orientation.w  = ori_des_values(0);
    target_pose1.orientation.x  = ori_des_values(1);
    target_pose1.orientation.y  = ori_des_values(2);
    target_pose1.orientation.z  = ori_des_values(3);
    // Position
    target_pose1.position.x = x_des_values(0);
    target_pose1.position.y = x_des_values(1);
    target_pose1.position.z = x_des_values(2);

    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    /******************************** Get trajectory points ************************************/
    moveit_msgs::RobotTrajectory final_trajectory = my_plan.trajectory_; 
    
    // Convert trajectory into a series of RobotStates
    // Copy the vector of RobotStates to a RobotTrajectory
    robot_trajectory::RobotTrajectoryPtr robot_trajectory(
        new robot_trajectory::RobotTrajectory(robot_model, joint_model_group->getName()));

    robot_trajectory->setRobotTrajectoryMsg(*kinematic_state, final_trajectory);

    std::size_t num_waypoints = robot_trajectory->getWayPointCount(); 

    ROS_INFO_STREAM("num waypoints: " << num_waypoints); 

    std::vector<double> joint_values(dof, 0);

    Eigen::MatrixXd full_traj_pos = Eigen::MatrixXd::Zero(num_waypoints,3); 

    std::vector<Eigen::Quaterniond> full_traj_ori;  

    for (int j = 0; j < num_waypoints; j++)
    {

      const robot_state::RobotState& rs = robot_trajectory->getWayPoint(j); 
      const std::vector<std::string>& var_names = rs.getVariableNames(); 

      // Get joint values 
      for (int i = 0; i < dof; i++)
      {
        #ifdef KINOVA
          joint_values[i] = rs.getVariablePosition(var_names.at(i)); 
        #else 
          int var_offset = 7;  
          joint_values[i] = rs.getVariablePosition(var_names.at(i+var_offset)); 
        #endif
      } 

      // Forward kinematics 
      kinematic_state->setJointGroupPositions(joint_model_group, joint_values); 

    #ifdef KINOVA
      const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("end_effector_link"); 
    #else 
      const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8"); 
    #endif

      full_traj_pos.row(j) = end_effector_state.translation(); 

      Eigen::Quaterniond ee_ori = Eigen::Quaterniond(end_effector_state.rotation()); 
      full_traj_ori.push_back(ee_ori); 

    }


    /**************************** Publish trajectory points **************************************/

    // ROS_INFO_STREAM("Full traj: " << "\n" << full_traj); 

    geometry_msgs::PoseArray poses;
    poses.header.stamp = ros::Time::now(); 
    for (int j = 0; j < num_waypoints; j++)
    {
      geometry_msgs::Pose pose; 
      pose.position.x = full_traj_pos(j,0); 
      pose.position.y = full_traj_pos(j,1); 
      pose.position.z = full_traj_pos(j,2); 

      Eigen::Quaterniond ee_ori = full_traj_ori.at(j); 
      pose.orientation.w = ee_ori.w(); 
      pose.orientation.x = ee_ori.x(); 
      pose.orientation.y = ee_ori.y(); 
      pose.orientation.z = ee_ori.z(); 

      poses.poses.push_back(pose); 
    }

    traj_pub.publish(poses); 

    loop_rate.sleep(); 
  }



  ros::waitForShutdown();
  return 0;
}
