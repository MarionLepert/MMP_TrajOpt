
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

// Redis
#include "RedisClient.h"

static RedisClient redis_client;

const std::string INITIAL_POSE_KEY = "mmp::initial_pose"; 
const std::string GOAL_STATE_KEY = "mmp::goal_state";

#define KINOVA


int main(int argc, char** argv)
{

  /************************************** Initialize ROS *************************************/
  ros::init(argc, argv, "move_test");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1); 
  spinner.start(); 
  ros::Rate loop_rate(10); // Hz

  ros::Publisher traj_pub = node_handle.advertise<geometry_msgs::PoseArray>("trajectory_points", 1000);

  /************************************ Initialize redis *************************************/
  redis_client = RedisClient();
  redis_client.connect("172.24.69.155", 6379);
  redis_client.authenticate("bohg");

  // Set initial value for initial pose as plaeholder 
  std::string initial_joint_values_str = "0 0 0 -1.57 0 1.57 0";
  redis_client.set(INITIAL_POSE_KEY, initial_joint_values_str); 

  std::string initial_goal_state_str = "-0.5 0 0.5"; 
  redis_client.set(GOAL_STATE_KEY, initial_goal_state_str); 


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

  // Initial Values 
  int dof = 7;

  /************************************** Begin loop *****************************************/

  while(ros::ok())
  {
    std::string initial_state = redis_client.get(INITIAL_POSE_KEY); 
    Eigen::VectorXd initial_joint_values = redis_client.decodeCPPRedisVector(initial_state);

    // ROS_INFO_STREAM("joint values: " << initial_joint_values);

    std::string goal_state = redis_client.get(GOAL_STATE_KEY); 
    Eigen::Vector3d goal_state_values = redis_client.decodeCPPRedisVector(goal_state); 

    ROS_INFO_STREAM("Goal state: " << goal_state_values); 

    kinematic_state->setJointGroupPositions(joint_model_group, initial_joint_values); 
    move_group.setStartState(*kinematic_state);


    /********************************* Plan to a goal pose ************************************/

    // We can plan a motion for this group to a desired pose for the end-effector.
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    // target_pose1.position.x = 0.5;
    // target_pose1.position.y = 0.1;
    // target_pose1.position.z = 0.6;
    target_pose1.position.x = goal_state_values(0);
    target_pose1.position.y = goal_state_values(1);
    target_pose1.position.z = goal_state_values(2);
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // /******************************** Get trajectory points ************************************/
    moveit_msgs::RobotTrajectory final_trajectory = my_plan.trajectory_; 
    
    // Convert trajectory into a series of RobotStates
    // Copy the vector of RobotStates to a RobotTrajectory
    robot_trajectory::RobotTrajectoryPtr robot_trajectory(
        new robot_trajectory::RobotTrajectory(robot_model, joint_model_group->getName()));

    robot_trajectory->setRobotTrajectoryMsg(*kinematic_state, final_trajectory);

    std::size_t num_waypoints = robot_trajectory->getWayPointCount(); 

    ROS_INFO_STREAM("num waypoints: " << num_waypoints); 


    std::vector<double> joint_values(dof, 0);

    Eigen::MatrixXd full_traj = Eigen::MatrixXd::Zero(num_waypoints,3); 

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

      full_traj.row(j) = end_effector_state.translation(); 

    }


    /**************************** Publish trajectory points **************************************/

    ROS_INFO_STREAM("Full traj: " << "\n" << full_traj); 

    geometry_msgs::PoseArray poses;
    poses.header.stamp = ros::Time::now(); 
    for (int j = 0; j < num_waypoints; j++)
    {
      geometry_msgs::Pose pose; 
      pose.position.x = full_traj(j,0); 
      pose.position.y = full_traj(j,1); 
      pose.position.z = full_traj(j,2); 
      poses.poses.push_back(pose); 
    }

    traj_pub.publish(poses); 

    loop_rate.sleep(); 
  }



  ros::waitForShutdown();
  return 0;
}
