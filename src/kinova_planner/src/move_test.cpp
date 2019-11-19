/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// MoveIt Messages
#include <moveit_msgs/CollisionObject.h>

// MoveIt
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/macros/console_colors.h>

// Conversions
#include <tf2_eigen/tf2_eigen.h>

// Transforms
#include <tf2_ros/transform_listener.h>

// Shape tools
#include <geometric_shapes/solid_primitive_dims.h>
#include <geometric_shapes/shape_operations.h>

// C++
#include <string>
#include <algorithm>
#include <utility>
#include <vector>
#include <set>
#include <limits>

#include <geometry_msgs/PoseArray.h>

#include "std_msgs/String.h"


int main(int argc, char** argv)
{

  /************************************** Initialize ROS *************************************/
  ros::init(argc, argv, "move_test");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1); 
  spinner.start(); 
  ros::Rate loop_rate(10); 


  /************************************ Initialize MoveIt ************************************/

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "panda_arm";

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
  kinematic_state = move_group.getCurrentState(); 

  /******************************** Initialize visualization **********************************/
  
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  /********************************* Plan to a goal pose ************************************/

  // We can plan a motion for this group to a desired pose for the end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.1;
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

  // Joint Values 
  int dof = 7;
  std::vector<double> joint_values(dof, 0);

  int var_offset = 7;  

  Eigen::MatrixXd full_traj = Eigen::MatrixXd::Zero(num_waypoints,3); 

  for (int j = 0; j < num_waypoints; j++)
  {

    const robot_state::RobotState& rs = robot_trajectory->getWayPoint(j); 
    const std::vector<std::string>& var_names = rs.getVariableNames(); 

    // Get joint values 
    for (int i = 0; i < dof; i++)
    {
      joint_values[i] = rs.getVariablePosition(var_names.at(i+var_offset)); 
    } 

    // Forward kinematics 
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values); 
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8"); 
    // ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n"); 
    // ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n"); 

    full_traj.row(j) = end_effector_state.translation(); 

  }


  ROS_INFO_STREAM("Full traj: " << "\n" << full_traj); 


  /**************************** Publish trajectory points **************************************/
  ros::Publisher traj_pub = node_handle.advertise<geometry_msgs::PoseArray>("trajectory_points", 1000); 
  while(ros::ok())
  {
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

  /********************************** Visualize Trajectory *************************************/

  // We can also visualize the plan as a line with markers in RViz.
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  ros::waitForShutdown();
  return 0;
}
