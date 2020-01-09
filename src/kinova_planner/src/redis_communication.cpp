#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


#include <moveit_visual_tools/moveit_visual_tools.h>

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

#include <geometry_msgs/PoseArray.h>

// C++
#include <string>
#include <algorithm>
#include <utility>
#include <vector>
#include <set>
#include <limits>

#include "std_msgs/String.h"

#include "RedisClient.h"

static RedisClient redis_client;

const std::string TRAJECTORY_KEY = "mmp::trajectory"; 


void pathCallback(const geometry_msgs::PoseArray& msg)
{
	ROS_INFO("Message!!!!!!!!!!!!!!!!!!!!!!!!");

	// ROS_INFO_STREAM("ms size " << (msg.poses).size()); 

	int num_waypoints = (msg.poses).size();

  Eigen::MatrixXd full_traj = Eigen::MatrixXd::Zero(num_waypoints,3); 

	for(int j = 0; j < num_waypoints; j++)
	{
		geometry_msgs::Pose pose = msg.poses[j]; 
		full_traj(j,0) = pose.position.x; 
    full_traj(j,1) = pose.position.y; 
    full_traj(j,2) = pose.position.z; 
	}

  redis_client.setEigenMatrixJSON(TRAJECTORY_KEY, full_traj);

  // ROS_INFO_STREAM("Full traj: " << full_traj); 

}


int main(int argc, char** argv)
{
  /************************************** Initialize ROS *************************************/
  ros::init(argc, argv, "redis_talker");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /***************************** Read command line arguments *********************************/
  std::string param; 
  node_handle.getParam("active_comp", param);

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

  ros::Subscriber sub = node_handle.subscribe("trajectory_points", 1000, pathCallback); 

  ros::waitForShutdown(); 
  return 0;
}
