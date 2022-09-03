#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <franka/exception.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>


// ROS
#include <ros/ros.h>
#include <manipulation_rubik/LfMoveLeft.h>
#include <manipulation_rubik/ResolveConfiguration.h>
#include <manipulation_rubik/MoveConfiguration.h>
#include "std_msgs/String.h"

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>


// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

bool resolveConfigurationRequest(manipulation_rubik::ResolveConfiguration::Request &req, manipulation_rubik::ResolveConfiguration::Response &res)
{

    manipulation_rubik::MoveConfiguration move1;
    move1.IsClockWise = true;
    move1.Move = "Top";
    res.result.push_back(move1);

    manipulation_rubik::MoveConfiguration move2;
    move2.IsClockWise = true;
    move2.Move = "Bottom";
    res.result.push_back(move2);

    manipulation_rubik::MoveConfiguration move3;
    move3.IsClockWise = false;
    move3.Move = "Bottom";
    res.result.push_back(move3);

    manipulation_rubik::MoveConfiguration move4;
    move4.IsClockWise = false;
    move4.Move = "Left";
    res.result.push_back(move4);

    manipulation_rubik::MoveConfiguration move5;
    move5.IsClockWise = false;
    move5.Move = "Front";
    res.result.push_back(move5);

    return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "logic");
  ros::NodeHandle nh;

  auto service1 = nh.advertiseService("resolve_configuration", resolveConfigurationRequest);
  
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::waitForShutdown();
  return 0;
}