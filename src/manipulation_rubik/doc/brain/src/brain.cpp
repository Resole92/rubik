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
//#include <manipulation_rubik/MoveLeft.h>
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


ros::ServiceClient clientLeft;
ros::ServiceClient clientRight;
ros::ServiceClient clientTop;
ros::ServiceClient clientBottom;
ros::ServiceClient clientFront;
ros::ServiceClient clientBehind;


ros::ServiceClient clientMaintainTopRight;
ros::ServiceClient clientMaintainBehindRight;
ros::ServiceClient clientMaintainBottomLeft;
ros::ServiceClient clientMaintainFrontLeft;

ros::ServiceClient clientDoRotationRight;
ros::ServiceClient clientDoRotationLeft;

ros::ServiceClient clientLeaveMaintainRight;
ros::ServiceClient clientLeaveMaintainLeft;

ros::ServiceClient clientResetEnvironment;
ros::ServiceClient clientStartPositionRight;
ros::ServiceClient clientStartPositionLeft;

ros::ServiceClient clientPickRight;
ros::ServiceClient clientPickLeft;

void moveLeftPosition()
{
  manipulation_rubik::LfMoveLeft srv;
  clientLeft.call(srv);
}

void moveRightPosition()
{
  manipulation_rubik::LfMoveLeft srv;
  clientRight.call(srv);
}

void moveTopPosition()
{
  manipulation_rubik::LfMoveLeft srv;
  clientTop.call(srv);
}

void moveBottomPosition()
{
  manipulation_rubik::LfMoveLeft srv;
  clientBottom.call(srv);
}

void moveFrontPosition()
{
  manipulation_rubik::LfMoveLeft srv;
  clientFront.call(srv);
}

void moveBehindPosition()
{
  manipulation_rubik::LfMoveLeft srv;
  clientBehind.call(srv);
}

void maintainTopRight()
{
  manipulation_rubik::LfMoveLeft srv;
  clientMaintainTopRight.call(srv);
}

void maintainBehindRight()
{
  manipulation_rubik::LfMoveLeft srv;
  clientMaintainBehindRight.call(srv);
}

void maintainBottomLeft()
{
  manipulation_rubik::LfMoveLeft srv;
  clientMaintainBottomLeft.call(srv);
}

void maintainFrontLeft()
{
  manipulation_rubik::LfMoveLeft srv;
  clientMaintainFrontLeft.call(srv);
}

void leaveMaintainRight()
{
  manipulation_rubik::LfMoveLeft srv;
  clientLeaveMaintainRight.call(srv);
}

void leaveMaintainLeft()
{
  manipulation_rubik::LfMoveLeft srv;
  clientLeaveMaintainLeft.call(srv);
}

void doRotationRight()
{
  manipulation_rubik::LfMoveLeft srv;
  clientDoRotationRight.call(srv);
}

void doRotationLeft()
{
  manipulation_rubik::LfMoveLeft srv;
  clientDoRotationLeft.call(srv);
}

void pickRight()
{
  manipulation_rubik::LfMoveLeft srv;
  clientPickRight.call(srv);
}

void pickLeft()
{
  manipulation_rubik::LfMoveLeft srv;
  clientPickLeft.call(srv);
}


void rotateTopFace()
{
  moveLeftPosition();
  maintainTopRight();
  leaveMaintainRight();
  moveTopPosition();
  doRotationRight();
}

void rotateRightFace()
{
  moveLeftPosition();
  maintainTopRight();
  leaveMaintainRight();
  moveRightPosition();
  doRotationRight();
}

void rotateBehindFace()
{
  moveLeftPosition();
  maintainBehindRight();
  leaveMaintainRight();
  moveBehindPosition();
  doRotationRight();
}

void rotateBottomFace()
{
  moveRightPosition();
  maintainBottomLeft();
  leaveMaintainLeft();
  moveBottomPosition();
  doRotationLeft();
}

void rotateLeftFace()
{
  moveRightPosition();
  maintainBottomLeft();
  leaveMaintainLeft();
  moveLeftPosition();
  doRotationLeft();
}

void rotateFrontFace()
{
  moveRightPosition();
  maintainFrontLeft();
  leaveMaintainLeft();
  moveFrontPosition();
  doRotationLeft();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "brain");
  ros::NodeHandle nh;

  clientLeft = nh.serviceClient<manipulation_rubik::LfMoveLeft>("move_left");
  clientRight = nh.serviceClient<manipulation_rubik::LfMoveLeft>("move_right");
  clientTop = nh.serviceClient<manipulation_rubik::LfMoveLeft>("move_top");
  clientBottom = nh.serviceClient<manipulation_rubik::LfMoveLeft>("move_bottom");
  clientFront = nh.serviceClient<manipulation_rubik::LfMoveLeft>("move_front");
  clientBehind  = nh.serviceClient<manipulation_rubik::LfMoveLeft>("move_behind");

  clientMaintainTopRight = nh.serviceClient<manipulation_rubik::LfMoveLeft>("maintain_top_right");
  clientMaintainBehindRight = nh.serviceClient<manipulation_rubik::LfMoveLeft>("maintain_behind_right");
  clientMaintainBottomLeft = nh.serviceClient<manipulation_rubik::LfMoveLeft>("maintain_bottom_left");
  clientMaintainFrontLeft = nh.serviceClient<manipulation_rubik::LfMoveLeft>("maintain_front_left");

  clientDoRotationRight = nh.serviceClient<manipulation_rubik::LfMoveLeft>("do_rotation_right");
  clientDoRotationLeft = nh.serviceClient<manipulation_rubik::LfMoveLeft>("do_rotation_left");

  clientLeaveMaintainRight = nh.serviceClient<manipulation_rubik::LfMoveLeft>("leave_maintain_right");
  clientLeaveMaintainLeft = nh.serviceClient<manipulation_rubik::LfMoveLeft>("leave_maintain_left");

  clientResetEnvironment = nh.serviceClient<manipulation_rubik::LfMoveLeft>("reset_environment");
  clientStartPositionRight = nh.serviceClient<manipulation_rubik::LfMoveLeft>("start_position_right");
  clientStartPositionLeft = nh.serviceClient<manipulation_rubik::LfMoveLeft>("start_position_left");

  clientPickRight = nh.serviceClient<manipulation_rubik::LfMoveLeft>("pick_right");
  clientPickLeft = nh.serviceClient<manipulation_rubik::LfMoveLeft>("pick_left");

  pickRight();
  moveLeftPosition();
  
  rotateTopFace(); 
  rotateBottomFace();
  rotateRightFace();
  rotateLeftFace();
  rotateBehindFace();
  rotateFrontFace();

  ros::waitForShutdown();
  return 0;
}