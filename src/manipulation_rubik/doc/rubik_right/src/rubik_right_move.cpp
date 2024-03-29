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
#include <manipulation_rubik/RotateCube.h>
#include <manipulation_rubik/Rotate.h>
#include <manipulation_rubik/PrepareForRotation.h>
#include <manipulation_rubik/LeaveObject.h>
#include <manipulation_rubik/MoveGripper.h>
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
//#include <src/manipulation_rubik/doc/arm/ArmBase.h>

bool isSimulation = true;

enum robotPositionSet { Left, Top, Right, Bottom, Front, Behind};
enum robotMaintainPosition { TopRight, BehindRight, BottomLeft, FrontLeft, None};

const double endEffectorLength = 0.125;
const double retreatLength = 0.09;
const double rubikDimension = 0.06;


const std::string tableName = "table";
const double xTablePosition = 0.65;
const double yTablePosition = 0.0;
const double tableHeight = 0.3;


const std::string rubikName = "cube_rubik";

const double xWorkPosition = 0.3;
const double yWorkPosition = 0;
const double zWorkPosition = 0.5;

ros::ServiceClient planning_scene_diff_client;
ros::Publisher planning_scene_diff_publisher;

robotPositionSet robotPositionSet1;
robotPositionSet robotPositionSet2;

robotMaintainPosition robotMaintainPosition1 = None;
robotMaintainPosition robotMaintainPosition2 = None;


const std::string pandaArmNumber = "_1";
const std::string endEffectorGroupName = "panda"+ pandaArmNumber +"_effector";
const std::string pandaArmGroupName = "panda"+ pandaArmNumber +"_arm";

geometry_msgs::Pose getCubeStartPose()
{
  
  geometry_msgs::Pose pose;
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, 0);
  
  pose.orientation = tf2::toMsg(orientation);
  pose.position.x = 0.5;
  pose.position.y = 0;
  pose.position.z = tableHeight + rubikDimension + 0.09;
  return pose;

}

bool moveToJointPosition(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double> joint_group_positions)
{
  move_group.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Fate il vostro piano
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success)
  {
    move_group.move();
  }

  return true;
}

void removeObject(std::string objectName)
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  std::vector<std::string> object_ids; 
  object_ids.push_back(objectName); 
  planning_scene_interface.removeCollisionObjects(object_ids);   
}

bool changeCheckContactWithObject(std::string objectName, bool disable, int pandaIdentifier)
{
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  planning_scene_monitor::LockedPlanningSceneRW planning_scene_mon = planning_scene_monitor::LockedPlanningSceneRW(psm);

  collision_detection::AllowedCollisionMatrix acm = planning_scene_mon->getAllowedCollisionMatrix();

  acm.setEntry("panda_1_leftfinger", objectName, disable);
  acm.setEntry("panda_1_rightfinger", objectName, disable);
  acm.setEntry("panda_1_hand", objectName, disable); 

  acm.setEntry("panda_2_leftfinger", objectName, disable);
  acm.setEntry("panda_2_rightfinger", objectName, disable);
  acm.setEntry("panda_2_hand", objectName, disable); 

  //acm.setEntry("panda_"+ std::to_string(pandaIdentifier) + "_leftfinger", objectName, disable);
  //acm.setEntry("panda_"+ std::to_string(pandaIdentifier) + "_rightfinger", objectName, disable);
  //acm.setEntry("panda_"+ std::to_string(pandaIdentifier) + "_hand", objectName, disable);

  if(disable)
  {
     std::cout << "Disable contact of panda arm "+ std::to_string(pandaIdentifier);
  }
  else
  {
    std::cout << "Enable contact of panda arm "+ std::to_string(pandaIdentifier);
  }
 


  acm.setEntry("panda_1_leftfinger", "panda_2_leftfinger", disable);
  acm.setEntry("panda_1_leftfinger", "panda_2_rightfinger", disable);
  acm.setEntry("panda_1_leftfinger", "panda_2_hand", disable);

  acm.setEntry("panda_1_rightfinger", "panda_2_leftfinger", disable);
  acm.setEntry("panda_1_rightfinger", "panda_2_rightfinger", disable);
  acm.setEntry("panda_1_rightfinger", "panda_2_hand", disable);

  acm.setEntry("panda_1_hand", "panda_2_leftfinger", disable);
  acm.setEntry("panda_1_hand", "panda_2_rightfinger", disable);
  acm.setEntry("panda_1_hand", "panda_2_hand", disable);

  acm.setEntry("panda_1_link8", "panda_2_leftfinger", disable);
  acm.setEntry("panda_1_link8", "panda_2_rightfinger", disable);
  acm.setEntry("panda_1_link8", "panda_2_hand", disable);

  acm.setEntry("panda_1_link7", "panda_2_leftfinger", disable);
  acm.setEntry("panda_1_link7", "panda_2_rightfinger", disable);
  acm.setEntry("panda_1_link7", "panda_2_hand", disable);

  acm.setEntry("panda_1_link6", "panda_2_leftfinger", disable);
  acm.setEntry("panda_1_link6", "panda_2_rightfinger", disable);
  acm.setEntry("panda_1_link6", "panda_2_hand", disable);

  acm.setEntry("panda_2_link8", "panda_1_leftfinger", disable);
  acm.setEntry("panda_2_link8", "panda_1_rightfinger", disable);
  acm.setEntry("panda_2_link8", "panda_1_hand", disable);

  acm.setEntry("panda_2_link7", "panda_1_leftfinger", disable);
  acm.setEntry("panda_2_link7", "panda_1_rightfinger", disable);
  acm.setEntry("panda_2_link7", "panda_1_hand", disable);

  acm.setEntry("panda_2_link6", "panda_1_leftfinger", disable);
  acm.setEntry("panda_2_link6", "panda_1_rightfinger", disable);
  acm.setEntry("panda_2_link6", "panda_1_hand", disable);

  moveit_msgs::PlanningScene planning_scene;

  acm.getMessage(planning_scene.allowed_collision_matrix);
  planning_scene.is_diff = true;
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);
}

std::string getArmIp(int pandaIdentifier)
{
   ros::NodeHandle nh;
   std::string ipRobotArm = "default";
   nh.getParam("combined_panda/panda_" + std::to_string(pandaIdentifier) + "/robot_ip", ipRobotArm);
   return ipRobotArm;
}

void openGripperManually(int pandaIdentifier, bool isObjectToDetach, std::string objectToDetach = rubikName)
{
  auto groupName = "panda_" + std::to_string(pandaIdentifier) + "_effector";
  moveit::planning_interface::MoveGroupInterface group(groupName); 
  auto joint_position = {0.04, 0.04};
  auto success = moveToJointPosition(group, joint_position);

  if(isObjectToDetach)
  {
    group.detachObject(objectToDetach);
  }
  
  if(!isSimulation)
  {
    auto armIp = getArmIp(pandaIdentifier);
    franka::Gripper gripper(armIp);
    gripper.move(0.006, 0.1);
  }
}

void closedGripperManually(int pandaIdentifier, bool isMainteined = false, std::string objectToDetach = rubikName)
{
  auto groupName = "panda_" + std::to_string(pandaIdentifier) + "_effector";
  moveit::planning_interface::MoveGroupInterface group(groupName); 

  changeCheckContactWithObject(objectToDetach, true, pandaIdentifier);
  auto joint_position = {0.03, 0.03};
  auto success = moveToJointPosition(group, joint_position);

  if(!isMainteined)
  {
    ROS_INFO("Attach object");
    group.attachObject(objectToDetach);
  }
  
  if(!isSimulation)
  {
    auto armIp = getArmIp(pandaIdentifier);
    franka::Gripper gripper(armIp);
    gripper.grasp(0.045, 0.1, 10.0);
  }
}

bool canMoveClockWise(int pandaIdentifier)
{
  auto pandaArmName = "panda_"+ std::to_string(pandaIdentifier) +"_arm";

  moveit::planning_interface::MoveGroupInterface group(pandaArmName);
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = group.getCurrentState();

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(pandaArmName);
  
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  if(joint_group_positions[6] > 0){
    return false;
  }
    
  return true;
}

double getRotationEndEffector(int pandaIdentifier)
{
  auto canClockWise = canMoveClockWise(pandaIdentifier);

  auto rotation = M_PI /2;
  if(!canClockWise)
  {
    return rotation;
  }
    
  return -rotation;
}

void print_actual_cartesian_position(moveit::planning_interface::MoveGroupInterface& move_group)
{
   geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

  ROS_INFO("Position x is: %f", current_pose.pose.position.x  );
  ROS_INFO("Position y is: %f", current_pose.pose.position.y  );
  ROS_INFO("Position z is: %f", current_pose.pose.position.z  );
}

void print_pose_position(geometry_msgs::Pose& pose)
{
  ROS_INFO("Position x is: %f", pose.position.x  );
  ROS_INFO("Position y is: %f", pose.position.y  );
  ROS_INFO("Position z is: %f", pose.position.z  );
}

void print_joint_values(moveit::planning_interface::MoveGroupInterface& move_group)
{

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(pandaArmGroupName);
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

   for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_group_positions[i]);
  }
}

bool changeCheckSelfContact(bool disable)
{
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  planning_scene_monitor::LockedPlanningSceneRW planning_scene_mon = planning_scene_monitor::LockedPlanningSceneRW(psm);

  collision_detection::AllowedCollisionMatrix acm = planning_scene_mon->getAllowedCollisionMatrix();

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  planning_scene_mon->checkSelfCollision(collision_request, collision_result); 

  collision_detection::CollisionResult::ContactMap::const_iterator it2;
  for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
  {
    acm.setEntry(it2->first.first, it2->first.second, disable);
  }  

  moveit_msgs::PlanningScene planning_scene;

  acm.getMessage(planning_scene.allowed_collision_matrix);
  planning_scene.is_diff = true;
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);
}

bool moveToCartesianPath(int pandaIdentifier, double x, double y, double z)
{
  moveit::planning_interface::MoveGroupInterface group("panda_" + std::to_string(pandaIdentifier) + "_arm");

  changeCheckContactWithObject(rubikName, true, pandaIdentifier);

  group.setStartStateToCurrentState();
  group.clearPathConstraints();
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
  geometry_msgs::Pose target_pose1 = current_pose.pose;
  target_pose1.position.x +=  x;
  target_pose1.position.y +=  y;
  target_pose1.position.z +=  z;

  waypoints.push_back(target_pose1);

  //group.setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.trajectory_ = trajectory; 

  const int wait_time = 1;
  //ros::Duration(wait_time).sleep();

  group.execute(my_plan);

  changeCheckContactWithObject(rubikName, false, pandaIdentifier);
  //ros::Duration(wait_time).sleep();
}

bool moveToCartesianPath(int pandaIdentifier, geometry_msgs::Pose& pose)
{
  moveit::planning_interface::MoveGroupInterface group("panda_" + std::to_string(pandaIdentifier) + "_arm");

  changeCheckContactWithObject(rubikName, true, pandaIdentifier);

  group.setStartStateToCurrentState();
  group.clearPathConstraints();
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
  geometry_msgs::Pose target_pose1 = current_pose.pose;
  target_pose1.position.x = pose.position.x;
  target_pose1.position.y = pose.position.y;
  target_pose1.position.z = pose.position.z;

  waypoints.push_back(target_pose1);

  //group.setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.trajectory_ = trajectory; 

  const int wait_time = 1;
  //ros::Duration(wait_time).sleep();

  group.execute(my_plan);

  changeCheckContactWithObject(rubikName, false, pandaIdentifier);

  //ros::Duration(wait_time).sleep();
}

void doTransformation(std::string from_frame, std::string to_frame, geometry_msgs::Pose& target_pose)
{

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);
  geometry_msgs::TransformStamped from_frame_to_frame; // My frames are named "base_link" and "leap_motion"

  from_frame_to_frame = tf_buffer.lookupTransform(from_frame, to_frame, ros::Time(0), ros::Duration(1.0) );

  tf2::doTransform(target_pose, target_pose, from_frame_to_frame); // robot_pose is the PoseStamped I want to transform
}

void checkLimits(int pandaIdentifier)
{
  auto identifier = std::to_string(pandaIdentifier);
  auto pandaName =  "panda_" + identifier;

  std::string jointsName [] = {
    pandaName +"_joint1",
    pandaName +"_joint2",
    pandaName +"_joint3",
    pandaName +"_joint4",
    pandaName +"_joint5",
    pandaName +"_joint6",
    pandaName +"_joint7"};

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  for(int i = 0; i < 7; i ++ )
  {
    auto jointName = jointsName[i];

    robot_model::JointModel* joint_model = kinematic_model->getJointModel(jointName);
    auto limits = joint_model->getVariableBoundsMsg();
    for(moveit_msgs::JointLimits joint_limits_check : limits){
          // Stampo i limiti di posizione del giunto

          std::cout << "Joint " << joint_model->getName() << " for panda arm " + identifier + " has position limits:" << std::endl;
          std::cout << "  Min position: " << joint_limits_check.min_position << std::endl;
          std::cout << "  Max position: " << joint_limits_check.max_position << std::endl;
    }   
  }
}

void moveToTargetPose(int pandaIdentifier, geometry_msgs::Pose& pose)
{
  auto pandaArmName = "panda_"+ std::to_string(pandaIdentifier) +"_arm";
  moveit::planning_interface::MoveGroupInterface group(pandaArmName);

  //applyJointContraint(pandaIdentifier, group);
  
  group.setPoseTarget(pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  group.move();
}

void rotateEndEffector(int pandaIdentifier, double rotation_radiants)
{
  auto pandaArmName = "panda_"+ std::to_string(pandaIdentifier) +"_arm";

  moveit::planning_interface::MoveGroupInterface group(pandaArmName);
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = group.getCurrentState();

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(pandaArmName);
  
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[6] =  joint_group_positions[6] + rotation_radiants;   // radians
  group.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //Fate il vostro piano
  //bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //if(success)
  {
    group.move();

  }
}

geometry_msgs::Pose workPose1;
geometry_msgs::Pose workPose2;

void moveWorkPosition(int pandaIdentifier)
{
  //auto joint_position = {1.415658, -0.046469, -0.945388, -1.562795, -1.619428, 1.101078, 2.328594};
  //auto success = moveToJointPosition(group, joint_position);

  geometry_msgs::Pose pose;
  pose.position.z = zWorkPosition;
  pose.position.y = yWorkPosition + endEffectorLength;
  pose.position.x = xWorkPosition;
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI/2, 3*M_PI/4 , M_PI);
  pose.orientation = tf2::toMsg(orientation);

  moveToTargetPose(pandaIdentifier, pose);

  workPose1 = pose;

  robotPositionSet1 = Right;
}

void moveRightPosition(int pandaIdentifier = 1)
{
  moveit::planning_interface::MoveGroupInterface group("panda_" + std::to_string(pandaIdentifier) + "_arm");

  geometry_msgs::Pose pose;
  pose.position.z = zWorkPosition;
  pose.position.y = yWorkPosition + endEffectorLength + retreatLength;
  pose.position.x = xWorkPosition;
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI/2, 3*M_PI/4 , M_PI);
  pose.orientation = tf2::toMsg(orientation);

  moveToTargetPose(pandaIdentifier, pose);

  pose.position.y -= retreatLength;
  workPose1 = pose;
  robotPositionSet1 = Right;
}

bool moveRightPositionRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Move Right");
  moveRightPosition();
  return true;
}

void moveBehindPosition(int pandaIdentifier = 1)
{
  moveit::planning_interface::MoveGroupInterface group("panda_" + std::to_string(pandaIdentifier) + "_arm");

  geometry_msgs::Pose pose;
  tf2::Quaternion orientation;
  //Second for rotating
  orientation.setRPY(M_PI/2, 5*M_PI/4 , M_PI/2);
  pose.orientation = tf2::toMsg(orientation);
  pose.position.z = zWorkPosition;
  pose.position.y = yWorkPosition;
  pose.position.x = xWorkPosition - endEffectorLength - retreatLength;

  moveToTargetPose(pandaIdentifier, pose);

  pose.position.x += retreatLength;
  workPose1 = pose;
  robotPositionSet1 = Behind;
}

bool moveBehindPositionRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Move Behind");
  moveBehindPosition();
  return true;
}

void moveTopPosition(int pandaIdentifier = 1)
{
  moveit::planning_interface::MoveGroupInterface group("panda_" + std::to_string(pandaIdentifier) + "_arm");

  geometry_msgs::Pose pose;
  tf2::Quaternion orientation;
  orientation.setRPY(0, M_PI , -M_PI/4);
  pose.orientation = tf2::toMsg(orientation);
  pose.position.z = zWorkPosition + endEffectorLength + retreatLength;
  pose.position.y = yWorkPosition;
  pose.position.x = xWorkPosition;

  moveToTargetPose(pandaIdentifier, pose);

  pose.position.z -= retreatLength;
  workPose1 = pose;
  robotPositionSet1 = Top;
}

bool moveTopPositionRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Move Top");
  moveTopPosition();
  return true;
}

void moveLeftPosition(int pandaIdentifier = 2)
{
  moveit::planning_interface::MoveGroupInterface group("panda_" + std::to_string(pandaIdentifier) + "_arm");

  geometry_msgs::Pose pose;
  tf2::Quaternion orientation;
  orientation.setRPY(M_PI/2, M_PI/4 , M_PI);
  pose.orientation = tf2::toMsg(orientation);
  pose.position.z = zWorkPosition;
  pose.position.y = yWorkPosition - endEffectorLength  - retreatLength; 
  pose.position.x = xWorkPosition;

  moveToTargetPose(pandaIdentifier, pose);

  pose.position.y += retreatLength;
  workPose2 = pose;
  robotPositionSet2 = Left;
}

bool moveLeftPositionRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Move Left");
  moveLeftPosition();
  return true;
}

void moveFrontPosition(int pandaIdentifier = 2)
{
  moveit::planning_interface::MoveGroupInterface group("panda_" + std::to_string(pandaIdentifier) + "_arm");

  geometry_msgs::Pose pose;
  tf2::Quaternion orientation;
  //Second for rotating
  orientation.setRPY(M_PI/2, 5*M_PI/4 , -M_PI/2);
  pose.orientation = tf2::toMsg(orientation);
  pose.position.z = zWorkPosition;
  pose.position.y = yWorkPosition;
  pose.position.x = xWorkPosition + endEffectorLength + retreatLength;

  moveToTargetPose(pandaIdentifier, pose);

  pose.position.x -= retreatLength;
  workPose2 = pose;
  robotPositionSet2 = Front;
}

bool moveFrontPositionRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Move Front");
  moveFrontPosition();
  return true;
}

void moveBottomPosition(int pandaIdentifier = 2)
{
  moveit::planning_interface::MoveGroupInterface group("panda_" + std::to_string(pandaIdentifier) + "_arm");

  geometry_msgs::Pose pose;
  tf2::Quaternion orientation;
  //Second for rotating
  orientation.setRPY(0, 0 , M_PI/4);
  pose.orientation = tf2::toMsg(orientation);
  pose.position.z = zWorkPosition - endEffectorLength - retreatLength;
  pose.position.y = yWorkPosition - endEffectorLength - retreatLength;
  pose.position.x = xWorkPosition;

  moveToTargetPose(pandaIdentifier, pose);

  pose.position.y += endEffectorLength + retreatLength;

  moveToCartesianPath(pandaIdentifier, pose);

  pose.position.z += retreatLength;
  workPose2 = pose;
  robotPositionSet2 = Bottom;
}

bool moveBottomPositionRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Move Bottom");
  moveBottomPosition();
  return true;
}

void addCollisionObjects()
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  collision_objects[0].id = tableName;
  collision_objects[0].header.frame_id = "base";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = tableHeight;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = xTablePosition;
  collision_objects[0].primitive_poses[0].position.y = yTablePosition;
  collision_objects[0].primitive_poses[0].position.z = tableHeight/2;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Define the object that we will be manipulating
  collision_objects[1].header.frame_id = "base";
  collision_objects[1].id = rubikName;

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = rubikDimension;
  collision_objects[1].primitives[0].dimensions[1] = rubikDimension;
  collision_objects[1].primitives[0].dimensions[2] = rubikDimension;

  /* Define the pose of the object. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = xTablePosition;
  collision_objects[1].primitive_poses[0].position.y = yTablePosition;
  collision_objects[1].primitive_poses[0].position.z = tableHeight + rubikDimension/2;

  collision_objects[1].operation = collision_objects[1].ADD;


   // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "base";
  collision_objects[2].id = "wall";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 1;
  collision_objects[2].primitives[0].dimensions[1] = 2;
  collision_objects[2].primitives[0].dimensions[2] = 1;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = -0.9;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;

  collision_objects[2].operation = collision_objects[1].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);

  ros::WallDuration(1.0).sleep();
}

bool resetEnvironmentRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Reset Behaviour");
  addCollisionObjects();
  return true;
}

void startPosition(int pandaIdentifier)
{
  moveit::planning_interface::MoveGroupInterface group("panda_" + std::to_string(pandaIdentifier) + "_arm");

  auto joint_position = { 0.000000, -0.785000, 0.000000, -2.356000, 0.000000, 1.571000, 0.785000};
  auto success = moveToJointPosition(group, joint_position);
}

bool startPositionRightRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Move Start Position Right");
  startPosition(1);
  return true;
}

bool startPositionLeftRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Move Start Position Left");
  startPosition(2);
  return true;
}

void pickObject(int pandaIdentifier)
{
  geometry_msgs::Pose pose;
  pose.position.x = xTablePosition;
  pose.position.y = yTablePosition;
  pose.position.z = tableHeight + rubikDimension/2 + endEffectorLength;

  moveToCartesianPath(pandaIdentifier, pose);
  closedGripperManually(pandaIdentifier);
  ros::WallDuration(1.0).sleep();
}

bool pickRightRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Pick Right");
  pickObject(1);
  moveWorkPosition(1);

  auto groupName = "panda_1_effector";
  moveit::planning_interface::MoveGroupInterface group(groupName); 
  group.detachObject(rubikName);

  return true;
}

bool pickLeftRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Pick Left");
  pickObject(2);
  moveWorkPosition(2);

  auto groupName = "panda_2_effector";
  moveit::planning_interface::MoveGroupInterface group(groupName); 
  group.detachObject(rubikName);

  return true;
}

void placeObject(int pandaIdentifier)
{
  changeCheckContactWithObject(rubikName, true, pandaIdentifier);
  startPosition(pandaIdentifier);

  geometry_msgs::Pose pose;
  pose.position.x = xTablePosition;
  pose.position.y = yTablePosition;
  pose.position.z = tableHeight + rubikDimension/2 + endEffectorLength - 0.01;

  moveToCartesianPath(pandaIdentifier, pose);
  openGripperManually(pandaIdentifier, true);
  ros::WallDuration(1.0).sleep();
}

bool placeRightRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Place Right");
  auto groupName = "panda_1_effector";
  moveit::planning_interface::MoveGroupInterface group(groupName); 
  group.attachObject(rubikName);
  placeObject(1);
  return true;
}

bool placeLeftRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Place Left");

  auto groupName = "panda_2_effector";
  moveit::planning_interface::MoveGroupInterface group(groupName); 
  group.attachObject(rubikName);
  placeObject(2);
  return true;
}

void resetBehaviorAndPickFromStart()
{
  addCollisionObjects();
  startPosition(1);
  startPosition(2);  
}

bool changeCheckCustom(bool disable)
{
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  planning_scene_monitor::LockedPlanningSceneRW planning_scene_mon = planning_scene_monitor::LockedPlanningSceneRW(psm);

  collision_detection::AllowedCollisionMatrix acm = planning_scene_mon->getAllowedCollisionMatrix();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda"+ pandaArmNumber +"_arm");
  std::vector<std::string> joint_names;
  joint_names.push_back("panda"+ pandaArmNumber +"_link0");
  joint_names.push_back("panda"+ pandaArmNumber +"_link1");
  joint_names.push_back("panda"+ pandaArmNumber +"_link2");
  joint_names.push_back("panda"+ pandaArmNumber +"_link3");
  joint_names.push_back("panda"+ pandaArmNumber +"_link4");
  joint_names.push_back("panda"+ pandaArmNumber +"_link5");
  joint_names.push_back("panda"+ pandaArmNumber +"_link6");
  joint_names.push_back("panda"+ pandaArmNumber +"_link7");
  joint_names.push_back("panda"+ pandaArmNumber +"_link8");

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
     ROS_INFO("Joint %s", joint_names[i].c_str());
    for (std::size_t j = 0; j < joint_names.size(); ++j)
    {
      acm.setEntry(joint_names[i].c_str(), joint_names[j].c_str(), disable);
     
    }
    acm.setEntry("panda"+ pandaArmNumber +"_hand", joint_names[i].c_str(), disable);
    acm.setEntry("panda"+ pandaArmNumber +"_rightfinger", joint_names[i].c_str(), disable);
    acm.setEntry("panda"+ pandaArmNumber +"_leftfinger", joint_names[i].c_str(), disable);
  }

  moveit_msgs::PlanningScene planning_scene;

  acm.getMessage(planning_scene.allowed_collision_matrix);
  planning_scene.is_diff = true;
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);
}

void leaveMantainAndRetreat(int pandaIdentifier)
{
 
  if(pandaIdentifier == 1)
  {
    auto targetPose = workPose1;
    auto rotation = getRotationEndEffector(pandaIdentifier);

    openGripperManually(pandaIdentifier, false);

    if(robotMaintainPosition1 == FrontLeft)
    {
      targetPose.position.y += retreatLength;
      moveToCartesianPath(pandaIdentifier, targetPose);
      rotateEndEffector(pandaIdentifier, -rotation);
    }

    if(robotMaintainPosition1 == BottomLeft)
    {
      targetPose.position.y += retreatLength;
      moveToCartesianPath(pandaIdentifier, targetPose);
    }

    robotMaintainPosition1 = None;

  }

  if(pandaIdentifier == 2)
  {
    auto targetPose = workPose2;
    auto rotation = getRotationEndEffector(pandaIdentifier);

    openGripperManually(pandaIdentifier, false);

    if(robotMaintainPosition2 == BehindRight)
    {
      targetPose.position.y -= retreatLength;
      moveToCartesianPath(pandaIdentifier, targetPose);
      rotateEndEffector(pandaIdentifier, -rotation);
    }

    if(robotMaintainPosition2 == TopRight)
    {
      targetPose.position.y -= retreatLength;
      moveToCartesianPath(pandaIdentifier, targetPose);
    }
    
    robotMaintainPosition2 = None;
  }

}

bool leaveMantainRightRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Leave Maintain Right");
  leaveMantainAndRetreat(1);
  return true;
}

bool leaveMantainLeftRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Leave Maintain Left");
  leaveMantainAndRetreat(2);
  return true;
}

void leaveObjectAndRetreat(int pandaIdentifier, bool isToDetach, std::string objectName = rubikName)
{
  openGripperManually(pandaIdentifier, isToDetach);

  if(pandaIdentifier == 1 && robotPositionSet1 == Right)
  {
    moveToCartesianPath(pandaIdentifier, 0, retreatLength, 0);
  }

  if(pandaIdentifier == 1 &&  robotPositionSet1 == Top)
  {
    moveToCartesianPath(pandaIdentifier, 0, 0, retreatLength);
  }

  if(pandaIdentifier == 1 && robotPositionSet1 == Behind)
  {
    moveToCartesianPath(pandaIdentifier, -retreatLength, 0, 0);
  }

  if(pandaIdentifier == 2 && robotPositionSet2 == Front)
  {
    moveToCartesianPath(pandaIdentifier, retreatLength, 0, 0);
  }

  if(pandaIdentifier == 2 && robotPositionSet2 == Bottom)
  {
    moveToCartesianPath(pandaIdentifier, 0, 0, -retreatLength);
  }

  if(pandaIdentifier == 2 && robotPositionSet2 == Left)
  {
    moveToCartesianPath(pandaIdentifier, 0, -retreatLength, 0);
  }

  
}

void keepObject(int pandaIdentifier, std::string objectName)
{
  auto targetPose = pandaIdentifier == 1 ? workPose1 : workPose2;

  moveToCartesianPath(pandaIdentifier, targetPose);
  closedGripperManually(pandaIdentifier);
}

void maintainObjectPanda(robotMaintainPosition mantainPosition, std::string objectName = rubikName)
{
  geometry_msgs::Pose targetPose;
  int pandaIdentifier;

  if(mantainPosition == robotMaintainPosition1 ||  mantainPosition == robotMaintainPosition2) 
  {
    return;
  }

  if(mantainPosition == FrontLeft)
  {
    moveRightPosition();
    targetPose = workPose1;
    pandaIdentifier = 1;
    robotMaintainPosition1 = mantainPosition;
    auto rotation = getRotationEndEffector(pandaIdentifier);
    rotateEndEffector(pandaIdentifier, -rotation);

    targetPose.position.x -= 0.01;
    targetPose.position.y -= 0.01;
    
  }

  if(mantainPosition == BottomLeft)
  {
    moveRightPosition();
    targetPose = workPose1;
    pandaIdentifier = 1;
    robotMaintainPosition1 = mantainPosition;
    targetPose.position.z += 0.01;
    targetPose.position.y -= 0.01;
  }

  if(mantainPosition == BehindRight)
  {
    moveLeftPosition();
    targetPose = workPose2;
    pandaIdentifier = 2;
    robotMaintainPosition2 = mantainPosition;

    auto rotation = getRotationEndEffector(pandaIdentifier);
    rotateEndEffector(pandaIdentifier, -rotation);

    targetPose.position.x += 0.01;
    targetPose.position.y += 0.01;
  }

  if(mantainPosition == TopRight)
  {
    moveLeftPosition();
    targetPose = workPose2;
    pandaIdentifier = 2;
    robotMaintainPosition2 = mantainPosition;
    targetPose.position.z -= 0.01;
    targetPose.position.y += 0.01;
  }

  moveToCartesianPath(pandaIdentifier, targetPose);
  closedGripperManually(pandaIdentifier, true);

}

bool maintainTopRightRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Maintain Top Right");
  maintainObjectPanda(TopRight);
  return true;
}

bool maintainBehindRightRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Maintain Behind Right");
  maintainObjectPanda(BehindRight);
  return true;
}

bool maintainBottomLeftRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Maintain Bottom Left");
  maintainObjectPanda(BottomLeft);
  return true;
}

bool maintainFrontLeftRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  ROS_INFO("Maintain Front Left");
  maintainObjectPanda(FrontLeft);
  return true;
}

void antiClockWiseRotation(int pandaIdentifier, double rotation, std::string objectName = rubikName)
{
  keepObject(pandaIdentifier,objectName);
  rotateEndEffector(pandaIdentifier, -rotation);
  leaveObjectAndRetreat(pandaIdentifier, true, rubikName); 
  rotateEndEffector(pandaIdentifier, rotation); 
}

void clockWiseRotation(int pandaIdentifier, double rotation, std::string objectName = rubikName)
{
  rotateEndEffector(pandaIdentifier, -rotation); 
  keepObject(pandaIdentifier,objectName);
  rotateEndEffector(pandaIdentifier, rotation);
  leaveObjectAndRetreat(pandaIdentifier, true, rubikName); 
}

void beginARotation(int pandaIdentifier, bool isClockWise, std::string objectName = rubikName)
{
  auto rotation = getRotationEndEffector(pandaIdentifier);
  auto isNegative = rotation < 0;

  if(isNegative && isClockWise)
  {
    antiClockWiseRotation(pandaIdentifier, rotation, objectName);
  }
  else if(!isNegative && isClockWise)
  {
    clockWiseRotation(pandaIdentifier, rotation, objectName);
  }
  else if(isNegative && !isClockWise)
  {
    clockWiseRotation(pandaIdentifier, rotation, objectName);
  }
  else if(isNegative && !isClockWise)
  {
    antiClockWiseRotation(pandaIdentifier, rotation, objectName);
  }

}

void prepareForRotationRequest(int pandaIdentifier, bool isClockwise, std::string objectName = rubikName)
{
  auto rotation = M_PI /2;
  auto canClockWise = canMoveClockWise(pandaIdentifier);
  if(isClockwise && !canClockWise)
  {
     rotateEndEffector(pandaIdentifier, -rotation); 
  }
  else if(isClockwise && canClockWise)
  {
    rotateEndEffector(pandaIdentifier, rotation);  
  }
  else if(!isClockwise && canClockWise)
  {
    rotateEndEffector(pandaIdentifier, rotation); 
  }
  else
  {
     rotateEndEffector(pandaIdentifier, -rotation); 
  }

  keepObject(pandaIdentifier,objectName);

}


void rotateRequest(int pandaIdentifier, bool isClockwise, bool isToAttach, std::string objectToAttach = rubikName)
{
  if(isToAttach)
  {
    auto groupName = "panda_" + std::to_string(pandaIdentifier) + "_effector";
    moveit::planning_interface::MoveGroupInterface group(groupName); 
    ROS_INFO("Attach object");
    group.attachObject(objectToAttach);
  }

  changeCheckContactWithObject(rubikName, true, pandaIdentifier);
  auto rotation = M_PI /2;
  if(isClockwise)
  {
     rotateEndEffector(pandaIdentifier, rotation); 
  }
  else
  {
     rotateEndEffector(pandaIdentifier, -rotation); 
  }

  changeCheckContactWithObject(rubikName, false, pandaIdentifier);
}


bool doRotationLeftRequest(manipulation_rubik::RotateCube::Request &req, manipulation_rubik::RotateCube::Response &res)
{
  ROS_INFO("Do Rotation Left");
  beginARotation(2, req.isClockWise);
  return true;
}

bool doRotationRightRequest(manipulation_rubik::RotateCube::Request &req, manipulation_rubik::RotateCube::Response &res)
{
  ROS_INFO("Do Rotation Right");
  beginARotation(1, req.isClockWise);
  return true;
}

bool prepareForRotationRightRequest(manipulation_rubik::PrepareForRotation::Request &req, manipulation_rubik::PrepareForRotation::Response &res)
{
  ROS_INFO("Prepare for rotation Right");
  prepareForRotationRequest(1, req.isClockWise);
  return true;
}

bool prepareForRotationLeftRequest(manipulation_rubik::PrepareForRotation::Request &req, manipulation_rubik::PrepareForRotation::Response &res)
{
  ROS_INFO("Prepare for rotation Left");
  prepareForRotationRequest(2, req.isClockWise);
  return true;
}

bool rotateRightRequest(manipulation_rubik::Rotate::Request &req, manipulation_rubik::Rotate::Response &res)
{
  ROS_INFO("Do Rotate Right");
  rotateRequest(1, req.isClockWise, req.isToAttach);
  return true;
}

bool rotateLeftRequest(manipulation_rubik::Rotate::Request &req, manipulation_rubik::Rotate::Response &res)
{
  ROS_INFO("Do Rotate Left");
  rotateRequest(2, req.isClockWise, req.isToAttach);
  return true;
}

bool leaveRightRequest(manipulation_rubik::LeaveObject::Request &req, manipulation_rubik::LeaveObject::Response &res)
{
  ROS_INFO("Leave Right");
  leaveObjectAndRetreat(1, req.isToDetach); 
  return true;
}

bool leaveLeftRequest(manipulation_rubik::LeaveObject::Request &req, manipulation_rubik::LeaveObject::Response &res)
{
  ROS_INFO("Leave Left");
  leaveObjectAndRetreat(2, req.isToDetach); 
  return true;
}


bool moveGripperRightRequest(manipulation_rubik::MoveGripper::Request &req, manipulation_rubik::MoveGripper::Response &res)
{
  
  if(req.isToOpen)
  {
    ROS_INFO("Open Gripper Left");
    openGripperManually(1,false);
  }
  else
  {
    ROS_INFO("Close Gripper Left");
    closedGripperManually(1,true);
  }
  return true;
}

bool moveGripperLeftRequest(manipulation_rubik::MoveGripper::Request &req, manipulation_rubik::MoveGripper::Response &res)
{
  
  if(req.isToOpen)
  {
    ROS_INFO("Open Gripper Left");
    openGripperManually(2,false);
  }
  else
  {
    ROS_INFO("Close Gripper Left");
    closedGripperManually(2,true);
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rubik_right_move");
  ros::NodeHandle nh;

  std::string nodeMode = "";
 
  if(argc > 1){
     nodeMode = argv[1];
  }
  
  if(nodeMode == "real"){
    ROS_INFO("Actual node is in REAL MODE");
    isSimulation = false;
  }
  else{
    
    ROS_INFO("Actual node is in SIMULATION MODE");
    isSimulation = true;
  }
  

  auto service1 = nh.advertiseService("move_left", moveLeftPositionRequest);
  auto service2 = nh.advertiseService("move_right", moveRightPositionRequest);
  auto service3 = nh.advertiseService("move_top", moveTopPositionRequest);
  auto service4 = nh.advertiseService("move_bottom", moveBottomPositionRequest);
  auto service5 = nh.advertiseService("move_front", moveFrontPositionRequest);
  auto service6 = nh.advertiseService("move_behind", moveBehindPositionRequest);

  auto service7 = nh.advertiseService("maintain_top_right", maintainTopRightRequest);
  auto service8 = nh.advertiseService("maintain_behind_right", maintainBehindRightRequest);
  auto service9 = nh.advertiseService("maintain_bottom_left", maintainBottomLeftRequest);
  auto service10 = nh.advertiseService("maintain_front_left", maintainFrontLeftRequest);

  auto service11 = nh.advertiseService("do_rotation_right", doRotationRightRequest);
  auto service12 = nh.advertiseService("do_rotation_left", doRotationLeftRequest);

  auto service13 = nh.advertiseService("leave_maintain_right", leaveMantainRightRequest);
  auto service14 = nh.advertiseService("leave_maintain_left", leaveMantainLeftRequest);

  auto service15 = nh.advertiseService("reset_environment", resetEnvironmentRequest);
  auto service16 = nh.advertiseService("start_position_right", startPositionRightRequest);
  auto service17 = nh.advertiseService("start_position_left", startPositionLeftRequest);

  auto service18 = nh.advertiseService("pick_right", pickRightRequest);
  auto service19 = nh.advertiseService("pick_left", pickLeftRequest);

  auto service20 = nh.advertiseService("place_right", placeRightRequest);
  auto service21 = nh.advertiseService("place_left", placeLeftRequest);

  auto service22 = nh.advertiseService("prepare_for_rotation_right", prepareForRotationRightRequest);
  auto service23 = nh.advertiseService("prepare_for_rotation_left", prepareForRotationLeftRequest);
  
  auto service24 = nh.advertiseService("rotate_right", rotateRightRequest);
  auto service25 = nh.advertiseService("rotate_left", rotateLeftRequest);

  auto service26 = nh.advertiseService("leave_right", leaveRightRequest);
  auto service27 = nh.advertiseService("leave_left", leaveLeftRequest);

  auto service29 = nh.advertiseService("move_gripper_right", moveGripperRightRequest);
  auto service28 = nh.advertiseService("move_gripper_left", moveGripperLeftRequest);


  ros::AsyncSpinner spinner(2);
  spinner.start();

  planning_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();

  
  planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
   sleep_t.sleep();
  }

  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::MoveGroupInterface panda1Group("panda_1_arm");
  moveit::planning_interface::MoveGroupInterface panda2Group("panda_2_arm");

  panda1Group.setPlanningTime(20.0);
  panda2Group.setPlanningTime(20.0);

  //checkLimits(1);
  //checkLimits(2);

  resetBehaviorAndPickFromStart();

  ros::waitForShutdown();
  return 0;
}

