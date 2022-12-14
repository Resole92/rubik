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
#include <manipulation_rubik/RotateCube.h>
#include <manipulation_rubik/Rotate.h>
#include <manipulation_rubik/PrepareForRotation.h>
#include <manipulation_rubik/LeaveObject.h>
#include <manipulation_rubik/RubikFaceDetect.h>
#include <manipulation_rubik/RubikFace.h>
#include <manipulation_rubik/StartProcess.h>
#include <manipulation_rubik/ConstructRubikEyes.h>
#include <manipulation_rubik/RequestEyesFace.h>
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

enum objectMainteined { None, TopRight, BehindRight, BottomLeft, FrontLeft};
enum objectMovement { NoMovement, Left, Top, Right, Bottom, Front, Behind};

objectMainteined MainteinedStatus = None;
objectMovement LastMovement = NoMovement; 

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

ros::ServiceClient clientPlaceRight;
ros::ServiceClient clientPlaceLeft;

ros::ServiceClient clientResolveConfiguration;

ros::ServiceClient clientPrepareForRotationRight;
ros::ServiceClient clientPrepareForRotationLeft;

ros::ServiceClient clientRotateRight;
ros::ServiceClient clientRotateLeft;

ros::ServiceClient clientLeaveRight;
ros::ServiceClient clientLeaveLeft;

ros::ServiceClient clientDetectFace;
ros::ServiceClient clientRubikFace;
ros::ServiceClient clientRetrieveFace;

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

void doRotationRight(bool isClockWise)
{
  manipulation_rubik::RotateCube srv;
  srv.request.isClockWise = isClockWise;
  clientDoRotationRight.call(srv);
}

void doRotationLeft(bool isClockWise)
{
  manipulation_rubik::RotateCube srv;
  srv.request.isClockWise = isClockWise;
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

void placeRight()
{
  manipulation_rubik::LfMoveLeft srv;
  clientPlaceRight.call(srv);
}

void placeLeft()
{
  manipulation_rubik::LfMoveLeft srv;
  clientPlaceLeft.call(srv);
}

void startPositionRight()
{
  manipulation_rubik::LfMoveLeft srv;
  clientStartPositionRight.call(srv);
}

void startPositionLeft()
{
  manipulation_rubik::LfMoveLeft srv;
  clientStartPositionLeft.call(srv);
}

void prepareForRotationRight(bool isClockWise)
{
  manipulation_rubik::PrepareForRotation srv;
  srv.request.isClockWise = isClockWise;
  clientPrepareForRotationRight.call(srv);
}

void prepareForRotationLeft(bool isClockWise)
{
  manipulation_rubik::PrepareForRotation srv;
  srv.request.isClockWise = isClockWise;
  clientPrepareForRotationLeft.call(srv);
}

void rotateRight(bool isClockWise, bool isToAttach)
{
  manipulation_rubik::Rotate srv;
  srv.request.isClockWise = isClockWise;
  srv.request.isToAttach = isToAttach;
  clientRotateRight.call(srv);
}

void rotateLeft(bool isClockWise, bool isToAttach)
{
  manipulation_rubik::Rotate srv;
  srv.request.isClockWise = isClockWise;
  srv.request.isToAttach = isToAttach;
  clientRotateLeft.call(srv);
}

void leaveRight()
{
  manipulation_rubik::LeaveObject srv;
  srv.request.isToDetach = true;
  clientLeaveRight.call(srv);
}

void leaveLeft()
{
  manipulation_rubik::LeaveObject srv;
  srv.request.isToDetach = true;
  clientLeaveLeft.call(srv);
}

std::vector<std::string> faceDetect(std::string face)
{
  manipulation_rubik::RubikFaceDetect srv;
  srv.request.face = face;
  clientDetectFace.call(srv);
  std::string result = "";
  for(int i=0; i < srv.response.colors.size(); i++)
  {
    result = result + srv.response.colors[i] + ",";
  }

  std::cout << result << std::endl;
  return srv.response.colors;
}

void sendFaceData(std::vector<std::string> colors, std::string face, bool isNewCube)
{
  manipulation_rubik::RubikFace srv;
  srv.request.dimension = 2;
  srv.request.isNewCube = isNewCube;
  srv.request.colors = colors;
  srv.request.face = face;
  clientRubikFace.call(srv);
  std::cout << srv.response.response << std::endl;
}

std::vector<std::string> retrieveFace(std::string face)
{
  manipulation_rubik::RequestEyesFace srv;
  srv.request.face = face;
  clientRetrieveFace.call(srv);
  return srv.response.colors;
}

void maintainFromTopRightToBehindRight()
{
    maintainBottomLeft();
    leaveMaintainLeft();
    maintainBehindRight();
    leaveMaintainRight();
}

void maintainFromBehindRightToTopRight()
{
    maintainBottomLeft();
    leaveMaintainLeft();
    maintainTopRight();
    leaveMaintainRight();
}

void maintainFromFrontLeftToBottomLeft()
{
    maintainTopRight();
    leaveMaintainRight();
    maintainBottomLeft();
    leaveMaintainLeft();
}

void maintainFromBottomLeftToFrontLeft()
{
    maintainTopRight();
    leaveMaintainRight();
    maintainFrontLeft();
    leaveMaintainLeft();
}

void rotateTopFace(bool isClockWise)
{

  if(LastMovement != Top)
  {
    if(MainteinedStatus == BehindRight)
    {
       maintainFromBehindRightToTopRight();
    }
    else
    {
        maintainTopRight();
    }
    MainteinedStatus = TopRight;

    leaveMaintainRight();
    moveTopPosition();
  }

  doRotationRight(isClockWise);
  LastMovement = Top;
}

void rotateRightFace(bool isClockWise)
{
  if(LastMovement != Right)
  {
    if(MainteinedStatus == BehindRight)
    {
      maintainFromBehindRightToTopRight();
    }
    else
    {
      maintainTopRight();
    }
   
    MainteinedStatus = TopRight;

    leaveMaintainRight();
    moveRightPosition();
  }

  doRotationRight(isClockWise);
  LastMovement = Right;
}

void rotateBehindFace(bool isClockWise)
{
  if(LastMovement != Behind)
  {
    if(MainteinedStatus == TopRight)
    {
       maintainFromTopRightToBehindRight();
    }
    else
    {
      maintainBehindRight();
    }

    MainteinedStatus = BehindRight;

    leaveMaintainRight();
    moveBehindPosition();
  }

  doRotationRight(isClockWise);
  LastMovement = Behind;
}

void rotateBottomFace(bool isClockWise)
{
  if(LastMovement != Bottom)
  {
    if(MainteinedStatus == FrontLeft)
    {
      maintainFromFrontLeftToBottomLeft();
    }
    else
    {
      maintainBottomLeft(); 
    }

    MainteinedStatus = BottomLeft;

    leaveMaintainLeft();
    moveBottomPosition();
  }

  doRotationLeft(isClockWise);
  LastMovement = Bottom;
}

void rotateLeftFace(bool isClockWise)
{
  if(LastMovement != Left)
  {
    if(MainteinedStatus == FrontLeft)
    {
      maintainFromFrontLeftToBottomLeft();
    }
    else
    {
      maintainBottomLeft();
    }

    MainteinedStatus = BottomLeft;
    leaveMaintainLeft();
    moveLeftPosition();
    }

  doRotationLeft(isClockWise);
  LastMovement = Left;
}

void rotateFrontFace(bool isClockWise)
{
  if(LastMovement != Front)
  {
    if(MainteinedStatus == BottomLeft)
    {
      maintainFromBottomLeftToFrontLeft();
    }
    else
    {
      maintainFrontLeft();
    }

    MainteinedStatus = FrontLeft;

    leaveMaintainLeft();
    moveFrontPosition();
  }

  doRotationLeft(isClockWise);
  LastMovement = Front;
}


bool constructRubikFromEyesData()
{
  std::cout << "Construct rubik data" << std::endl;

  std::vector<std::string> faces;
  faces.push_back("Top");
  faces.push_back("Bottom");
  faces.push_back("Right");
  faces.push_back("Left");
  faces.push_back("Front");
  faces.push_back("Behind");

  bool isNewCube = true;
  for(int i = 0; i < faces.size(); i++)
  {
    auto face = faces[i];
    auto colors = retrieveFace(face);
    sendFaceData(colors, face, isNewCube);

    if(isNewCube)
    {
      isNewCube = false;
    }
  }
}

void retrieveFaces()
{
  
  std::string actualFace = "Behind";
  prepareForRotationLeft(true);
  leaveRight();
  faceDetect(actualFace);
  ROS_INFO("Detect behind face");
  rotateLeft(false, true);

  actualFace = "Top";
  prepareForRotationRight(false);
  leaveLeft();
  faceDetect(actualFace);
  ROS_INFO("Detect top face");
  rotateRight(true, true);

  actualFace = "Front";
  prepareForRotationLeft(true);
  leaveRight();
  faceDetect(actualFace);
  ROS_INFO("Detect front face");
  rotateLeft(false, true);

  actualFace = "Bottom";
  prepareForRotationRight(false);
  leaveLeft();
  faceDetect(actualFace);
  ROS_INFO("Detect bottom face");
  rotateRight(true, true);

  moveBottomPosition();

  actualFace = "Left";
  prepareForRotationLeft(true);
  leaveRight();
  rotateLeft(false, true);
  faceDetect(actualFace);
  ROS_INFO("Detect left face");
  prepareForRotationRight(false);
  leaveLeft();

  prepareForRotationLeft(true);
  leaveRight();
  rotateLeft(false, true);
  ROS_INFO("Already detected");
  prepareForRotationRight(false);
  leaveLeft();

  actualFace = "Right";
  prepareForRotationLeft(true);
  leaveRight();
  rotateLeft(false, true);
  faceDetect(actualFace);
  ROS_INFO("Detect right face");
  prepareForRotationRight(false);
  leaveLeft();

  prepareForRotationLeft(true);
  leaveRight();
  rotateLeft(false, true);
  ROS_INFO("Already detected");
  prepareForRotationRight(false);
  leaveLeft();

  constructRubikFromEyesData();

  moveLeftPosition();

}


bool startProcessRequest(manipulation_rubik::StartProcess::Request &req, manipulation_rubik::StartProcess::Response &res)
{
  pickRight();
  moveLeftPosition();
  if(req.doFaceDetecting)
  {
    retrieveFaces();
  }

  if(!req.doMovement) return true;
  
  maintainTopRight();
  MainteinedStatus = TopRight;

  manipulation_rubik::ResolveConfiguration srv;
  srv.request.issimulation = req.doSimulationLogic;
  clientResolveConfiguration.call(srv);
  auto moves = srv.response.result;
  auto numberOfMoves = srv.response.numberOfMoves;

  for(int i = 0; i < numberOfMoves; i++)
  {
    if(moves[i].Move == "Top")
    {
        rotateTopFace(moves[i].IsClockWise); 
    }
    if(moves[i].Move == "Bottom")
    {
        rotateBottomFace(moves[i].IsClockWise); 
    }
    if(moves[i].Move == "Right")
    {
        rotateRightFace(moves[i].IsClockWise); 
    }
    if(moves[i].Move == "Left")
    {
        rotateLeftFace(moves[i].IsClockWise); 
    }
    if(moves[i].Move == "Behind")
    {
        rotateBehindFace(moves[i].IsClockWise); 
    }
    if(moves[i].Move == "Front")
    {
        rotateFrontFace(moves[i].IsClockWise); 
    }
  }
  
  if(MainteinedStatus == TopRight || MainteinedStatus == BehindRight)
  {
    startPositionRight();
    placeLeft();
    startPositionLeft();
  }
  else
  {
    startPositionLeft();
    placeRight();
    startPositionRight();
  }
  return true;
}


bool constructRubikEyesRequest(manipulation_rubik::ConstructRubikEyes::Request &req, manipulation_rubik::ConstructRubikEyes::Response &res)
{
  constructRubikFromEyesData();
  return true;
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

  clientDoRotationRight = nh.serviceClient<manipulation_rubik::RotateCube>("do_rotation_right");
  clientDoRotationLeft = nh.serviceClient<manipulation_rubik::RotateCube>("do_rotation_left");

  clientLeaveMaintainRight = nh.serviceClient<manipulation_rubik::LfMoveLeft>("leave_maintain_right");
  clientLeaveMaintainLeft = nh.serviceClient<manipulation_rubik::LfMoveLeft>("leave_maintain_left");

  clientResetEnvironment = nh.serviceClient<manipulation_rubik::LfMoveLeft>("reset_environment");
  clientStartPositionRight = nh.serviceClient<manipulation_rubik::LfMoveLeft>("start_position_right");
  clientStartPositionLeft = nh.serviceClient<manipulation_rubik::LfMoveLeft>("start_position_left");

  clientPickRight = nh.serviceClient<manipulation_rubik::LfMoveLeft>("pick_right");
  clientPickLeft = nh.serviceClient<manipulation_rubik::LfMoveLeft>("pick_left");

  clientPlaceRight = nh.serviceClient<manipulation_rubik::LfMoveLeft>("place_right");
  clientPlaceLeft = nh.serviceClient<manipulation_rubik::LfMoveLeft>("place_left");

  clientResolveConfiguration = nh.serviceClient<manipulation_rubik::ResolveConfiguration>("resolve_configuration");

  clientPrepareForRotationRight = nh.serviceClient<manipulation_rubik::PrepareForRotation>("prepare_for_rotation_right");
  clientPrepareForRotationLeft = nh.serviceClient<manipulation_rubik::PrepareForRotation>("prepare_for_rotation_left");

  clientRotateRight = nh.serviceClient<manipulation_rubik::Rotate>("rotate_right");
  clientRotateLeft = nh.serviceClient<manipulation_rubik::Rotate>("rotate_left");

  clientLeaveRight = nh.serviceClient<manipulation_rubik::LeaveObject>("leave_right");
  clientLeaveLeft = nh.serviceClient<manipulation_rubik::LeaveObject>("leave_left");

  clientDetectFace = nh.serviceClient<manipulation_rubik::RubikFaceDetect>("detect_face");
  clientRubikFace = nh.serviceClient<manipulation_rubik::RubikFace>("rubik_face");
  clientRetrieveFace =  nh.serviceClient<manipulation_rubik::RequestEyesFace>("retrieve_face");


  auto service1 = nh.advertiseService("start_process", startProcessRequest);
  auto service2 = nh.advertiseService("construct_rubik_eyes", constructRubikEyesRequest);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::waitForShutdown();
  return 0;
}