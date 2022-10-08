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
#include <manipulation_rubik/RubikFaceDetect.h>
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


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

string captureFace(string faceName)
{
  cv::VideoCapture capture(0);
  if(!capture.isOpened()){
    cout << "could not read file" << endl;
    return "";
  }  

  cv::Mat mat;
 
  auto frameToRead = 30;
  auto actualFrameRead = 0;
  while(actualFrameRead < frameToRead)
  {
    capture >> mat;
    cout << "capture frame " << actualFrameRead << endl;
    if(mat.empty())
    {
      cerr << "Something is wrong with the webcam, could not get frame." << endl;
    }
    actualFrameRead++;
   
  }

  
  string processPath = "src/manipulation_rubik/doc/eyes/photos/"+ faceName +".jpg";
  cv::imwrite(processPath, mat);
  cv::waitKey(25);

  string scriptPath = "src/manipulation_rubik/doc/eyes/photoProcess.py";
  string processFolder = "src/manipulation_rubik/doc/eyes/photos_processed";
  system(("python " + scriptPath + " " + processPath + " " + processFolder + " " + faceName ).c_str());

  string resultPath = processFolder + "/" + faceName + ".txt";
  return resultPath;

}


bool detectFaceRequest(manipulation_rubik::RubikFaceDetect::Request &req, manipulation_rubik::RubikFaceDetect::Response &res)
{
  string resultPath =  captureFace(req.face);
  string line;
  string str;
  ifstream myfile (resultPath);
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
      str = line;
    }
  }

  stringstream ss( str );
  vector<string> result;

  while( ss.good() )
  {
    string substr;
    getline( ss, substr, ',' );
    res.colors.push_back(substr);
  }
  
  return true;
}


bool capturePhotoRequest(manipulation_rubik::LfMoveLeft::Request &req, manipulation_rubik::LfMoveLeft::Response &res)
{
  captureFace("test"); 
  return true;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eyes");
  ros::NodeHandle nh;
 
  auto service1 = nh.advertiseService("detect_face", detectFaceRequest);
  auto service2 = nh.advertiseService("capture_photo", capturePhotoRequest);
 

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::waitForShutdown();
  return 0;
}