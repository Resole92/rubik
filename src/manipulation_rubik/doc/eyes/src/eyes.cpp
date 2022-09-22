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


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eyes");
  ros::NodeHandle nh;
  cv::Mat image;
  cv::VideoCapture capture(1);//les video
  if(!capture.isOpened()){
    cout << "could not read file" << endl;
    return -1;
  }   

  //capture.set(cv::CAP_PROP_FRAME_WIDTH , 1600);
  //capture.set(cv::CAP_PROP_FRAME_HEIGHT, 1200);
  cv::Mat mat;

  // wait for some external event here so I know it is time to take a picture...
 for(;;)
  {
      capture >> mat;
      cout << "capture frame" << endl;
      //sleep(10);
      
    cv::imshow("Display window", mat);
    cv::waitKey(2000);

  }

}