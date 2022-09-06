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

#include <iostream>
#include <fstream>
#include <regex>
#include <stdio.h>
#include <vector>

using namespace std;

bool resolveConfigurationRequest(manipulation_rubik::ResolveConfiguration::Request &req, manipulation_rubik::ResolveConfiguration::Response &res)
{

    
    char solutionFileName[] = "solution.txt";
    //char commandClingo[] = "clingo src/manipulation_rubik/doc/logic/rubik.lp >> ";

    //string command = strcat(commandClingo, solutionFileName);

    //int n = command.length();
    //char char_array[n + 1];
    //strcpy(char_array, command.c_str());

    //cout << char_array;

    //system(char_array);

   
    // regex expression for pattern to be searched 
    regex moveRegex ("move\\(\\d+,\\w+,\\d,-?\\d\\)"); 
    regex tokenRegex ("\\w+");

    string line;
    ifstream myfile (solutionFileName);
    if (myfile.is_open())
    {
      while ( getline (myfile,line) )
      {

      // Template instantiations for
      // extracting the matching pattern.
      smatch match;
      int i = 1;
      while (regex_search(line, match, moveRegex)) {
        auto complexMove = match.str(0);
        string moveTrunc = complexMove.substr(5, complexMove.length() - 6);

        //ROS_INFO(moveTrunc);
        cout << "Match: " << moveTrunc + "\n";
        i++;
 
        // suffix to find the rest of the string.
        line = match.suffix().str();

        smatch tokenMatch;
        int j = 1;

        vector<string>  tokens;

        while (regex_search(moveTrunc, tokenMatch, tokenRegex)) {
          auto token = tokenMatch.str(0);
          cout << "   Token: " << token + "\n";
          j++;

          tokens.push_back(token);
  
          // suffix to find the rest of the string.
          moveTrunc = tokenMatch.suffix().str();
        }

        manipulation_rubik::MoveConfiguration move;

        auto angleMovement = tokens[1];
        auto face = tokens[2];
        auto clockWise =  tokens[3] == "1";

        move.IsClockWise = clockWise;

        if(angleMovement == "yaw")
        {
            if(face == "0")
            {
                move.Move = "Top";
            }
            else
            {
                move.Move = "Bottom";
            }
        }

        if(angleMovement == "pitch")
        {
            if(face == "0")
            {
                move.Move = "Right";
            }
            else
            {
                move.Move = "Left";
            }
        }

        if(angleMovement == "roll")
        {
            if(face == "0")
            {
                move.Move = "Front";
            }
            else
            {
                move.Move = "Behind";
            }
        }

        res.result.push_back(move);

      }


       //cout << line << '\n';
      }
      myfile.close();
    }
    else cout << "Unable to open file"; 

    //remove(solutionFileName);



    manipulation_rubik::MoveConfiguration move1;
    move1.IsClockWise = true;
    move1.Move = "Bottom";
    //res.result.push_back(move1);

    manipulation_rubik::MoveConfiguration move2;
    move2.IsClockWise = false;
    move2.Move = "Front";
    //res.result.push_back(move2);

    manipulation_rubik::MoveConfiguration move3;
    move3.IsClockWise = false;
    move3.Move = "Front";
    //res.result.push_back(move3);

    manipulation_rubik::MoveConfiguration move4;
    move4.IsClockWise = false;
    move4.Move = "Behind";
    //res.result.push_back(move4);

    manipulation_rubik::MoveConfiguration move5;
    move5.IsClockWise = true;
    move5.Move = "Right";
    //res.result.push_back(move5);

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