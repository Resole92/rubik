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
#include <manipulation_rubik/RubikFace.h>
#include <manipulation_rubik/CubeDto.h>
#include <manipulation_rubik/RubikCubies.h>
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

class Cube
{
    public:
     string xColor;
     string yColor;
     string zColor;
};

class Face {
  public:
    vector<string> colors;   
    string name;
};

class Rubik
{
    public: 
    int dimension;
    vector<Face> faces;
    vector<Cube> cubies;

    void composeCubeFromFace(Face targetFace)
    {
        if(targetFace.name == "Top")
        {
            if(dimension == 2)
            {
                cubies[0].yColor = targetFace.colors[0];
                cubies[3].yColor = targetFace.colors[1];
                cubies[2].yColor = targetFace.colors[2];
                cubies[1].yColor = targetFace.colors[3];
            }
            
        }
        if(targetFace.name == "Bottom")
        {
            if(dimension == 2)
            {
                cubies[5].yColor = targetFace.colors[0];
                cubies[6].yColor = targetFace.colors[1];
                cubies[4].yColor = targetFace.colors[2];
                cubies[7].yColor = targetFace.colors[3];
            }
        }
        if(targetFace.name == "Right")
        {
            if(dimension == 2)
            {
                cubies[0].xColor = targetFace.colors[0];
                cubies[1].xColor = targetFace.colors[1];
                cubies[4].xColor = targetFace.colors[2];
                cubies[5].xColor = targetFace.colors[3];
            }
        }
        if(targetFace.name == "Left")
        {
            if(dimension == 2)
            {
                cubies[2].xColor = targetFace.colors[0];
                cubies[3].xColor = targetFace.colors[1];
                cubies[6].xColor = targetFace.colors[2];
                cubies[7].xColor = targetFace.colors[3];
            }
        }
        if(targetFace.name == "Front")
        {
            if(dimension == 2)
            {
                cubies[4].zColor = targetFace.colors[0];
                cubies[7].zColor = targetFace.colors[1];
                cubies[0].zColor = targetFace.colors[2];
                cubies[3].zColor = targetFace.colors[3];
            }
        }
        if(targetFace.name == "Behind")
        {
             if(dimension == 2)
            {
                cubies[1].zColor = targetFace.colors[0];
                cubies[2].zColor = targetFace.colors[1];
                cubies[5].zColor = targetFace.colors[2];
                cubies[6].zColor = targetFace.colors[3];
            }
        }
    }

    void initialize(int _dimension)
    {
        dimension = _dimension;
        faces.clear();
        cubies.clear();

        for(int i = 0; i < dimension*dimension*dimension; i++)
        {
            Cube cube;
            cubies.push_back(cube);
        }

        Face faceTop;
        faceTop.name = "Top";
        for(int i = 0; i < dimension*dimension; i++)
        {
            faceTop.colors.push_back("red");
        }
        composeCubeFromFace(faceTop);
        faces.push_back(faceTop);

        Face faceBottom;
        faceBottom.name = "Bottom";
        for(int i = 0; i < dimension*dimension; i++)
        {
            faceBottom.colors.push_back("orange");
        }
        composeCubeFromFace(faceBottom);
        faces.push_back(faceBottom);
        
        Face faceLeft;
        faceLeft.name = "Left";
        for(int i = 0; i < dimension*dimension; i++)
        {
            faceLeft.colors.push_back("white");
        }
        composeCubeFromFace(faceLeft);
        faces.push_back(faceLeft);

        Face faceRight;
        faceRight.name = "Right";
        for(int i = 0; i < dimension*dimension; i++)
        {
            faceRight.colors.push_back("yellow");
        }
        composeCubeFromFace(faceRight);
        faces.push_back(faceRight);
        
        Face faceFront;
        faceFront.name = "Front";
        for(int i = 0; i < dimension*dimension; i++)
        {
            faceFront.colors.push_back("green");
        }
        composeCubeFromFace(faceFront);
        faces.push_back(faceFront);

        Face faceBehind;
        faceBehind.name = "Behind";
        for(int i = 0; i < dimension*dimension; i++)
        {
            faceBehind.colors.push_back("blue");
        }
        composeCubeFromFace(faceBehind);
        faces.push_back(faceBehind);
        

    }

    string addFace(Face targetFace)
    {
        
        for(int i = 0; i < faces.size(); i++)
        {
            auto face = faces[i];
            if(face.name == targetFace.name)
            {
                faces[i].colors = targetFace.colors;
                cout << "Replace face " << faces[i].name << " with first color " << faces[i].colors[0] << "\n";
                composeCubeFromFace(targetFace);
                return "";
            }
        }

        string error = "Face " + targetFace.name + " is not present as face on rubik cube";     
        cout << "Fail! " << error << "\n";
        return error;
    }
};

Rubik RubikCube;

void example1()
{
    Face faceTop;
    faceTop.name = "Top";
    faceTop.colors.push_back("red");
    faceTop.colors.push_back("green");
    faceTop.colors.push_back("red");
    faceTop.colors.push_back("green");
    RubikCube.addFace(faceTop);

    Face faceBottom;
    faceBottom.name = "Bottom";
    faceBottom.colors.push_back("orange");
    faceBottom.colors.push_back("blue");
    faceBottom.colors.push_back("orange");
    faceBottom.colors.push_back("blue");
    RubikCube.addFace(faceBottom);

    Face faceLeft;
    faceLeft.name = "Left";
    faceLeft.colors.push_back("white");
    faceLeft.colors.push_back("white");
    faceLeft.colors.push_back("white");
    faceLeft.colors.push_back("white");
    RubikCube.addFace(faceLeft);

    Face faceRight;
    faceRight.name = "Right";
    faceRight.colors.push_back("yellow");
    faceRight.colors.push_back("yellow");
    faceRight.colors.push_back("yellow");
    faceRight.colors.push_back("yellow");
    RubikCube.addFace(faceRight);

    Face faceFront;
    faceFront.name = "Front";
    faceFront.colors.push_back("green");
    faceFront.colors.push_back("orange");
    faceFront.colors.push_back("green");
    faceFront.colors.push_back("orange");
    RubikCube.addFace(faceFront);

    Face faceBehind;
    faceBehind.name = "Behind";
    faceBehind.colors.push_back("red");
    faceBehind.colors.push_back("blue");
    faceBehind.colors.push_back("red");
    faceBehind.colors.push_back("blue");
    RubikCube.addFace(faceBehind);

}

void example2()
{
    Face faceTop;
    faceTop.name = "Top";
    faceTop.colors.push_back("white");
    faceTop.colors.push_back("green");
    faceTop.colors.push_back("blue");
    faceTop.colors.push_back("yellow");
    RubikCube.addFace(faceTop);

    Face faceBottom;
    faceBottom.name = "Bottom";
    faceBottom.colors.push_back("red");
    faceBottom.colors.push_back("blue");
    faceBottom.colors.push_back("white");
    faceBottom.colors.push_back("yellow");
    RubikCube.addFace(faceBottom);

    Face faceLeft;
    faceLeft.name = "Left";
    faceLeft.colors.push_back("orange");
    faceLeft.colors.push_back("red");
    faceLeft.colors.push_back("green");
    faceLeft.colors.push_back("yellow");
    RubikCube.addFace(faceLeft);

    Face faceRight;
    faceRight.name = "Right";
    faceRight.colors.push_back("orange");
    faceRight.colors.push_back("white");
    faceRight.colors.push_back("white");
    faceRight.colors.push_back("blue");
    RubikCube.addFace(faceRight);

    Face faceFront;
    faceFront.name = "Front";
    faceFront.colors.push_back("yellow");
    faceFront.colors.push_back("green");
    faceFront.colors.push_back("green");
    faceFront.colors.push_back("red");
    RubikCube.addFace(faceFront);

    Face faceBehind;
    faceBehind.name = "Behind";
    faceBehind.colors.push_back("orange");
    faceBehind.colors.push_back("blue");
    faceBehind.colors.push_back("orange");
    faceBehind.colors.push_back("red");
    RubikCube.addFace(faceBehind);

}

bool rubikCubiesRequest(manipulation_rubik::RubikCubies::Request &req, manipulation_rubik::RubikCubies::Response &res)
{
    auto numberOfCubies = RubikCube.cubies.size();
    for(int i = 0; i < numberOfCubies; i++)
    {
        manipulation_rubik::CubeDto cubeDto;
        cubeDto.Position = i;
        cubeDto.XColor = RubikCube.cubies[i].xColor;
        cubeDto.YColor = RubikCube.cubies[i].yColor;
        cubeDto.ZColor = RubikCube.cubies[i].zColor;
        res.Cubies.push_back(cubeDto);

    }
    return true;
}

bool rubikFaceRequest(manipulation_rubik::RubikFace::Request &req, manipulation_rubik::RubikFace::Response &res)
{
    Face face;
    face.name = req.face;

    auto totalRectangle = req.dimension * req.dimension;

    if(req.colors.size() != totalRectangle)
    {
        res.response = "Fail! Face requested dimension is " + std::to_string(req.colors.size()) + " but dimension pass is " + std::to_string(totalRectangle);
        return true;
    }

    for(int i = 0; i < totalRectangle; i++)
    {
        auto color = req.colors[i];
        face.colors.push_back(color);
    }

    if(req.isNewCube)
    {
        RubikCube.initialize(req.dimension);
    }
    else
    {
        if(RubikCube.dimension == 0)
        {
            res.response = "No cube initialize, please set new cube";
            return true;
        }

        if(req.dimension != RubikCube.dimension)
        {
            res.response = "Fail! Actual cube dimension is " + std::to_string(RubikCube.dimension) + " but dimension pass is " + std::to_string(req.dimension);
            return true;
        }
    }

    auto error = RubikCube.addFace(face);
    res.response = error;

    return true;
}

void createInputFile()
{
    auto numberOfCubies = RubikCube.cubies.size();
    ofstream myfile("src/manipulation_rubik/doc/logic/input.lp");

    if(myfile.is_open())
    {
        for(int i = 0; i < numberOfCubies; i++)
        {
            auto str = "is(0,"+ std::to_string(i) +","+ RubikCube.cubies[i].xColor + ","+  RubikCube.cubies[i].yColor + "," + RubikCube.cubies[i].zColor +").";   
            myfile << str << endl;
        }
        myfile.close();
    }
    else cerr<<"Unable to open file";

   
}

bool resolveConfigurationRequest(manipulation_rubik::ResolveConfiguration::Request &req, manipulation_rubik::ResolveConfiguration::Response &res)
{
    /*
    char solutionFileName[] = "solution.txt";

    //char commandClingo[] = "clingo src/manipulation_rubik/doc/logic/rubik.lp >> ";
    //string command = strcat(commandClingo, solutionFileName);
    //int n = command.length();
    //char char_array[n];
    //strcpy(char_array, command.c_str());
    //system(char_array);
    //remove(solutionFileName);
    //createInputFile();
    system("clingo src/manipulation_rubik/doc/logic/input.lp src/manipulation_rubik/doc/logic/rubik.lp >> solution.txt");
   
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

        //cout << "Match: " << moveTrunc + "\n";
        i++;
 
        // suffix to find the rest of the string.
        line = match.suffix().str();

        smatch tokenMatch;
        int j = 1;

        vector<string>  tokens;

        while (regex_search(moveTrunc, tokenMatch, tokenRegex)) {
          auto token = tokenMatch.str(0);
          //cout << "   Token: " << token + "\n";
          j++;

          tokens.push_back(token);
  
          // suffix to find the rest of the string.
          moveTrunc = tokenMatch.suffix().str();
        }

        manipulation_rubik::MoveConfiguration move;

        auto actualMove = tokens[0];
        auto angleMovement = tokens[1];
        auto face = tokens[2];
        auto clockWise =  tokens[3] == "1";

        res.numberOfMoves = stoi(actualMove);

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

    for(int i = 0; i < res.numberOfMoves; i++)
    {
      cout << "Move " << i + 1 << ": " << res.result[i]; 
    }
    */
    
    manipulation_rubik::MoveConfiguration move1;
    move1.IsClockWise = true;
    move1.Move = "Bottom";
    res.result.push_back(move1);

    manipulation_rubik::MoveConfiguration move2;
    move2.IsClockWise = false;
    move2.Move = "Front";
    res.result.push_back(move2);

    manipulation_rubik::MoveConfiguration move3;
    move3.IsClockWise = false;
    move3.Move = "Front";
    res.result.push_back(move3);

    manipulation_rubik::MoveConfiguration move4;
    move4.IsClockWise = false;
    move4.Move = "Behind";
    res.result.push_back(move4);

    manipulation_rubik::MoveConfiguration move5;
    move5.IsClockWise = true;
    move5.Move = "Right";
    res.result.push_back(move5);
    res.numberOfMoves = 5;

    return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "logic");
  ros::NodeHandle nh;

  auto service1 = nh.advertiseService("resolve_configuration", resolveConfigurationRequest);
  auto service2 = nh.advertiseService("rubik_face", rubikFaceRequest);
  auto service3 = nh.advertiseService("rubik_cubies", rubikCubiesRequest);
  
  ros::AsyncSpinner spinner(2);
  spinner.start();

  RubikCube.initialize(2);
  example1();

  ros::waitForShutdown();
  return 0;
}

