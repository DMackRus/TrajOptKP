//
// Created by dave on 3/30/23.
//

#include "fileHandler.h"

fileHandler::fileHandler(){
    // Init Controls
    project_display_mode = 0;
    // Pendulum
    taskNumber = 0;
    // interpolated iLQR
    optimiser = "interpolated_iLQR";

    // Get project parent path
    projectParentPath = __FILE__;
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));

}

void fileHandler::readModelConfigFile(std::string yamlFilePath, vector<robot> &_robots, vector<bodyStateVec> &_bodies, std::string &modelFilePath){
    YAML::Node node = YAML::LoadFile(projectParentPath + yamlFilePath);

    int counter = 0;

    modelFilePath = projectParentPath + node["modelFile"].as<std::string>();

    for(YAML::const_iterator it=node.begin(); it!=node.end(); ++it) {
        // Robots
        if(counter == 1){
            // Loop through robots list
            robot tempRobot;
            string robotName;
            vector<string> jointNames;
            int numActuators;
            bool torqueControlled;
            vector<double> torqueLimits;
            vector<double> jointPosCosts;
            vector<double> jointVelCosts;
            vector<double> jointControlCosts;

            for(YAML::const_iterator robot_it=it->second.begin(); robot_it!=it->second.end(); ++robot_it){

                robotName = robot_it->first.as<string>();

                for(int i = 0; i < robot_it->second["jointNames"].size(); i++){
                    jointNames.push_back(robot_it->second["jointNames"][i].as<std::string>());
                }

                numActuators = robot_it->second["numActuators"].as<int>();
                torqueControlled = robot_it->second["torqueControl"].as<bool>();

                for(int i = 0; i < robot_it->second["torqueLimits"].size(); i++){
                    torqueLimits.push_back(robot_it->second["torqueLimits"][i].as<double>());
                }

                for(int i = 0; i < robot_it->second["jointPosCosts"].size(); i++){
                    jointPosCosts.push_back(robot_it->second["jointPosCosts"][i].as<double>());
                }

                for(int i = 0; i < robot_it->second["jointVelCosts"].size(); i++){
                    jointVelCosts.push_back(robot_it->second["jointVelCosts"][i].as<double>());
                }

                for(int i = 0; i < robot_it->second["jointControlCosts"].size(); i++){
                    jointControlCosts.push_back(robot_it->second["jointControlCosts"][i].as<double>());
                }
            }

            tempRobot.name = robotName;
            tempRobot.jointNames = jointNames;
            tempRobot.numActuators = numActuators;
            tempRobot.torqueControlled = torqueControlled;
            tempRobot.torqueLimits = torqueLimits;
            tempRobot.jointPosCosts = jointPosCosts;
            tempRobot.jointVelCosts = jointVelCosts;
            tempRobot.jointControlCosts = jointControlCosts;

            _robots.push_back(tempRobot);
        }
            // Loop through bodies
        else if(counter == 2){
            bodyStateVec tempBody;
            std::string bodyName;
            bool activeLinearDOF[3];
            bool activeAngularDOF[3];
            double linearPosCosts[3];
            double linearVelCosts[3];
            double angularPosCosts[3];
            double angularVelCosts[3];
            // Loop through bodies list
            for(YAML::const_iterator robot_it=it->second.begin(); robot_it!=it->second.end(); ++robot_it){

                bodyName = robot_it->first.as<string>();

                for(int i = 0; i < robot_it->second["activeLinearDOF"].size(); i++){
                    activeLinearDOF[i] = robot_it->second["activeLinearDOF"][i].as<bool>();
                }

                for(int i = 0; i < robot_it->second["activeAngularDOF"].size(); i++){
                    activeAngularDOF[i] = robot_it->second["activeAngularDOF"][i].as<bool>();
                }

                for(int i = 0; i < robot_it->second["linearPosCost"].size(); i++){
                    linearPosCosts[i] = robot_it->second["linearPosCost"][i].as<double>();
                }

                for(int i = 0; i < robot_it->second["linearVelCost"].size(); i++){
                    linearVelCosts[i] = robot_it->second["linearVelCost"][i].as<double>();
                }

                for(int i = 0; i < robot_it->second["angularPosCost"].size(); i++){
                    angularPosCosts[i] = robot_it->second["angularPosCost"][i].as<double>();
                }

                for(int i = 0; i < robot_it->second["angularVelCost"].size(); i++){
                    angularVelCosts[i] = robot_it->second["angularVelCost"][i].as<double>();
                }

                tempBody.name = bodyName;
                for(int i = 0; i < 3; i++){
                    tempBody.activeLinearDOF[i] = activeLinearDOF[i];
                    tempBody.activeAngularDOF[i] = activeAngularDOF[i];
                    tempBody.linearPosCost[i] = linearPosCosts[i];
                    tempBody.linearVelCost[i] = linearVelCosts[i];
                    tempBody.angularPosCost[i] = angularPosCosts[i];
                    tempBody.angularVelCost[i] = angularVelCosts[i];
                }

                _bodies.push_back(tempBody);
            }
        }
        counter ++;
    }
}

void fileHandler::readSettingsFile(std::string settingsFilePath){

    YAML::Node node = YAML::LoadFile(projectParentPath + settingsFilePath);

    optimiser = node["optimiser"].as<std::string>();
    project_display_mode = node["displayMode"].as<int>();
    taskNumber = node["taskNumber"].as<int>();
}