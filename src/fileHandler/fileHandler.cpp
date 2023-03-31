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

    // Loop through robots
    for(YAML::const_iterator robot_it=node["robots"].begin(); robot_it!=node["robots"].end(); ++robot_it){
        robot tempRobot;
        string robotName;
        vector<string> jointNames;
        int numActuators;
        bool torqueControlled;
        vector<double> torqueLimits;
        vector<double> startPos;
        vector<double> goalPos;
        vector<double> jointPosCosts;
        vector<double> jointVelCosts;
        vector<double> jointControlCosts;

        robotName = robot_it->first.as<string>();

        for(int i = 0; i < robot_it->second["jointNames"].size(); i++){
            jointNames.push_back(robot_it->second["jointNames"][i].as<std::string>());
        }

        numActuators = robot_it->second["numActuators"].as<int>();
        torqueControlled = robot_it->second["torqueControl"].as<bool>();

        for(int i = 0; i < robot_it->second["torqueLimits"].size(); i++){
            torqueLimits.push_back(robot_it->second["torqueLimits"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["startPos"].size(); i++){
            startPos.push_back(robot_it->second["startPos"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["goalPos"].size(); i++){
            goalPos.push_back(robot_it->second["goalPos"][i].as<double>());
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

        tempRobot.name = robotName;
        tempRobot.jointNames = jointNames;
        tempRobot.numActuators = numActuators;
        tempRobot.torqueControlled = torqueControlled;
        tempRobot.torqueLimits = torqueLimits;
        tempRobot.startPos = startPos;
        tempRobot.goalPos = goalPos;
        tempRobot.jointPosCosts = jointPosCosts;
        tempRobot.jointVelCosts = jointVelCosts;
        tempRobot.jointControlCosts = jointControlCosts;

        _robots.push_back(tempRobot);
    }

    // Loop through bodies
    for(YAML::const_iterator body_it=node["bodies"].begin(); body_it!=node["bodies"].end(); ++body_it) {
        bodyStateVec tempBody;
        std::string bodyName;
        bool activeLinearDOF[3];
        bool activeAngularDOF[3];
        double startLinearPos[3];
        double startAngularPos[3];
        double goalLinearPos[3];
        double goalAngularPos[3];
        double linearPosCosts[3];
        double linearVelCosts[3];
        double angularPosCosts[3];
        double angularVelCosts[3];

        bodyName = body_it->first.as<string>();

        for(int i = 0; i < body_it->second["activeLinearDOF"].size(); i++){
            activeLinearDOF[i] = body_it->second["activeLinearDOF"][i].as<bool>();
        }

        for(int i = 0; i < body_it->second["activeAngularDOF"].size(); i++){
            activeAngularDOF[i] = body_it->second["activeAngularDOF"][i].as<bool>();
        }

        for(int i = 0; i < body_it->second["startLinearPos"].size(); i++){
            startLinearPos[i] = body_it->second["startLinearPos"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["startAngularPos"].size(); i++){
            startAngularPos[i] = body_it->second["startAngularPos"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["goalLinearPos"].size(); i++){
            goalLinearPos[i] = body_it->second["goalLinearPos"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["goalAngularPos"].size(); i++){
            goalAngularPos[i] = body_it->second["goalAngularPos"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["linearPosCost"].size(); i++){
            linearPosCosts[i] = body_it->second["linearPosCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["linearVelCost"].size(); i++){
            linearVelCosts[i] = body_it->second["linearVelCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["angularPosCost"].size(); i++){
            angularPosCosts[i] = body_it->second["angularPosCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["angularVelCost"].size(); i++){
            angularVelCosts[i] = body_it->second["angularVelCost"][i].as<double>();
        }

        tempBody.name = bodyName;
        for(int i = 0; i < 3; i++){
            tempBody.activeLinearDOF[i] = activeLinearDOF[i];
            tempBody.activeAngularDOF[i] = activeAngularDOF[i];
            tempBody.startLinearPos[i] = startLinearPos[i];
            tempBody.startAngularPos[i] = startAngularPos[i];
            tempBody.goalLinearPos[i] = goalLinearPos[i];
            tempBody.goalAngularPos[i] = goalAngularPos[i];
            tempBody.linearPosCost[i] = linearPosCosts[i];
            tempBody.linearVelCost[i] = linearVelCosts[i];
            tempBody.angularPosCost[i] = angularPosCosts[i];
            tempBody.angularVelCost[i] = angularVelCosts[i];
        }

        _bodies.push_back(tempBody);
    }
}

void fileHandler::readSettingsFile(std::string settingsFilePath){

    YAML::Node node = YAML::LoadFile(projectParentPath + settingsFilePath);

    optimiser = node["optimiser"].as<std::string>();
    project_display_mode = node["displayMode"].as<int>();
    taskNumber = node["taskNumber"].as<int>();
}

void fileHandler::readOptimisationSettingsFile(int optimiser) {
    std::string optimisationSettingsFilePath;
    YAML::Node node;

    if (optimiser == opt_iLQR) {
        optimisationSettingsFilePath = "/optimiserConfigs/iLQR.yaml";
    }
    else if(optimiser == opt_stomp){
        optimisationSettingsFilePath = "/optimiserConfigs/stomp.yaml";
    }
    else if(optimiser == opt_gradDescent){
        std::cout << "grad descent not yet implemented \n";
    }
    else{
        std::cout << "invalid optimiser selected!!!";
    }

    node = YAML::LoadFile(projectParentPath + optimisationSettingsFilePath);
    minIter = node["minIter"].as<int>();
    maxIter = node["maxIter"].as<int>();
    maxHorizon = node["maxHorizon"].as<int>();

    if(optimiser == opt_iLQR){
        setIntervalMethod = node["setInterval"].as<bool>();
        intervalSize = node["intervalSize"].as<int>();
    }
}