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


void fileHandler::readModelConfigFile(std::string yamlFilePath, vector<robot> &_robots, vector<bodyStateVec> &_bodies, std::string &modelFilePath, std::string &_modelName){
    YAML::Node node = YAML::LoadFile(projectParentPath + yamlFilePath);

    modelFilePath = projectParentPath + node["modelFile"].as<std::string>();
    _modelName = node["modelName"].as<std::string>();

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
        vector<double> jointJerkThresholds;

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

        for(int i = 0; i < robot_it->second["jointJerkThresholds"].size(); i++){
            jointJerkThresholds.push_back(robot_it->second["jointJerkThresholds"][i].as<double>());
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
        tempRobot.jointJerkThresholds = jointJerkThresholds;

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
        double linearJerkThreshold[3];
        double angularJerkThreshold[3];

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

        for(int i = 0; i < body_it->second["linearJerkThreshold"].size(); i++){
            linearJerkThreshold[i] = body_it->second["linearJerkThreshold"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["angularJerkThreshold"].size(); i++){
            angularJerkThreshold[i] = body_it->second["angularJerkThreshold"][i].as<double>();
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
            tempBody.linearJerkThreshold[i] = linearJerkThreshold[i];
            tempBody.angularJerkThreshold[i] = angularJerkThreshold[i];
        }

        _bodies.push_back(tempBody);
    }
}

void fileHandler::readSettingsFile(std::string settingsFilePath){

    YAML::Node node = YAML::LoadFile(projectParentPath + settingsFilePath);

    optimiser = node["optimiser"].as<std::string>();
    project_display_mode = node["displayMode"].as<int>();
    taskNumber = node["taskNumber"].as<int>();
    taskInitMode = node["taskInitMode"].as<std::string>();
    csvRow = node["csvRow"].as<int>();
    filtering = node["filtering"].as<bool>();
    costDerivsFD = node["costDerivsFD"].as<bool>();

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
        optimisationSettingsFilePath = "/optimiserConfigs/gradDescent.yaml";
    }
    else{
        std::cout << "invalid optimiser selected!!!";
    }

    node = YAML::LoadFile(projectParentPath + optimisationSettingsFilePath);
    minIter = node["minIter"].as<int>();
    maxIter = node["maxIter"].as<int>();
    maxHorizon = node["maxHorizon"].as<int>();

    if(optimiser == opt_iLQR || optimiser == opt_gradDescent){
        keyPointMethod = node["keyPointMethod"].as<int>();
        minInterval = node["min_interval"].as<int>();
        maxInterval = node["max_interval"].as<int>();

    }

    if(optimiser == opt_iLQR){
        approximate_backwardsPass = node["approximate_backwardsPass"].as<bool>();
    }
}

void fileHandler::generalSaveMatrices(std::vector<MatrixXd> matrices, std::string fileName){
    int size = matrices.size();
    cout << "trajectory size: " << size << endl;
    std::string rootPath = projectParentPath;
    mkdir(rootPath.c_str(), 0777);
    std::string filename = rootPath + "/" + fileName + ".csv";
    fileOutput.open(filename);
    int rows = matrices[0].rows();
    int cols = matrices[0].cols();

    // trajectory length
    for(int i = 0; i < size; i++){
        // Row
        for(int j = 0; j < (rows); j++){
            // Column
            for(int k = 0; k < cols; k++){
                fileOutput << matrices[i](j, k) << ",";
            }

        }
        fileOutput << endl;
    }
    fileOutput.close();



}

void fileHandler::saveTrajecInfomation(std::vector<MatrixXd> A_matrices, std::vector<MatrixXd> B_matrices, std::vector<MatrixXd> states, std::vector<MatrixXd> controls, std::string filePrefix, int trajecNumber, int horizonLength){
    cout << "trajectory size: " << horizonLength << endl;
    std::string rootPath = projectParentPath + "/savedTrajecInfo" + filePrefix + "/" + std::to_string(trajecNumber);
    mkdir(rootPath.c_str(), 0777);
    std::string filename = rootPath + "/A_matrices.csv";
    fileOutput.open(filename);
    int dof = A_matrices[0].rows() / 2;
    int num_ctrl = B_matrices[0].cols();

    // trajectory length
    for(int i = 0; i < horizonLength; i++){
        // Row
        for(int j = 0; j < (dof); j++){
            // Column
            for(int k = 0; k < (2 * dof); k++){
                fileOutput << A_matrices[i](j + dof, k) << ",";
            }

        }
        fileOutput << endl;
    }
    fileOutput.close();

    filename = rootPath + "/B_matrices.csv";
    fileOutput.open(filename);

    for(int i = 0; i < horizonLength; i++){
        for(int j = 0; j < (dof); j++){
            for(int k = 0; k < num_ctrl; k++){
                fileOutput << B_matrices[i](j + dof, k) << ",";
            }
        }
        fileOutput << endl;
    }

    fileOutput.close();

    filename = rootPath + "/states.csv";
    fileOutput.open(filename);
    for(int i = 0; i < horizonLength; i++){
        for(int j = 0; j < (dof * 2); j++)
        {
            fileOutput << states[i](j) << ",";
        }
        fileOutput << endl;
    }

    fileOutput.close();

    filename = rootPath + "/controls.csv";
    fileOutput.open(filename);

    for(int i = 0; i < horizonLength; i++){
        for(int j = 0; j < num_ctrl; j++) {
            fileOutput << controls[i](j) << ",";
        }
        fileOutput << endl;
    }

    fileOutput.close();
}

void fileHandler::saveTaskToFile(std::string taskPrefix, int fileNum, MatrixXd startState, MatrixXd goalState){

    std::string rootPath = projectParentPath + "/testTasks/" + taskPrefix;
    mkdir(rootPath.c_str(), 0777);
    std::string filename = rootPath + "/" + std::to_string(fileNum) + ".csv";
    fileOutput.open(filename);

    for(int i = 0; i < startState.rows(); i++){
        fileOutput << startState(i) << ",";
    }

    for(int i = 0; i < goalState.rows(); i++){
        fileOutput << goalState(i) << ",";
    }

    fileOutput << std::endl;

    fileOutput.close();

}

void fileHandler::loadTaskFromFile(std::string taskPrefix, int fileNum, MatrixXd &startState, MatrixXd &goalState){

    std::string rootPath = projectParentPath + "/testTasks" + taskPrefix;
    mkdir(rootPath.c_str(), 0777);
    std::string filename = rootPath + "/" + std::to_string(fileNum) + ".csv";

    fstream fin;

    fin.open(filename, ios::in);
    std::vector<string> row;
    std::string line, word, temp;

    while(fin >> temp){
        row.clear();

        getline(fin, line);

        stringstream s(temp);

        while(getline(s, word, ',')){
            row.push_back(word);
        }
        int stateVecSize = startState.size();
        for(int i = 0; i < stateVecSize; i++){
            startState(i) = stod(row[i]);
            goalState(i) = stod(row[i + stateVecSize]);
        }

    }
}

void fileHandler::saveCostHistory(std::vector<double> costHistory, std::string filePrefix, int trajecNumber){
    std::string rootPath = projectParentPath + "/testingData/" + filePrefix;
    mkdir(rootPath.c_str(), 0777);
    std::string filename = rootPath + "/" + std::to_string(trajecNumber) + ".csv";
    fileOutput.open(filename);

    for(int i = 0; i < costHistory.size(); i++){
        fileOutput << costHistory[i] << ",";
    }

    fileOutput << std::endl;

    fileOutput.close();
}

void fileHandler::saveResultsDataForMethods(std::string taskPrefix, std::vector<std::string> methodNames, std::vector<std::vector<double>> optTimes,
                                            std::vector<std::vector<double>> costReduction, std::vector<std::vector<double>> avgPercentageDerivs,
                                            std::vector<std::vector<double>> avgTimeGettingDerivs, std::vector<std::vector<int>> numIterations){
    std::string rootPath = projectParentPath;
    std::string filename = rootPath + taskPrefix + "_testingData.csv";

    fileOutput.open(filename);

    // Make header
    for(int i = 0; i < methodNames.size(); i++){
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
    }
    fileOutput << std::endl;

    for(int i = 0; i < methodNames.size(); i++){
        fileOutput << "optTime" << ",";
        fileOutput << "Cost reduction" << ",";
        fileOutput << "avgPercentDerivs" << ",";
        fileOutput << "avgTimeDerivs" << ",";
        fileOutput << "numIterations" << ",";
    }
    fileOutput << std::endl;

    int numTrajecs = optTimes.size();

    for(int i = 0; i < numTrajecs; i++){
        for(int j = 0; j < methodNames.size(); j++){
            fileOutput << optTimes[i][j] << ",";
            fileOutput << costReduction[i][j] << ",";
            fileOutput << avgPercentageDerivs[i][j] << ",";
            fileOutput << avgTimeGettingDerivs[i][j] << ",";
            fileOutput << numIterations[i][j] << ",";

        }
        fileOutput << std::endl;
    }
    fileOutput.close();
}

void fileHandler::saveResultsData_MPC(std::string taskPrefix, std::vector<std::string> methodNames, std::vector<std::vector<bool>> sucesses,
                         std::vector<std::vector<double>> finalDist, std::vector<std::vector<double>> executionTimes, std::vector<std::vector<double>> optimisationTimes,
                         std::vector<std::vector<double>> avgTimeGettingDerivs,std::vector<std::vector<double>> avgTimeBP, std::vector<std::vector<double>> avgTimeFP, std::vector<std::vector<double>> avgPercentDerivs){
    std::string rootPath = projectParentPath;
    std::string filename = rootPath + taskPrefix + "_testingData.csv";

    fileOutput.open(filename);

    // Make header
    for(int i = 0; i < methodNames.size(); i++){
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
    }
    fileOutput << std::endl;

    for(int i = 0; i < methodNames.size(); i++){
        fileOutput << "sucess" << ",";
        fileOutput << "final dist" << ",";
        fileOutput << "execution time" << ",";
        fileOutput << "optimisation time" << ",";
        fileOutput << "avgTimeDerivs" << ",";
        fileOutput << "avgTimeBP" << ",";
        fileOutput << "avgTimeFP" << ",";
        fileOutput << "avgpercent derivs" << ",";
    }
    fileOutput << std::endl;

    int numTrajecs = executionTimes.size();

    for(int i = 0; i < numTrajecs; i++){
        for(int j = 0; j < methodNames.size(); j++){
            if(sucesses[i][j]){
                fileOutput << 1 << ",";
            }
            else{
                fileOutput << 0 << ",";
            }
            fileOutput << finalDist[i][j] << ",";
            fileOutput << executionTimes[i][j] << ",";
            fileOutput << optimisationTimes[i][j] << ",";
            fileOutput << avgTimeGettingDerivs[i][j] << ",";
            fileOutput << avgTimeBP[i][j] << ",";
            fileOutput << avgTimeFP[i][j] << ",";
            fileOutput << avgPercentDerivs[i][j] << ",";
        }
        fileOutput << std::endl;
    }
    fileOutput.close();
}