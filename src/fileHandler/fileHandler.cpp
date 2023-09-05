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


void fileHandler::readModelConfigFile(std::string yamlFilePath, task &_taskConfig){
    YAML::Node node = YAML::LoadFile(projectParentPath + yamlFilePath);

    // General task settings
    _taskConfig.modelFilePath = projectParentPath + node["modelFile"].as<std::string>();
    _taskConfig.modelName = node["modelName"].as<std::string>();
    _taskConfig.modelTimeStep = node["timeStep"].as<double>();
    _taskConfig.keypointMethod = node["keypointMethod"].as<std::string>();
    _taskConfig.minN = node["minN"].as<int>();
    _taskConfig.maxN = node["maxN"].as<int>();
    _taskConfig.iterativeErrorThreshold = node["iterativeErrorThreshold"].as<double>();

    // Loop through robots
    for(YAML::const_iterator robot_it=node["robots"].begin(); robot_it!=node["robots"].end(); ++robot_it){
        robot tempRobot;
        string robotName;
        vector<string> jointNames;
        vector<string> actuatorNames;
        bool torqueControlled;
        vector<double> torqueLimits;
        vector<double> startPos;
        vector<double> goalPos;
        vector<double> jointPosCosts;
        vector<double> jointVelCosts;
        vector<double> terminalJointPosCosts;
        vector<double> terminalJointVelCosts;
        vector<double> jointControlCosts;
        vector<double> jointJerkThresholds;
        vector<double> magVelThresholds;

        robotName = robot_it->first.as<string>();

        for(int i = 0; i < robot_it->second["jointNames"].size(); i++){
            jointNames.push_back(robot_it->second["jointNames"][i].as<std::string>());
        }

        for(int i = 0; i < robot_it->second["actuatorNames"].size(); i++){
            actuatorNames.push_back(robot_it->second["actuatorNames"][i].as<std::string>());
        }

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

        for(int i = 0; i < robot_it->second["terminalJointPosCosts"].size(); i++){
            terminalJointPosCosts.push_back(robot_it->second["terminalJointPosCosts"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["terminalJointVelCosts"].size(); i++){
            terminalJointVelCosts.push_back(robot_it->second["terminalJointVelCosts"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["jointControlCosts"].size(); i++){
            jointControlCosts.push_back(robot_it->second["jointControlCosts"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["jointJerkThresholds"].size(); i++){
            jointJerkThresholds.push_back(robot_it->second["jointJerkThresholds"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["magVelThresholds"].size(); i++){
            magVelThresholds.push_back(robot_it->second["magVelThresholds"][i].as<double>());
        }

        tempRobot.name = robotName;
        tempRobot.jointNames = jointNames;
        tempRobot.actuatorNames = actuatorNames;
        tempRobot.torqueControlled = torqueControlled;
        tempRobot.torqueLimits = torqueLimits;
        tempRobot.startPos = startPos;
        tempRobot.goalPos = goalPos;
        tempRobot.jointPosCosts = jointPosCosts;
        tempRobot.jointVelCosts = jointVelCosts;
        tempRobot.terminalJointPosCosts = terminalJointPosCosts;
        tempRobot.terminalJointVelCosts = terminalJointVelCosts;
        tempRobot.jointControlCosts = jointControlCosts;
        tempRobot.jointJerkThresholds = jointJerkThresholds;
        tempRobot.magVelThresholds = magVelThresholds;

        _taskConfig.robots.push_back(tempRobot);
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
        double terminalLinearPosCosts[3];
        double linearVelCosts[3];
        double terminalLinearVelCosts[3];
        double angularPosCosts[3];
        double terminalAngularPosCosts[3];
        double angularVelCosts[3];
        double terminalAngularVelCosts[3];
        double linearJerkThreshold[3];
        double angularJerkThreshold[3];
        double linearMagVelThreshold[3];
        double angularMagVelThreshold[3];

        bodyName = body_it->first.as<string>();
        cout << "Body name: " << bodyName << endl;

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

        for(int i = 0; i < body_it->second["terminalLinearPosCost"].size(); i++){
            terminalLinearPosCosts[i] = body_it->second["terminalLinearPosCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["linearVelCost"].size(); i++){
            linearVelCosts[i] = body_it->second["linearVelCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["terminalLinearVelCost"].size(); i++){
            terminalLinearVelCosts[i] = body_it->second["terminalLinearVelCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["angularPosCost"].size(); i++){
            angularPosCosts[i] = body_it->second["angularPosCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["terminalAngularPosCost"].size(); i++){
            terminalAngularPosCosts[i] = body_it->second["terminalAngularPosCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["angularVelCost"].size(); i++){
            angularVelCosts[i] = body_it->second["angularVelCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["terminalAngularVelCost"].size(); i++){
            terminalAngularVelCosts[i] = body_it->second["terminalAngularVelCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["linearJerkThreshold"].size(); i++){
            linearJerkThreshold[i] = body_it->second["linearJerkThreshold"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["angularJerkThreshold"].size(); i++){
            angularJerkThreshold[i] = body_it->second["angularJerkThreshold"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["linearMagVelThreshold"].size(); i++){
            linearMagVelThreshold[i] = body_it->second["linearMagVelThreshold"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["angularMagVelThreshold"].size(); i++){
            angularMagVelThreshold[i] = body_it->second["angularMagVelThreshold"][i].as<double>();
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
            tempBody.terminalLinearPosCost[i] = terminalLinearPosCosts[i];
            tempBody.linearVelCost[i] = linearVelCosts[i];
            tempBody.terminalLinearVelCost[i] = terminalLinearVelCosts[i];
            tempBody.angularPosCost[i] = angularPosCosts[i];
            tempBody.terminalAngularPosCost[i] = terminalAngularPosCosts[i];
            tempBody.angularVelCost[i] = angularVelCosts[i];
            tempBody.terminalAngularVelCost[i] = terminalAngularVelCosts[i];
            tempBody.linearJerkThreshold[i] = linearJerkThreshold[i];
            tempBody.angularJerkThreshold[i] = angularJerkThreshold[i];
            tempBody.linearMagVelThreshold[i] = linearMagVelThreshold[i];
            tempBody.angularMagVelThreshold[i] = angularMagVelThreshold[i];
        }

        _taskConfig.bodiesStates.push_back(tempBody);
    }
}

void fileHandler::readSettingsFile(std::string settingsFilePath){

    YAML::Node node = YAML::LoadFile(projectParentPath + settingsFilePath);

    optimiser = node["optimiser"].as<std::string>();
    project_display_mode = node["displayMode"].as<int>();
    taskNumber = node["taskNumber"].as<int>();
    taskInitMode = node["taskInitMode"].as<std::string>();
    csvRow = node["csvRow"].as<int>();
    filtering = node["filtering"].as<std::string>();
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
    std::string rootPath = projectParentPath + "/savedTrajecInfo" + filePrefix + "/" + std::to_string(trajecNumber);
    mkdir(rootPath.c_str(), 0777);
    std::string filename = rootPath + "/A_matrices.csv";
    cout << "filename: " << filename << endl;
    fileOutput.open(filename);
    int dof = A_matrices[0].rows() / 2;
    int num_ctrl = B_matrices[0].cols();

    // trajectory length
    for(int i = 0; i < horizonLength - 1; i++){
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

    for(int i = 0; i < horizonLength - 1; i++){
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
    for(int i = 0; i < horizonLength - 1; i++){
        for(int j = 0; j < (dof * 2); j++)
        {
            fileOutput << states[i](j) << ",";
        }
        fileOutput << endl;
    }

    fileOutput.close();

    filename = rootPath + "/controls.csv";
    fileOutput.open(filename);

    for(int i = 0; i < horizonLength - 1; i++){
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
    std::string filename = rootPath + std::to_string(trajecNumber) + ".csv";
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

void fileHandler::saveResultsData_MPC(std::string taskPrefix, std::vector<std::string> methodNames, std::vector<std::vector<double>> finalCosts, std::vector<std::vector<double>> avgHZ,
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
    }
    fileOutput << std::endl;

    for(int i = 0; i < methodNames.size(); i++){
        fileOutput << "final costs" << ",";
        fileOutput << "avg Hz" << ",";
        fileOutput << "avgTimeDerivs" << ",";
        fileOutput << "avgTimeBP" << ",";
        fileOutput << "avgTimeFP" << ",";
        fileOutput << "avgpercent derivs" << ",";
    }
    fileOutput << std::endl;

    int numTrajecs = finalCosts.size();
    cout << "num trajecs: " << numTrajecs << endl;

    for(int i = 0; i < numTrajecs; i++){
        for(int j = 0; j < methodNames.size(); j++){
            fileOutput << finalCosts[i][j] << ",";
            fileOutput << avgHZ[i][j] << ",";
            fileOutput << avgTimeGettingDerivs[i][j] << ",";
            fileOutput << avgTimeBP[i][j] << ",";
            fileOutput << avgTimeFP[i][j] << ",";
            fileOutput << avgPercentDerivs[i][j] << ",";
        }
        fileOutput << std::endl;
    }
    fileOutput.close();
}