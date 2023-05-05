//
// Created by dave on 3/30/23.
//

#ifndef PHYSICSSIMSWITCHING_FILEHANDLER_H
#define PHYSICSSIMSWITCHING_FILEHANDLER_H

#include "stdInclude.h"
#include "physicsSimulator.h"
#include <yaml-cpp/yaml.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

enum optimisers{
        opt_iLQR = 0,
        opt_stomp = 1,
        opt_gradDescent = 2
};

class fileHandler{
public:
    fileHandler();
    void readModelConfigFile(std::string yamlFilePath, vector<robot> &_robots, vector<bodyStateVec> &_bodies, std::string &modelFilePath, std::string &_modelName);
    void readSettingsFile(std::string settingsFilePath);
    void readOptimisationSettingsFile(int optimiser);

    void saveTrajecInfomation(std::vector<MatrixXd> A_matrices, std::vector<MatrixXd> B_matrices, std::vector<MatrixXd> states, std::vector<MatrixXd> controls, std::string filePrefix, int trajecNumber);

    void saveTaskToFile(std::string filePrefix, int fileNum, MatrixXd startState, MatrixXd goalState);
    void loadTaskFromFile(std::string filePrefix, int fileNum, MatrixXd &startState, MatrixXd &goalState);

    void saveCostHistory(std::vector<double> costHistory, std::string filePrefix, int trajecNumber);

    int project_display_mode;
    int taskNumber;
    std::string optimiser;
    std::string taskInitMode;
    int csvRow = 0;
    bool filtering = false;
    ofstream fileOutput;

    int minIter;
    int maxIter;
    int maxHorizon;

    int keyPointMethod;
    int interpolationMethod;
    int minInterval;
    int maxInterval;

    std::string modelName;

private:
    std::string projectParentPath;

};

#endif //PHYSICSSIMSWITCHING_FILEHANDLER_H
