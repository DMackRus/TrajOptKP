//
// Created by dave on 3/30/23.
//

#ifndef PHYSICSSIMSWITCHING_FILEHANDLER_H
#define PHYSICSSIMSWITCHING_FILEHANDLER_H

#include "StdInclude.h"
#include "PhysicsSimulator.h"
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

class FileHandler{
public:
    FileHandler();
    void readModelConfigFile(std::string yamlFilePath, task &_taskConfig);
    void readSettingsFile(std::string settingsFilePath);

    void saveTrajecInfomation(std::vector<MatrixXd> A_matrices, std::vector<MatrixXd> B_matrices, std::vector<MatrixXd> states, std::vector<MatrixXd> controls, std::string filePrefix, int trajecNumber, int horizonLength);

    void saveTaskToFile(std::string filePrefix, int fileNum, MatrixXd startState, MatrixXd goalState);
    void loadTaskFromFile(std::string filePrefix, int fileNum, MatrixXd &startState, MatrixXd &goalState);

    void saveCostHistory(std::vector<double> costHistory, std::string filePrefix, int trajecNumber);

    void saveResultsDataForMethods(std::string taskPrefix, std::vector<std::string> methodNames, std::vector<std::vector<double>> optTimes,
                                   std::vector<std::vector<double>> costReduction, std::vector<std::vector<double>> avgPercentageDerivs,
                                   std::vector<std::vector<double>> avgTimeGettingDerivs, std::vector<std::vector<int>> numIterations);

    void saveResultsData_MPC(std::string taskPrefix, std::vector<std::string> methodNames, std::vector<std::vector<double>> finalCosts,
                             std::vector<std::vector<double>> avgHZ, std::vector<std::vector<double>> avgTimeGettingDerivs,
                             std::vector<std::vector<double>> avgTimeBP, std::vector<std::vector<double>> avgTimeFP, std::vector<std::vector<double>> avgPercentDerivs);

    std::string project_run_mode;
    std::string taskName;
    std::string optimiser;
    std::string taskInitMode;
    int csvRow = 0;
    std::string filtering = "none";
    bool costDerivsFD = false;
    ofstream fileOutput;

    int minIter;
    int maxIter;
    int maxHorizon;

    int keyPointMethod;
    int interpolationMethod;
    int minInterval;
    int maxInterval;

private:
    std::string projectParentPath;

};

#endif //PHYSICSSIMSWITCHING_FILEHANDLER_H
