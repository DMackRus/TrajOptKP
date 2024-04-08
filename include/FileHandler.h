#pragma once

#include "StdInclude.h"
#include <yaml-cpp/yaml.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

class FileHandler{
public:
    FileHandler();

    void readModelConfigFile(const std::string& yamlFilePath, task &_taskConfig);

    void readSettingsFile(std::string settingsFilePath);

    void saveTrajecInfomation(std::vector<MatrixXd> A_matrices, std::vector<MatrixXd> B_matrices, std::vector<MatrixXd> states, std::vector<MatrixXd> controls, std::string filePrefix, int trajecNumber, int horizonLength);

    void saveTaskToFile(std::string filePrefix, int fileNum, const stateVectorList &state_vector);
    void loadTaskFromFile(std::string filePrefix, int fileNum, stateVectorList &state_vector);

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

    int min_interval;

private:
    std::string projectParentPath;

};