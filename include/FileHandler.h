#pragma once

#include "StdInclude.h"
#include <yaml-cpp/yaml.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <filesystem>

class FileHandler{
public:
    FileHandler();

    void ReadModelConfigFile(const std::string& yamlFilePath, task &_taskConfig);

    void ReadSettingsFile(const std::string& settingsFilePath);

    void SaveTrajecInformation(std::vector<MatrixXd> A_matrices, std::vector<MatrixXd> B_matrices,
                               std::vector<MatrixXd> states, std::vector<MatrixXd> controls, std::string file_prefix);

    void SaveTaskToFile(std::string file_prefix, int file_num, const stateVectorList &state_vector, const vector<residual> &residuals);
    void SaveKeypointsToFile(std::string file_prefix, int file_num, const std::vector<std::vector<int>> &keypoints);
    void LoadTaskFromFile(std::string task_prefix, int file_num, stateVectorList &state_vector, vector<residual> &residuals);

//    void saveCostHistory(std::vector<double> costHistory, std::string filePrefix, int trajecNumber);
//
//    void saveResultsDataForMethods(std::string taskPrefix, std::vector<std::string> methodNames, std::vector<std::vector<double>> optTimes,
//                                   std::vector<std::vector<double>> costReduction, std::vector<std::vector<double>> avgPercentageDerivs,
//                                   std::vector<std::vector<double>> avgTimeGettingDerivs, std::vector<std::vector<int>> numIterations);
//
//    void saveResultsData_MPC(std::string taskPrefix, std::vector<std::string> methodNames, std::vector<std::vector<double>> finalCosts,
//                             std::vector<std::vector<double>> avgHZ, std::vector<std::vector<double>> avgTimeGettingDerivs,
//                             std::vector<std::vector<double>> avgTimeBP, std::vector<std::vector<double>> avgTimeFP, std::vector<std::vector<double>> avgPercentDerivs);

    std::string project_run_mode;
    std::string taskName;
    std::string optimiser;
    std::string taskInitMode;
    int csvRow = 0;
    std::string filtering = "none";
    bool costDerivsFD = false;
    bool async_mpc = true;
    bool record_trajectory = false;
    ofstream fileOutput;

    int minIter;
    int maxIter;

private:
    std::string projectParentPath;

};