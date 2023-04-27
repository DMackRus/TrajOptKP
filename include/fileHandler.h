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
    void readModelConfigFile(std::string yamlFilePath, vector<robot> &_robots, vector<bodyStateVec> &_bodies, std::string &modelFilePath, std::string &modelName);
    void readSettingsFile(std::string settingsFilePath);
    void readOptimisationSettingsFile(int optimiser);

    void saveTrajecInfomation(std::vector<MatrixXd> A_matrices, std::vector<MatrixXd> B_matrices, std::vector<MatrixXd> states, std::vector<MatrixXd> controls, std::string filePrefix, int trajecNumber);

    int project_display_mode;
    int taskNumber;
    std::string optimiser;
    ofstream fileOutput;

    int minIter;
    int maxIter;
    int maxHorizon;
    bool setIntervalMethod;
    int intervalSize;

private:
    std::string projectParentPath;

};

#endif //PHYSICSSIMSWITCHING_FILEHANDLER_H
