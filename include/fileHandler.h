//
// Created by dave on 3/30/23.
//

#ifndef PHYSICSSIMSWITCHING_FILEHANDLER_H
#define PHYSICSSIMSWITCHING_FILEHANDLER_H

#include "stdInclude.h"
#include "physicsSimulator.h"
#include <yaml-cpp/yaml.h>

enum optimisers{
        opt_iLQR = 0,
        opt_stomp = 1,
        opt_gradDescent = 2
};

class fileHandler{
public:
    fileHandler();
    void readModelConfigFile(std::string yamlFilePath, vector<robot> &_robots, vector<bodyStateVec> &_bodies, std::string &modelFilePath);
    void readSettingsFile(std::string settingsFilePath);
    void readOptimisationSettingsFile(int optimiser);

    int project_display_mode;
    int taskNumber;
    std::string optimiser;

    int minIter;
    int maxIter;
    int maxHorizon;
    bool setIntervalMethod;
    int intervalSize;

private:
    std::string projectParentPath;

};

#endif //PHYSICSSIMSWITCHING_FILEHANDLER_H
