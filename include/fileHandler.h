//
// Created by dave on 3/30/23.
//

#ifndef PHYSICSSIMSWITCHING_FILEHANDLER_H
#define PHYSICSSIMSWITCHING_FILEHANDLER_H

#include "stdInclude.h"
#include "physicsSimulator.h"
#include <yaml-cpp/yaml.h>

class fileHandler{
public:
    fileHandler();
    void readModelConfigFile(std::string yamlFilePath, vector<robot> &_robots, vector<bodyStateVec> &_bodies, std::string &modelFilePath);
    void readSettingsFile(std::string settingsFilePath);

    int project_display_mode;
    int taskNumber;
    std::string optimiser;

private:
    std::string projectParentPath;

};

#endif //PHYSICSSIMSWITCHING_FILEHANDLER_H
