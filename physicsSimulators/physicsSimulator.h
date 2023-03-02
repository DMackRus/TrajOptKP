//
// Created by dave on 01/03/23.
//

#ifndef PHYSICSSIMSWITCHING_PHYSICSSIMULATOR_H
#define PHYSICSSIMSWITCHING_PHYSICSSIMULATOR_H

#include "../stdInclude/stdInclude.h"

struct robot{
    string name;
    vector<string> jointNames;
    int numActuators;
};

struct pose{
    m_point position;
    m_quat quat;
};



class physicsSimulator {
public:
    // Constructor
    physicsSimulator(vector<robot> _robots, vector<string> _bodies);

    // Utility functions
    bool isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName);
    bool setRobotJointsPositions(string robotName, vector<double> jointPositions);
    bool setRobotJointsVelocities(string robotName, vector<double> jointVelocities);
    bool setRobotJointsControls(string robotName, vector<double> jointControls);

    bool getRobotJointsPositions(string robotName, vector<double> &jointPositions);
    bool getRobotJointsVelocities(string robotName, vector<double> &jointVelocities);
    bool getRobotJointsControls(string robotName, vector<double> &joinsControls);

    virtual void initSimulator();

private:
    vector<robot> robots;
    vector<string> bodies;
};

#endif //PHYSICSSIMSWITCHING_PHYSICSSIMULATOR_H
