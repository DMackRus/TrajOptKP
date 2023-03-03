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

struct pose_7{
    m_point position;
    m_quat quat;
};

struct pose_6{
    m_point position;
    m_point orientation;
};



class physicsSimulator {
public:
    // Constructor
    physicsSimulator(vector<robot> _robots, vector<string> _bodies);

    // ----- Robot functions -----
    virtual bool isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName);
    virtual bool setRobotJointsPositions(string robotName, vector<double> jointPositions);
    virtual bool setRobotJointsVelocities(string robotName, vector<double> jointVelocities);
    virtual bool setRobotJointsControls(string robotName, vector<double> jointControls);

    virtual bool getRobotJointsPositions(string robotName, vector<double> &jointPositions);
    virtual bool getRobotJointsVelocities(string robotName, vector<double> &jointVelocities);
    virtual bool getRobotJointsControls(string robotName, vector<double> &joinsControls);

    // ----- Body functions -----
    virtual bool isValidBodyName(string bodyName, int &bodyIndex);
    virtual bool setBodyPose_quat(string bodyName, pose_7 pose);
    virtual bool setBodyPose_angle(string bodyName, pose_6 pose);
    virtual bool setBodyVelocity(string bodyName, pose_6 velocity);

    virtual bool getBodyPose_quat(string bodyName, pose_7 &pose);
    virtual bool getBodyPose_angle(string bodyName, pose_6 &pose);
    virtual bool getBodyVelocity(string bodyName, pose_6 &velocity);

    virtual void initSimulator();
protected:
    vector<robot> robots;
    vector<string> bodies;

};

#endif //PHYSICSSIMSWITCHING_PHYSICSSIMULATOR_H
