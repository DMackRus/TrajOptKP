//
// Created by dave on 01/03/23.
//

#ifndef PHYSICSSIMSWITCHING_PHYSICSSIMULATOR_H
#define PHYSICSSIMSWITCHING_PHYSICSSIMULATOR_H

#include "stdInclude.h"
#include <GLFW/glfw3.h>
#include "mujoco.h"
#include <thread>

#define MAIN_DATA_STATE     -1
#define MASTER_RESET_DATA   -2

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
    virtual bool isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName) = 0;
    virtual bool setRobotJointsPositions(string robotName, vector<double> jointPositions, int dataIndex) = 0;
    virtual bool setRobotJointsVelocities(string robotName, vector<double> jointVelocities, int dataIndex) = 0;
    virtual bool setRobotJointsControls(string robotName, vector<double> jointControls, int dataIndex) = 0;

    virtual bool getRobotJointsPositions(string robotName, vector<double> &jointPositions, int dataIndex) = 0;
    virtual bool getRobotJointsVelocities(string robotName, vector<double> &jointVelocities, int dataIndex) = 0;
    virtual bool getRobotJointsAccelerations(string robotName, vector<double> &jointsAccelerations, int dataIndex) = 0;
    virtual bool getRobotJointsControls(string robotName, vector<double> &jointsControls, int dataIndex) = 0;
    virtual bool getRobotJointsGravityCompensaionControls(string robotName, vector<double> &jointsControls, int dataIndex) = 0;

    // ----- Body functions -----
    virtual bool isValidBodyName(string bodyName, int &bodyIndex) = 0;
    virtual bool setBodyPose_quat(string bodyName, pose_7 pose, int dataIndex) = 0;
    virtual bool setBodyPose_angle(string bodyName, pose_6 pose, int dataIndex) = 0;
    virtual bool setBodyVelocity(string bodyName, pose_6 velocity, int dataIndex) = 0;

    virtual bool getBodyPose_quat(string bodyName, pose_7 &pose, int dataIndex) = 0;
    virtual bool getBodyPose_angle(string bodyName, pose_6 &pose, int dataIndex) = 0;
    virtual bool getBodyVelocity(string bodyName, pose_6 &velocity, int dataIndex) = 0;
    virtual bool getBodyAcceleration(string bodyName, pose_6 &acceleration, int dataIndex) = 0;

    virtual Eigen::MatrixXd calculateJacobian(std::string bodyName, int dataIndex) = 0;
    virtual int checkSystemForCollisions(int dataIndex) = 0;
    virtual bool checkBodyForCollisions(string bodyName, int dataIndex) = 0;

    // ----- Loading and saving system states -----
    virtual bool appendSystemStateToEnd(int dataIndex) = 0;
    virtual bool checkIfDataIndexExists(int dataIndex) = 0;
    virtual bool copySystemState(int dataDestinationIndex, int dataSourceIndex) = 0;
    virtual bool deleteSystemStateFromIndex(int listIndex) = 0;
    virtual bool clearSystemStateList() = 0;

    // ------------------------------- Visualisation -----------------------------------------
    virtual void initVisualisation() = 0;
    virtual void updateScene(GLFWwindow *window, const char* label) = 0;
    virtual void mouseMove(double dx, double dy, bool button_left, bool button_right,  GLFWwindow *window) = 0;
    virtual void scroll(double yoffset) = 0;

    virtual void initSimulator(double timeStep, const char* fileName) = 0;

    virtual bool stepSimulator(int steps, int dataIndex) = 0;
    virtual bool forwardSimulator(int dataIndex) = 0;
    virtual bool forwardSimulatorWithSkip(int dataIndex, int skipStage, int skipSensor) = 0;

    virtual double* sensorState(int dataIndex, std::string sensorName) = 0;



    virtual void initModelForFiniteDifferencing() = 0;
    virtual void resetModelAfterFiniteDifferencing() = 0;

    virtual double returnModelTimeStep() = 0;



protected:
    vector<robot> robots;
    vector<string> bodies;

};

#endif //PHYSICSSIMSWITCHING_PHYSICSSIMULATOR_H
