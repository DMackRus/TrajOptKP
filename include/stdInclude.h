//
// Created by dave on 01/03/23.
//

#ifndef PHYSICSSIMSWITCHING_STDINCLUDE_H
#define PHYSICSSIMSWITCHING_STDINCLUDE_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <chrono>
#include <random>

#define POSE_DOFS       6
#define MUJOCO_DT   0.002
#define PI          3.14152

struct robot{
    std::string name;
    std::vector<std::string> jointNames;
    int numActuators;
    bool torqueControlled;
    std::vector<double> torqueLimits;
    std::vector<double> startPos;
    std::vector<double> goalPos;
    std::vector<double> jointPosCosts;
    std::vector<double> jointVelCosts;
    std::vector<double> jointControlCosts;
    std::vector<double> jointJerkThresholds;
};

struct bodyStateVec{
    std::string name;
    bool activeLinearDOF[3];
    bool activeAngularDOF[3];
    double startLinearPos[3];
    double startAngularPos[3];
    double goalLinearPos[3];
    double goalAngularPos[3];
    double linearPosCost[3];
    double linearVelCost[3];
    double angularPosCost[3];
    double angularVelCost[3];
    double linearJerkThreshold[3];
    double angularJerkThreshold[3];
};

struct stateVectorList{
    std::vector<robot> robots;
    std::vector<bodyStateVec> bodiesStates;
};

using namespace std;
using namespace Eigen;
using namespace chrono;

typedef Eigen::Matrix<double, 3, 1> m_point;
typedef Eigen::Matrix<double, 4, 1> m_quat;

float randFloat(float floor, float ceiling);

m_quat eul2Quat(m_point eulerAngles);
m_point quat2Eul(m_quat quaternion);
m_point quat2Axis(m_quat quaternion);
m_quat rotMat2Quat(Eigen::Matrix3d rotMat);
m_quat invQuat(m_quat quat);
m_quat multQuat(m_quat quat_l, m_quat quat_r);

m_point crossProduct(m_point vec1, m_point vec2);



#endif //PHYSICSSIMSWITCHING_STDINCLUDE_H
