#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <chrono>
#include <random>

#define PI          3.14152

struct robot{
    std::string name;
    std::vector<std::string> jointNames;
    std::vector<std::string> actuatorNames;
    bool torqueControlled;
    std::vector<double> torqueLimits;
    std::vector<double> startPos;
    std::vector<double> goalPos;
    std::vector<double> goalVel;
    std::vector<double> jointPosCosts;
    std::vector<double> jointVelCosts;
    std::vector<double> terminalJointPosCosts;
    std::vector<double> terminalJointVelCosts;
    std::vector<double> jointControlCosts;
    std::vector<double> jointJerkThresholds;
    std::vector<double> magVelThresholds;
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
    double terminalLinearPosCost[3];
    double linearVelCost[3];
    double terminalLinearVelCost[3];
    double angularPosCost[3];
    double terminalAngularPosCost[3];
    double angularVelCost[3];
    double terminalAngularVelCost[3];
    double linearJerkThreshold[3];
    double angularJerkThreshold[3];
    double linearMagVelThreshold[3];
    double angularMagVelThreshold[3];
};

struct task{
    std::vector<robot> robots;
    std::vector<bodyStateVec> bodiesStates;
    double modelTimeStep;
    int openloop_horizon;
    int mpc_horizon;
    std::string modelName;
    std::string modelFilePath;
    std::string keypointMethod;
    bool auto_adjust;
    int minN;
    int maxN;
    std::vector<double> jerkThresholds;
    std::vector<double> acellThresholds;
    double iterativeErrorThreshold;
    std::vector<double> magVelThresholds;
};

struct stateVectorList{
    int dof = 0;
    int dof_quat = 0;
    int num_ctrl = 0;
    std::vector<std::string> state_names;
    std::vector<robot> robots;
    std::vector<bodyStateVec> bodiesStates;

    void Update(){
        dof = 0;
        dof_quat = 0;
        num_ctrl = 0;
        state_names.clear();

        std::string lin_suffixes[3] = {"_x", "_y", "_z"};
        std::string ang_suffixes[3] = {"_roll", "_pitch", "_yaw"};

        for(auto & robot : robots){
            // Increment the number of dofs by the number of joints in the robots
            dof += static_cast<int>(robot.jointNames.size());
            dof_quat += static_cast<int>(robot.jointNames.size());
            num_ctrl += static_cast<int>(robot.actuatorNames.size());

            for(const auto & joint_name : robot.jointNames){
                state_names.push_back(joint_name);
            }
        }

        // Loop through all bodies in the state vector
        for(auto & body : bodiesStates) {
            for(int i = 0; i < 3; i++) {
                if(body.activeLinearDOF[i]) {
                    dof++;
                    dof_quat++;
                    state_names.push_back(body.name + lin_suffixes[i]);
                }
            }

            bool any_angular_dofs = false;
            for(int i = 0; i < 3; i++){
                if(body.activeAngularDOF[i]){
                    any_angular_dofs = true;
                    dof++;
                    state_names.push_back(body.name + ang_suffixes[i]);
                }
            }

            if(any_angular_dofs){
                dof_quat += 4;
            }
        }
    }
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
m_quat axis2Quat(m_point axisAngles);
Eigen::Matrix3d eul2RotMat(m_point euler);
m_quat rotMat2Quat(Eigen::Matrix3d rotMat);
m_quat invQuat(m_quat quat);
m_quat multQuat(m_quat quat_l, m_quat quat_r);

m_point crossProduct(m_point vec1, m_point vec2);

double GaussNoise(double mean, double stddev);

bool CompareDescend(const std::pair<double, int>& a, const std::pair<double, int>& b);
bool CompareAscend(const std::pair<double, int>& a, const std::pair<double, int>& b);

std::vector<int> SortIndices(const std::vector<double>& values, bool ascending);

template <typename T>
inline T* DataAt(std::vector<T>& vec, typename std::vector<T>::size_type elem) {
    if (elem < vec.size()) {
        return &vec[elem];
    } else {
        return nullptr;
    }
}

bool endsWith(const std::string& mainString, const std::string& subString);
