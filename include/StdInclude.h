#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <chrono>
#include <random>
#include <iomanip>

#define PI          3.14152

struct robot{
    std::string name;
    std::string root_name;
    std::vector<std::string> joint_names;
    std::vector<std::string> actuator_names;

    std::vector<double> start_pos;
//    std::vector<double> goal_pos;
//    std::vector<double> goal_vel;

    std::vector<double> jerk_thresholds;
    std::vector<double> vel_change_thresholds;
};

struct rigid_body{
    std::string name;
    bool active_linear_dof[3];
    bool active_angular_dof[3];
    double start_linear_pos[3];
    double start_angular_pos[3];
//    double goal_linear_pos[3];
//    double goal_angular_pos[3];
//    double linearPosCost[3];
//    double terminal_linear_pos_cost[3];
//    double linear_vel_cost[3];
//    double terminal_linear_vel_cost[3];
//    double angular_pos_cost[3];
//    double terminal_angular_pos_cost[3];
//    double angular_vel_cost[3];
//    double terminal_angular_vel_cost[3];
    double linear_jerk_threshold[3];
    double angular_jerk_threshold[3];
    double linear_vel_change_threshold[3];
    double angular_vel_change_threshold[3];
};

struct vertex{
    bool active_linear_dof[3];
    double linear_jerk_threshold[3];
    double linear_vel_change_threshold[3];
};

struct soft_body{
    std::string name;
    int num_vertices;
//    std::vector<bool> active_linear_dof;
    std::vector<vertex> vertices;

    // Centroid of the soft body
    double start_linear_pos[3];
    double start_angular_pos[3];
//    double goal_linear_pos[3];
//    double goal_angular_pos[3];
//    double linearPosCost[3];
//    double terminal_linear_pos_cost[3];
//    double linear_vel_cost[3];
//    double terminal_linear_vel_cost[3];
//    double angular_pos_cost[3];
//    double terminal_angular_pos_cost[3];
//    double angular_vel_cost[3];
//    double terminal_angular_vel_cost[3];

    // Individual vertices specific
//    std::vector<double> linear_jerk_threshold;
//    std::vector<double> linear_vel_change_threshold;
};

struct residual{
    std::string name;
    std::vector<double> target;
    int resid_dimension;
    double weight;
    double weight_terminal;
};

struct task{
    std::vector<robot> robots;
    std::vector<rigid_body> rigid_bodies;
    std::vector<soft_body> soft_bodies;
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
    std::vector<residual> residuals;
};

struct stateVectorList{
    int dof = 0;
    int dof_quat = 0;
    int num_ctrl = 0;
    std::vector<std::string> state_names;
    std::vector<robot> robots;
    std::vector<rigid_body> rigid_bodies;
    std::vector<soft_body> soft_bodies;

    void Update(){
        dof = 0;
        dof_quat = 0;
        num_ctrl = 0;
        state_names.clear();

        std::string lin_suffixes[3] = {"_x", "_y", "_z"};
        std::string ang_suffixes[3] = {"_roll", "_pitch", "_yaw"};

        for(auto & robot : robots){
            // Check if robot has a free root joint?
            if(robot.root_name != "-"){
                dof += 6;
                dof_quat += 7;

                state_names.push_back(robot.root_name + "_x");
                state_names.push_back(robot.root_name + "_y");
                state_names.push_back(robot.root_name + "_z");
                state_names.push_back(robot.root_name + "_roll");
                state_names.push_back(robot.root_name + "_pitch");
                state_names.push_back(robot.root_name + "_yaw");
            }


            // Increment the number of dofs by the number of joints in the robots
            dof += static_cast<int>(robot.joint_names.size());
            dof_quat += static_cast<int>(robot.joint_names.size());
            num_ctrl += static_cast<int>(robot.actuator_names.size());

            for(const auto & joint_name : robot.joint_names){
                state_names.push_back(joint_name);
            }
        }

        // Loop through all bodies in the state vector
        for(auto & body : rigid_bodies) {
            for(int i = 0; i < 3; i++) {
                if(body.active_linear_dof[i]) {
                    dof++;
                    dof_quat++;
                    state_names.push_back(body.name + lin_suffixes[i]);
                }
            }

            bool any_angular_dofs = false;
            for(int i = 0; i < 3; i++){
                if(body.active_angular_dof[i]){
                    any_angular_dofs = true;
                    dof++;
                    state_names.push_back(body.name + ang_suffixes[i]);
                }
            }

            if(any_angular_dofs){
                dof_quat += 4;
            }
        }

        for(auto & soft_body: soft_bodies){
            for(int i = 0; i < soft_body.num_vertices; i++){
                for(int j = 0; j < 3; j++){
                    if(soft_body.vertices[i].active_linear_dof[j]){
                        dof++;
                        dof_quat++;
                        state_names.push_back(soft_body.name + "_V" + std::to_string(i) + lin_suffixes[j]);
                    }
                }
            }
        }
    }

    void PrintStateVector(){
        for(auto & state_name : state_names){
            std::cout << state_name << " ";
        }
        std::cout << "\n";
    }

    void PrintFormattedStateVector(){
        std::string lin_suffixes[3] = {"_x", "_y", "_z"};
        std::string ang_suffixes[3] = {"_roll", "_pitch", "_yaw"};

        std::cout << "---- robots ----\n";
        for(auto& robot : robots){
            std::cout << robot.name << ": ";
            if(robot.root_name != "-"){
                std::cout << robot.root_name + "_x" << " ";
                std::cout << robot.root_name + "_y" << " ";
                std::cout << robot.root_name + "_z" << " ";
                std::cout << robot.root_name + "_roll" << " ";
                std::cout << robot.root_name + "_pitch" << " ";
                std::cout << robot.root_name + "_yaw" << " ";
            }

            for(auto& joint_name: robot.joint_names){
                std::cout << joint_name << " ";
            }
            std::cout << "\n";
        }

        std::cout << "------ rigid bodies -----\n";
        for(auto& rigid_body : rigid_bodies){
            std::cout << rigid_body.name << ": ";
            for(int i = 0; i < 3; i++) {
                if(rigid_body.active_linear_dof[i]) {
                    std::cout << lin_suffixes[i] << " ";
                }
            }

            for(int i = 0; i < 3; i++){
                if(rigid_body.active_angular_dof[i]){
                    std::cout << ang_suffixes[i] << " ";
                }
            }
            std::cout << "\n";
        }

        std::cout << "------ soft bodies -----\n";
        for(auto& soft_body: soft_bodies){
            std::cout << soft_body.name << ": ";
            for(int i = 0; i < soft_body.num_vertices; i++){
                for(int j = 0; j < 3; j++){
                    if(soft_body.vertices[i].active_linear_dof[j]){
                        std::cout << "_V" + std::to_string(i) + lin_suffixes[j] << " ";
                    }
                }
            }
            std::cout << "\n";
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

inline std::string GetCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_c);

    std::ostringstream oss;
    oss << std::put_time(now_tm, "%Y%m%d_%H%M");
    return oss.str();
}

bool endsWith(const std::string& mainString, const std::string& subString);
