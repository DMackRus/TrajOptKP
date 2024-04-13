//
// Created by dave on 01/03/23.
//

#pragma once

#include "StdInclude.h"
#include "mujoco.h"
#include <GLFW/glfw3.h>
#include <thread>

struct pose_7{
    m_point position;
    m_quat quat;
};

struct pose_6{
    m_point position;
    m_point orientation;
};

class MuJoCoHelper {
public:
    // Constructor
    MuJoCoHelper(vector<robot> robots, vector<string> _bodies);

    // Utility functions -- robots
    bool IsValidRobotName(const string& robot_name, int &robotIndex, string &robotBaseJointName);
    void SetRobotJointPositions(const string& robot_name, vector<double> joint_positions, mjData *d);
    void SetRobotJointsVelocities(const string& robot_name, vector<double> joint_velocities, mjData *d);
    void SetRobotJointsControls(const string& robot_name, vector<double> joint_controls, mjData *d);

    void GetRobotJointsPositions(const string& robot_name, vector<double> &joint_positions, mjData *d);
    void GetRobotJointsVelocities(const string& robot_name, vector<double> &joint_velocities, mjData *d);
    void GetRobotJointsAccelerations(const string& robot_name, vector<double> &joint_accelerations, mjData *d);
    void GetRobotJointsControls(const string& robot_name, vector<double> &joint_controls, mjData *d);
    void GetRobotJointsGravityCompensationControls(const string& robot_name, vector<double> &joint_controls, mjData *d);
    void GetRobotControlLimits(const string& robot_name, vector<double> &control_limits);
    void GetRobotJointLimits(const string& robot_name, vector<double> &joint_limits, mjData *d);

    // Utility functions -- bodies
//    bool isValidBodyName(const string& bodyName, int &bodyIndex);
    void SetBodyPoseQuat(const string& body_name, pose_7 pose, mjData *d) const;
    void SetBodyPoseAngle(const string& body_name, pose_6 pose, mjData *d) const;
    void SetBodyVelocity(const string& body_name, pose_6 velocity, mjData *d) const;

    void GetBodyPoseQuat(const string& body_name, pose_7 &pose, mjData *d) const;
    void GetBodyPoseAngle(const string& body_name, pose_6 &pose, mjData *d) const;
    void GetBodyVelocity(const string& body_name, pose_6 &velocity, mjData *d) const;
    void GetBodyAcceleration(const string& body_name, pose_6 &acceleration, mjData *d) const;

    void GetBodyPoseQuatViaXpos(const string& body_name, pose_7 &pose, mjData *d) const;
    void GetBodyPoseAngleViaXpos(const string& body_name, pose_6 &pose, mjData *d) const;

    // Extras
    Eigen::MatrixXd GetJacobian(const std::string& body_name, mjData *d) const;
    int CheckSystemForCollisions(mjData *d) const;
    bool CheckBodyForCollisions(const string& body_name, mjData *d) const;

    // ----- Loading and saving system states -----
    bool AppendSystemStateToEnd(mjData *d);
    bool CheckIfDataIndexExists(int list_index) const;
    bool CopySystemState(mjData *d_dest, mjData *d_src) const;
    bool DeleteSystemStateFromIndex(int list_index);
    bool ClearSystemStateList();
    void SaveDataToRolloutBuffer(mjData *d, int rollout_index);
    void CopyRolloutBufferToSavedSystemStatesList();

    static void CpMjData(const mjModel* m, mjData* d_dest, mjData* d_src);

    // ------------------------------- Visualisation -----------------------------------------
    void InitVisualisation();
    void UpdateScene(GLFWwindow *window, const char* label);
    void MouseMove(double dx, double dy, bool button_left, bool button_right, GLFWwindow *window);
    void Scroll(double yoffset);

    void InitSimulator(double timestep, const char* file_name);
    bool ForwardSimulator(mjData *d) const;
    bool ForwardSimulatorWithSkip(mjData *d, int skip_stage, int skip_sensor) const;

//    void setupMuJoCoWorld(double timestep, const char* fileName);

    void InitModelForFiniteDifferencing();
    void ResetModelAfterFiniteDifferencing() const;

    double ReturnModelTimeStep() const;

    double* SensorState(mjData *d, const std::string& sensor_name);

    vector<mjData*> saved_systems_state_list;       // List of saved system states
    vector<mjData*> fp_rollout_data;                // forwards pass rollout data
    mjData* master_reset_data{};                    // Master reset mujoco data
    mjData* main_data{};                            // main MuJoCo data
    mjData* vis_data{};                             // Visualisation MuJoCo data
    mjModel* model{};                               // MuJoCo model
    std::vector<mjData*> fd_data;                   // Finite differencing MuJoCo data - instantiated with the number of cores on the pc

    mjvCamera cam{};                                // abstract camera
    mjvScene scn{};                                 // abstract scene
    mjvOption opt{};			                    // visualization options
    mjrContext con{};				                // custom GPU context

private:
    int save_iterations{};
    mjtNum save_tolerance{};

    vector<robot> robots;
    vector<string> bodies;
};