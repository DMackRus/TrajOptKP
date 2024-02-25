//
// Created by dave on 01/03/23.
//

#pragma once

#include "StdInclude.h"
#include "mujoco.h"
#include <GLFW/glfw3.h>
#include <thread>

//#define MAIN_DATA_STATE     -1
//#define MASTER_RESET_DATA   -2
//#define VISUALISATION_DATA  -3

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
    bool isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName);
    bool setRobotJointsPositions(string robotName, vector<double> jointPositions, mjData *d);
    bool setRobotJointsVelocities(string robotName, vector<double> jointVelocities, mjData *d);
    bool setRobotJointsControls(string robotName, vector<double> jointControls, mjData *d);

    bool getRobotJointsPositions(string robotName, vector<double> &jointPositions, mjData *d);
    bool getRobotJointsVelocities(string robotName, vector<double> &jointVelocities, mjData *d);
    bool getRobotJointsAccelerations(string robotName, vector<double> &jointsAccelerations, mjData *d);
    bool getRobotJointsControls(string robotName, vector<double> &jointsControls, mjData *d);
    bool getRobotJointsGravityCompensaionControls(string robotName, vector<double> &jointsControls, mjData *d);
    bool getRobotControlLimits(string robotName, vector<double> &controlLimits);
    bool getRobotJointLimits(string robotName, vector<double> &jointLimits, mjData *d);

    // Utility functions -- bodies
    bool isValidBodyName(string bodyName, int &bodyIndex);
    bool setBodyPose_quat(string bodyName, pose_7 pose, mjData *d);
    bool setBodyPose_angle(string bodyName, pose_6 pose, mjData *d);
    bool setBodyVelocity(string bodyName, pose_6 velocity, mjData *d);

    bool getBodyPose_quat(string bodyName, pose_7 &pose, mjData *d);
    bool getBodyPose_angle(string bodyName, pose_6 &pose, mjData *d);
    bool getBodyVelocity(string bodyName, pose_6 &velocity, mjData *d);
    bool getBodyAcceleration(string bodyName, pose_6 &acceleration, mjData *d);

    bool getBodyPose_quat_ViaXpos(string bodyName, pose_7 &pose, mjData *d);
    bool getBodyPose_angle_ViaXpos(string bodyName, pose_6 &pose, mjData *d);

    // Extras
    Eigen::MatrixXd calculateJacobian(std::string bodyName, mjData *d);
    int checkSystemForCollisions(mjData *d);
    bool checkBodyForCollisions(string bodyName, mjData *d);

    // ----- Loading and saving system states -----
    bool appendSystemStateToEnd(mjData *d);
    bool checkIfDataIndexExists(int list_index);
    bool copySystemState(mjData *d_dest, mjData *d_src);
    bool deleteSystemStateFromIndex(int listIndex);
    bool clearSystemStateList();
    void saveDataToRolloutBuffer(mjData *d, int rolloutIndex);
    void copyRolloutBufferToSavedSystemStatesList();

    void cpMjData(const mjModel* m, mjData* d_dest, mjData* d_src);
    std::shared_ptr<mjData> returnDesiredDataState(mjData *d);

    void _mjdTransitionFD();


    // ------------------------------- Visualisation -----------------------------------------
    void initVisualisation();
    void updateScene(GLFWwindow *window, const char* label);
    void mouseMove(double dx, double dy, bool button_left, bool button_right,  GLFWwindow *window);
    void scroll(double yoffset);

    void initSimulator(double timestep, const char* fileName);
    bool stepSimulator(int steps, mjData *d);
    bool forwardSimulator(mjData *d);
    bool forwardSimulatorWithSkip(mjData *d, int skipStage, int skipSensor);

    void setupMuJoCoWorld(double timestep, const char* fileName);

    bool setBodyPosition(string bodyName, m_point position);

    void initModelForFiniteDifferencing();
    void resetModelAfterFiniteDifferencing();

    double returnModelTimeStep();

    double* sensorState(mjData *d, std::string sensorName);

    vector<mjData*> savedSystemStatesList;      // List of saved system states
    vector<mjData*> fp_rollout_data;            // forwards pass rollout data
    mjData* master_reset_data;                  // Master reset mujoco data
    mjData* main_data;                          // main MuJoCo data
    mjData* vis_data;                           // Visualisation MuJoCo data
    mjModel* model;                             // MuJoCo model
    std::vector<mjData*> fd_data;               // Finite differencing MuJoCo data - instantiated with the number of cores on the pc

    mjvCamera cam;                   // abstract camera
    mjvScene scn;                    // abstract scene
    mjvOption opt;			        // visualization options
    mjrContext con;				    // custom GPU context

private:
    int save_iterations;
    mjtNum save_tolerance;

    vector<robot> robots;
    vector<string> bodies;

};