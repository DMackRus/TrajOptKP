//
// Created by dave on 01/03/23.
//

#ifndef PHYSICSSIMSWITCHING_MUJOCOHELPER_H
#define PHYSICSSIMSWITCHING_MUJOCOHELPER_H

#include "physicsSimulator.h"
#include "mujoco.h"
#include "glfw3.h"

class MuJoCoHelper : public physicsSimulator {
public:
    // Constructor
    MuJoCoHelper(vector<robot> robots, vector<string> _bodies);

    // Utility functions -- robots
    bool isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName);
    bool setRobotJointsPositions(string robotName, vector<double> jointPositions);
    bool setRobotJointsVelocities(string robotName, vector<double> jointVelocities);
    bool setRobotJointsControls(string robotName, vector<double> jointControls);

    bool getRobotJointsPositions(string robotName, vector<double> &jointPositions);
    bool getRobotJointsVelocities(string robotName, vector<double> &jointVelocities);
    bool getRobotJointsControls(string robotName, vector<double> &joinsControls);

    // Utility functions -- bodies
    bool isValidBodyName(string bodyName, int &bodyIndex);
    bool setBodyPose(string bodyName, pose pose);
    bool setBodyPosition(string bodyName, m_point position);


    // ------------------------------- Variables -----------------------------------------
    // Screen variables
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;
    double lastx = 0;
    double lasty = 0;

    mjvCamera cam;                   // abstract camera
    mjvScene scn;                    // abstract scene
    mjvOption opt;			        // visualization options
    mjrContext con;				    // custom GPU context
    GLFWwindow *window;
    // Internal variables

    // ------------------------------- Functions -------------------------------------------

    // Callback wrappers to interface c glfw with c++
    static void scrollCallbackWrapper(GLFWwindow* window, double xoffset, double yoffset);
    static void mouseMoveCallbackWrapper(GLFWwindow* window, double xpos, double ypos);
    static void mouseButtonCallbackWrapper(GLFWwindow* window, int button, int action, int mods);
    static void keyboardCallbackWrapper(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void windowCloseCallbackWrapper(GLFWwindow *window);

    // Screen callbacks for GL window
    void scroll(GLFWwindow* window, double xoffset, double yoffset);
    void mouse_move(GLFWwindow* window, double xpos, double ypos);
    void mouse_button(GLFWwindow* window, int button, int act, int mods);
    void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
    void windowCloseCallback(GLFWwindow * /*window*/);


    // UI rendering
    void render();

    void setupMuJoCoWorld(double timestep, const char* fileName);

private:
    mjData *mdata;                   // MuJoCo data
    mjModel *model;                  // MuJoCo model
    vector<robot> robots;
    vector<string> bodies;
};

#endif //PHYSICSSIMSWITCHING_MUJOCOHELPER_H
