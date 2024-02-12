//
// Created by dave on 07/03/23.
//

#ifndef PHYSICSSIMSWITCHING_VISUALIZER_H
#define PHYSICSSIMSWITCHING_VISUALIZER_H

#include "MuJoCoHelper.h"
#include <GLFW/glfw3.h>
#include "StdInclude.h"
#include "ModelTranslator.h"
#include "Differentiator.h"
#include "Optimiser.h"

class Visualiser {
public:
    Visualiser(std::shared_ptr<ModelTranslator> _modelTranslator);
    void init();
    void update();
    void draw();
    void close();

    // ------------------------------- Variables -----------------------------------------
    // Screen variables
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;
    double lastx = 0;
    double lasty = 0;

    GLFWwindow* window;
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
    bool windowOpen();
    void render(const char* label);

    float testVel = 0;

    std::vector<MatrixXd> replayControls;
    bool replayTriggered = false;

    // Asynchronus control variables
    std::vector<MatrixXd> controlBuffer;
    int current_control_index = 0;
    bool new_controls_flag = false;
    std::vector<MatrixXd> trajectory_states;
    std::vector<MatrixXd> trajectory_controls;
    bool task_finished = false;

private:
    std::shared_ptr<MuJoCoHelper> MuJoCo_helper;
    std::shared_ptr<ModelTranslator> activeModelTranslator;
};

#endif //PHYSICSSIMSWITCHING_VISUALIZER_H
