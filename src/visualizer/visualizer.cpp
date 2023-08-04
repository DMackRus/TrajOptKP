//
// Created by dave on 07/03/23.
//

#include "visualizer.h"

visualizer::visualizer(std::shared_ptr<modelTranslator> _modelTranslator){

    mujocoHelper = _modelTranslator->mujocoHelper;
    activeModelTranslator = _modelTranslator;

    if (!glfwInit())
        mju_error("Could not initialize GLFW");
    window = glfwCreateWindow(1200, 900, "MuJoCo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mujocoHelper->initVisualisation();

    // Set window pointer to this class
    glfwSetWindowUserPointer(window, this);
    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboardCallbackWrapper);
    glfwSetCursorPosCallback(window, mouseMoveCallbackWrapper);
    glfwSetMouseButtonCallback(window, mouseButtonCallbackWrapper);
    glfwSetScrollCallback(window, scrollCallbackWrapper);
    glfwSetWindowCloseCallback(window, windowCloseCallbackWrapper);
}

// ------------------------------- Keyboard Callback -------------------------------------------------
void visualizer::keyboardCallbackWrapper(GLFWwindow* window, int key, int scancode, int action, int mods){
    visualizer* myVisualizer = static_cast<visualizer*>(glfwGetWindowUserPointer(window));
    myVisualizer->keyboard(window, key, scancode, action, mods);
}

void visualizer::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods){
    // backspace: reset simulation


    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        replayTriggered = true;
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_P){
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_O){
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_Q){
        
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_W){

    }
        // left arrow key pressed
    else if(act == GLFW_PRESS && key == GLFW_KEY_A){
        // Analyse a specific stored system state

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_S){
        // cout << "finite differencing test \n";
        // MatrixXd A, B;
        // int dataIndex = 0;
        // activeDifferentiator->getDerivatives(A, B, false, dataIndex);

        // cout << "----------------B ------------------ \n";
        // cout << B << endl;
        // cout << "--------------- A ---------------------- \n";
        // cout << A << endl;

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_Z){
        // Print screen view settings


    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_X){

    }

//     if up arrow key pressed
    else if(act == GLFW_PRESS && key == GLFW_KEY_UP){

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_DOWN){

    }
        // left arrow key pressed
    else if(act == GLFW_PRESS && key == GLFW_KEY_LEFT){


    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_RIGHT){


    }

}
// -----------------------------------------------------------------------------------------------------

// ------------------------------- Mouse button Callback -----------------------------------------------
void visualizer::mouseButtonCallbackWrapper(GLFWwindow* window, int button, int action, int mods){
    visualizer* myVisualizer = static_cast<visualizer*>(glfwGetWindowUserPointer(window));
    myVisualizer->mouse_button(window, button, action, mods);
}

// mouse button callback
void visualizer::mouse_button(GLFWwindow* window, int button, int act, int mods){
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}
// -----------------------------------------------------------------------------------------------------


// ------------------------------- Mouse move Callback ------------------------------------------------

void visualizer::mouseMoveCallbackWrapper(GLFWwindow* window, double xpos, double ypos){
    visualizer* myVisualizer = static_cast<visualizer*>(glfwGetWindowUserPointer(window));
    myVisualizer->mouse_move(window, xpos, ypos);
}

void visualizer::mouse_move(GLFWwindow* window, double xpos, double ypos){
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    mujocoHelper->mouseMove(dx, dy, button_left, button_right, window);
}
// -----------------------------------------------------------------------------------------------------

// ------------------------------- Scroll Callback ---------------------------------------------------
void visualizer::scrollCallbackWrapper(GLFWwindow* window, double xoffset, double yoffset){
    visualizer* myVisualizer = static_cast<visualizer*>(glfwGetWindowUserPointer(window));
    myVisualizer->scroll(window, xoffset, yoffset);
}
// scroll callback
void visualizer::scroll(GLFWwindow* window, double xoffset, double yoffset){
    // emulate vertical mouse motion = 5% of window height
    mujocoHelper->scroll(yoffset);
}
// -----------------------------------------------------------------------------------------------------

// ------------------------------- Window close callback -----------------------------------------------
void visualizer::windowCloseCallbackWrapper(GLFWwindow* window){
    visualizer* myVisualizer = static_cast<visualizer*>(glfwGetWindowUserPointer(window));
    myVisualizer->windowCloseCallback(window);
}

void visualizer::windowCloseCallback(GLFWwindow * /*window*/) {
    // Use this flag if you wish not to terminate now.
    // glfwSetWindowShouldClose(window, GLFW_FALSE);


//    mjv_freeScene(&scn);
//    mjr_freeContext(&con);
//
//    // free MuJoCo model and data, deactivate
//    mj_deleteData(mdata);
//    mj_deleteModel(model);
//    mj_deactivate();
}
// ----------------------------------------------------------------------------------------------------

bool visualizer::windowOpen(){
    return !glfwWindowShouldClose(window);
}

void visualizer::render(const char* label) {

    mujocoHelper->updateScene(window, label);
    glfwSwapBuffers(window);
    glfwPollEvents();
}