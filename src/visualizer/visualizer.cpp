//
// Created by dave on 07/03/23.
//

#include "visualizer.h"

visualizer::visualizer(modelTranslator *_modelTranslator, optimiser *_optimiser){

    activePhysicsSimulator = _modelTranslator->activePhysicsSimulator;
    activeModelTranslator = _modelTranslator;
    activeOptimiser = _optimiser;

    if (!glfwInit())
        mju_error("Could not initialize GLFW");
    window = glfwCreateWindow(1200, 900, "MuJoCo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    activePhysicsSimulator->initVisualisation();

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

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_P){

        activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_O){

        activePhysicsSimulator->loadSystemStateFromIndex(MAIN_DATA_STATE, 0);
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_Q){

        std::cout << "before ut " << std::endl;
        MatrixXd Xt(activeModelTranslator->stateVectorSize, 1);
        MatrixXd X_last(activeModelTranslator->stateVectorSize, 1);
        MatrixXd Ut(activeModelTranslator->num_ctrl, 1);
        MatrixXd U_last(activeModelTranslator->num_ctrl, 1);
        Ut << 0, 0;
        U_last << 0, 0;

        Xt = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);
        X_last = Xt.replicate(1, 1);
        double cost = activeModelTranslator->costFunction(Xt, Ut, X_last, U_last, false);
        std::cout << "cost: " << cost << std::endl;


        MatrixXd l_x, l_xx, l_u, l_uu;
        activeModelTranslator->costDerivatives(Xt, Ut, X_last, U_last, l_x, l_xx, l_u, l_uu, false);
        cout << "l_x: " << l_x << endl;
        cout << "l_xx:" << l_xx << endl;

        MatrixXd posVector, velVector, accelVec, stateVector;
        // posVector = activeModelTranslator->returnPositionVector();
        // velVector = activeModelTranslator->returnVelocityVector();
        // stateVector = activeModelTranslator->returnStateVector();
        accelVec = activeModelTranslator->returnAccelerationVector(MAIN_DATA_STATE);
        // cout << "pos Vector: " << posVector << endl;
        // cout << "vel vector: " << velVector << endl;
        // cout << "stateVector: " << stateVector << endl;
        cout << "accel vector: " << accelVec << endl;
        
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_W){
        MatrixXd controlVec;
        controlVec = activeModelTranslator->returnControlVector(0);
        cout << "control vec: " << controlVec << endl;

    }
        // left arrow key pressed
    else if(act == GLFW_PRESS && key == GLFW_KEY_A){
        // Analyse a specific stored system state


        int dataIndex = 1;

        MatrixXd Xt(activeModelTranslator->stateVectorSize, 1);
        MatrixXd X_last(activeModelTranslator->stateVectorSize, 1);
        MatrixXd Ut(activeModelTranslator->num_ctrl, 1);
        MatrixXd U_last(activeModelTranslator->num_ctrl, 1);


        Ut = activeModelTranslator->returnControlVector(dataIndex);
        U_last = activeModelTranslator->returnControlVector(dataIndex);

        Xt = activeModelTranslator->returnStateVector(dataIndex);
        X_last = Xt.replicate(1, 1);
        double cost = activeModelTranslator->costFunction(Xt, Ut, X_last, U_last, false);
        cout << "------------------------------------------------- \n";
        std::cout << "cost: " << cost << std::endl;


        MatrixXd l_x, l_xx, l_u, l_uu;
        activeModelTranslator->costDerivatives(Xt, Ut, X_last, U_last, l_x, l_xx, l_u, l_uu, false);
        cout << "l_x: " << l_x << endl;
        cout << "l_xx:" << l_xx << endl;

        MatrixXd posVector, velVector, accelVec, stateVector;
        posVector = activeModelTranslator->returnPositionVector(dataIndex);
        velVector = activeModelTranslator->returnVelocityVector(dataIndex);
        stateVector = activeModelTranslator->returnStateVector(dataIndex);
        accelVec = activeModelTranslator->returnAccelerationVector(dataIndex);
        cout << "pos Vector: " << posVector << endl;
        cout << "vel vector: " << velVector << endl;
        cout << "stateVector: " << stateVector << endl;
        cout << "accel vector: " << accelVec << endl;
        cout << "------------------------------------------------- \n";


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


    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_X){

    }

//     if up arrow key pressed
    else if(act == GLFW_PRESS && key == GLFW_KEY_UP){
        for(int i = 0; i < 10; i++){
            MatrixXd vel = activeModelTranslator->returnVelocityVector(MAIN_DATA_STATE);
            vel(0) = testVel;
            cout << "vel: " << vel << endl;
            activeModelTranslator->setVelocityVector(vel, MAIN_DATA_STATE);
            activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
        }
        
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_DOWN){

    }
        // left arrow key pressed
    else if(act == GLFW_PRESS && key == GLFW_KEY_LEFT){
        MatrixXd vel = activeModelTranslator->returnVelocityVector(MAIN_DATA_STATE);
        vel(0) -= 0.1;
        testVel = vel(0);
        cout << testVel << endl;
        activeModelTranslator->setVelocityVector(vel, MAIN_DATA_STATE);

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_RIGHT){
        MatrixXd vel = activeModelTranslator->returnVelocityVector(MAIN_DATA_STATE);
        vel(0) += 0.1;
        testVel = vel(0);
        cout << "vel(0) " << vel(0) << endl;
        cout << "testVel " << testVel << endl;
        activeModelTranslator->setVelocityVector(vel, MAIN_DATA_STATE);

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

    activePhysicsSimulator->mouseMove(dx, dy, button_left, button_right, window);
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
    activePhysicsSimulator->scroll(yoffset);
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

    activePhysicsSimulator->updateScene(window, label);
    glfwSwapBuffers(window);
    glfwPollEvents();
}