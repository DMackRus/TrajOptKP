//
// Created by dave on 07/03/23.
//

#include "Visualiser.h"

Visualiser::Visualiser(std::shared_ptr<ModelTranslator> _modelTranslator){

    MuJoCo_helper = _modelTranslator->MuJoCo_helper;
    activeModelTranslator = _modelTranslator;

    if (!glfwInit())
        mju_error("Could not initialize GLFW");
    window = glfwCreateWindow(1200, 900, "MuJoCo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    MuJoCo_helper->InitVisualisation();

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
void Visualiser::keyboardCallbackWrapper(GLFWwindow* window, int key, int scancode, int action, int mods){
    Visualiser* myVisualizer = static_cast<Visualiser*>(glfwGetWindowUserPointer(window));
    myVisualizer->keyboard(window, key, scancode, action, mods);
}

void Visualiser::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods){
    // backspace: reset simulation
    pose_7 body_quat;
    pose_7 goal_quat;
    MuJoCo_helper->GetBodyPoseQuat("goal", body_quat, MuJoCo_helper->vis_data);
    MuJoCo_helper->GetBodyPoseQuat("display_goal", goal_quat, MuJoCo_helper->vis_data);

    pose_6 body_eul;
    pose_6 goal_eul;
    body_eul.position = body_quat.position;
    body_eul.orientation = quat2Eul(body_quat.quat);
    goal_eul.position = goal_quat.position;
    goal_eul.orientation = quat2Eul(goal_quat.quat);

    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        replayTriggered = true;
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_P){

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_O){

    }
    // Manipulandum rotation
    else if(act == GLFW_PRESS && key == GLFW_KEY_Q){
        body_eul.orientation(0) += 0.1;
        
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_W){
        body_eul.orientation(1) += 0.1;

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_E){
        body_eul.orientation(2) += 0.1;

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_A){
        body_eul.orientation(0) -= 0.1;

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_S){
        body_eul.orientation(1) -= 0.1;
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_D){
        body_eul.orientation(2) -= 0.1;

    }
    // Goal pose
    else if(act == GLFW_PRESS && key == GLFW_KEY_R){
        goal_eul.orientation(0) += 0.1;

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_T){
        goal_eul.orientation(1) += 0.1;

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_Y){
        goal_eul.orientation(2) += 0.1;

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_F){
        goal_eul.orientation(0) -= 0.1;

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_G){
        goal_eul.orientation(1) -= 0.1;

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_H){
        goal_eul.orientation(2) -= 0.1;

    }

    else if(act == GLFW_PRESS && key == GLFW_KEY_Z){


    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_X){

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_UP){
        
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_DOWN){

    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_LEFT){


    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_RIGHT){

    }

    if(act == GLFW_PRESS){
        // Convert euler to quaternion
        body_quat.quat = eul2Quat(body_eul.orientation);
        goal_quat.quat = eul2Quat(goal_eul.orientation);
        MuJoCo_helper->SetBodyPoseQuat("goal", body_quat, MuJoCo_helper->vis_data);
        MuJoCo_helper->SetBodyPoseQuat("display_goal", goal_quat, MuJoCo_helper->vis_data);

        std::cout << "body orientation " << body_eul.orientation(0) << " " << body_eul.orientation(1) << " " << body_eul.orientation(2) << "\n";
        double cost = activeModelTranslator->CostFunction(MuJoCo_helper->vis_data,
                                                          activeModelTranslator->full_state_vector, false);
        std::cout << "cost: " << cost << std::endl;

        MatrixXd l_x, l_u, l_xx, l_uu;
//    activeModelTranslator->CostDerivatives(MuJoCo_helper->vis_data, l_x, l_xx, l_u, l_uu, false);

        //test rotations difference with f.d
        int rot_index = 10;

//        std::cout << "l_x from cost derivs: \n" << l_x.block(rot_index, 0, 3, 1).transpose() << std::endl;
    }

}
// -----------------------------------------------------------------------------------------------------

// ------------------------------- Mouse button Callback -----------------------------------------------
void Visualiser::mouseButtonCallbackWrapper(GLFWwindow* window, int button, int action, int mods){
    Visualiser* myVisualizer = static_cast<Visualiser*>(glfwGetWindowUserPointer(window));
    myVisualizer->mouse_button(window, button, action, mods);
}

// mouse button callback
void Visualiser::mouse_button(GLFWwindow* window, int button, int act, int mods){
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}
// -----------------------------------------------------------------------------------------------------


// ------------------------------- Mouse move Callback ------------------------------------------------

void Visualiser::mouseMoveCallbackWrapper(GLFWwindow* window, double xpos, double ypos){
    Visualiser* myVisualizer = static_cast<Visualiser*>(glfwGetWindowUserPointer(window));
    myVisualizer->mouse_move(window, xpos, ypos);
}

void Visualiser::mouse_move(GLFWwindow* window, double xpos, double ypos){
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    MuJoCo_helper->MouseMove(dx, dy, button_left, button_right, window);
}
// -----------------------------------------------------------------------------------------------------

// ------------------------------- Scroll Callback ---------------------------------------------------
void Visualiser::scrollCallbackWrapper(GLFWwindow* window, double xoffset, double yoffset){
    Visualiser* myVisualizer = static_cast<Visualiser*>(glfwGetWindowUserPointer(window));
    myVisualizer->scroll(window, xoffset, yoffset);
}
// scroll callback
void Visualiser::scroll(GLFWwindow* window, double xoffset, double yoffset){
    // emulate vertical mouse motion = 5% of window height
    MuJoCo_helper->Scroll(yoffset);
}
// -----------------------------------------------------------------------------------------------------

// ------------------------------- Window close callback -----------------------------------------------
void Visualiser::windowCloseCallbackWrapper(GLFWwindow* window){
    Visualiser* myVisualizer = static_cast<Visualiser*>(glfwGetWindowUserPointer(window));
    myVisualizer->windowCloseCallback(window);
}

void Visualiser::windowCloseCallback(GLFWwindow * /*window*/) {
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

bool Visualiser::windowOpen(){
    return !glfwWindowShouldClose(window);
}

void Visualiser::render(const char* label) {

    MuJoCo_helper->UpdateScene(window, label);
    glfwSwapBuffers(window);
    glfwPollEvents();
}