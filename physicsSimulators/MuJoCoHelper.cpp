//
// Created by dave on 01/03/23.
//

#include "MuJoCoHelper.h"

// Empty constructor
MuJoCoHelper::MuJoCoHelper() {

}
// ------------------------------- Keyboard Callback -------------------------------------------------
void MuJoCoHelper::keyboardCallbackWrapper(GLFWwindow* window, int key, int scancode, int action, int mods){
    MuJoCoHelper* mjHelper = static_cast<MuJoCoHelper*>(glfwGetWindowUserPointer(window));
    mjHelper->keyboard(window, key, scancode, action, mods);
}

void MuJoCoHelper::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods){
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        cout << "reset MPC sim" << endl;
    }
}
// -----------------------------------------------------------------------------------------------------

// ------------------------------- Mouse button Callback -----------------------------------------------
void MuJoCoHelper::mouseButtonCallbackWrapper(GLFWwindow* window, int button, int action, int mods){
    MuJoCoHelper* mjHelper = static_cast<MuJoCoHelper*>(glfwGetWindowUserPointer(window));
    mjHelper->mouse_button(window, button, action, mods);
}

// mouse button callback
void MuJoCoHelper::mouse_button(GLFWwindow* window, int button, int act, int mods){
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}
// -----------------------------------------------------------------------------------------------------


// ------------------------------- Mouse move Callback ------------------------------------------------

void MuJoCoHelper::mouseMoveCallbackWrapper(GLFWwindow* window, double xpos, double ypos){
    MuJoCoHelper* mjHelper = static_cast<MuJoCoHelper*>(glfwGetWindowUserPointer(window));
    mjHelper->mouse_move(window, xpos, ypos);
}

void MuJoCoHelper::mouse_move(GLFWwindow* window, double xpos, double ypos){
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(model, action, dx / height, dy / height, &scn, &cam);

}
// -----------------------------------------------------------------------------------------------------

// ------------------------------- Scroll Callback ---------------------------------------------------
void MuJoCoHelper::scrollCallbackWrapper(GLFWwindow* window, double xoffset, double yoffset){
    MuJoCoHelper* mjHelper = static_cast<MuJoCoHelper*>(glfwGetWindowUserPointer(window));
    mjHelper->scroll(window, xoffset, yoffset);
}
// scroll callback
void MuJoCoHelper::scroll(GLFWwindow* window, double xoffset, double yoffset){
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}
// -----------------------------------------------------------------------------------------------------

// ------------------------------- Window close callback -----------------------------------------------
void MuJoCoHelper::windowCloseCallbackWrapper(GLFWwindow* window){
    MuJoCoHelper* mjHelper = static_cast<MuJoCoHelper*>(glfwGetWindowUserPointer(window));
    mjHelper->windowCloseCallback(window);
}

void MuJoCoHelper::windowCloseCallback(GLFWwindow * /*window*/) {
    // Use this flag if you wish not to terminate now.
    // glfwSetWindowShouldClose(window, GLFW_FALSE);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(mdata);
    mj_deleteModel(model);
    mj_deactivate();
}
// ----------------------------------------------------------------------------------------------------

void MuJoCoHelper::setupMuJoCoWorld(double timestep, const char* fileName){
    char error[1000];

    model = mj_loadXML(fileName, NULL, error, 1000);

    model->opt.timestep = timestep;

    if( !model ) {

        printf("%s\n", error);
    }

    // make data corresponding to model
    mdata = mj_makeData(model);

    if (!glfwInit())
        mju_error("Could not initialize GLFW");
    window = glfwCreateWindow(1200, 900, "iLQR_Testing", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // new


    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    //mjv_defaultPerturb(&pert);				// what data type for pert?
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);

    cam.distance = 0.891;
    cam.azimuth = 110.9;
    cam.elevation = -19.9;
    cam.lookat[0] = 0.396;
    cam.lookat[1] = -0.0629;
    cam.lookat[2] = 0.1622;

    //model->opt.gravity[2] = 0;
    //model->opt.integrator = mjINT_EULER;

    // create scene and context
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);

    // Set window pointer to this class
    glfwSetWindowUserPointer(window, this);
    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboardCallbackWrapper);
    glfwSetCursorPosCallback(window, mouseMoveCallbackWrapper);
    glfwSetMouseButtonCallback(window, mouseButtonCallbackWrapper);
    glfwSetScrollCallback(window, MuJoCoHelper::scrollCallbackWrapper);
    glfwSetWindowCloseCallback(window, MuJoCoHelper::windowCloseCallbackWrapper);

}

void MuJoCoHelper::render(){
    // run main loop, target real-time simulation and 60 fps rendering

    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = mdata->time;
        while (mdata->time - simstart < 1.0 / 60.0){

            mj_step(model, mdata);
        }

        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(model, mdata, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        mjrRect rect{0, 0, 100, 100};
        mjr_rectangle(rect, 0, 0, 0, 0);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }
}