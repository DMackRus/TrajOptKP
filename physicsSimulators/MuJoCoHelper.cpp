//
// Created by dave on 01/03/23.
//

#include "MuJoCoHelper.h"

// Empty constructor
MuJoCoHelper::MuJoCoHelper(vector<robot> _robots, vector<string> _bodies): physicsSimulator(_robots, _bodies) {
    std::cout << "created mujoco helper" << std::endl;
}

// ------------------------------------    ROBOT UTILITY   --------------------------------------------
// Checks whether a robot of this name exists in the simulation
bool MuJoCoHelper::isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName){
    bool validRobot = false;
    for(int i = 0; i < robots.size(); i++){
        if(robots[i].name == robotName){
            validRobot = true;
            robotBaseJointName = robots[i].jointNames[0];
            robotIndex = i;
        }
    }

    return validRobot;
}

// Sets a robot joint positions the given values
bool MuJoCoHelper::setRobotJointsPositions(string robotName, vector<double> jointPositions){

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        if(jointPositions.size() != robots[robotIndex].numActuators){
            cout << "Invalid number of joint positions\n";
            return false;
        }
    }
    else{
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }

    // Get the body id of the base link of the robot
    int jointId = mj_name2id(model, mjOBJ_JOINT, robotBaseJointName.c_str());
    cout << "body id: " << jointId << endl;

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }
    int startIndex = model->jnt_qposadr[jointId];

    if(startIndex == -1){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    for(int i = 0; i < jointPositions.size(); i++){
        mdata->qpos[startIndex + i] = jointPositions[i];
    }

    mj_forward(model, mdata);

    return true;
}

// Sets a robot joint velocities the given values
bool MuJoCoHelper::setRobotJointsVelocities(string robotName, vector<double> jointVelocities){

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        if(jointVelocities.size() != robots[robotIndex].numActuators){
            cout << "Invalid number of joint positions\n";
            return false;
        }
    }
    else{
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }

    // Get the body id of the base link of the robot
    int jointId = mj_name2id(model, mjOBJ_JOINT, robotBaseJointName.c_str());

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }
    int startIndex = model->jnt_dofadr[jointId];

    if(startIndex == -1){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    for(int i = 0; i < jointVelocities.size(); i++){
        mdata->qvel[startIndex + i] = jointVelocities[i];
    }

    return true;
}

bool MuJoCoHelper::setRobotJointsControls(string robotName, vector<double> jointControls){

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        if(jointControls.size() != robots[robotIndex].numActuators){
            cout << "Invalid number of joint positions\n";
            return false;
        }
    }
    else{
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }

    // Get the body id of the base link of the robot
    int jointId = mj_name2id(model, mjOBJ_JOINT, robotBaseJointName.c_str());

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }
    int startIndex = model->jnt_dofadr[jointId];

    if(startIndex == -1){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    for(int i = 0; i < jointControls.size(); i++){
        mdata->ctrl[startIndex + i] = jointControls[i];
    }

    return true;
}

bool MuJoCoHelper::getRobotJointsPositions(string robotName, vector<double> &jointPositions){

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(!isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }

    // Get the body id of the base link of the robot
    int jointId = mj_name2id(model, mjOBJ_JOINT, robotBaseJointName.c_str());

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }
    int startIndex = model->jnt_dofadr[jointId];

    if(startIndex == -1){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    for(int i = 0; i < robots[robotIndex].jointNames.size(); i++){
        jointPositions.push_back(mdata->qpos[startIndex + i]);
    }

    return true;
}

bool MuJoCoHelper::getRobotJointsVelocities(string robotName, vector<double> &jointVelocities) {

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(!isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }

    // Get the body id of the base link of the robot
    int jointId = mj_name2id(model, mjOBJ_JOINT, robotBaseJointName.c_str());

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }
    int startIndex = model->jnt_dofadr[jointId];

    if(startIndex == -1){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    for(int i = 0; i < robots[robotIndex].jointNames.size(); i++){
        jointVelocities.push_back(mdata->qvel[startIndex + i]);
    }

    return true;
}

bool MuJoCoHelper::getRobotJointsControls(string robotName, vector<double> &jointVelocities) {

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(!isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }

    // Get the body id of the base link of the robot
    int jointId = mj_name2id(model, mjOBJ_JOINT, robotBaseJointName.c_str());

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }
    int startIndex = model->jnt_dofadr[jointId];

    if(startIndex == -1){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    for(int i = 0; i < robots[robotIndex].jointNames.size(); i++){
        jointVelocities.push_back(mdata->ctrl[startIndex + i]);
    }

    return true;
}
// --------------------------------- END OF ROBOT UTILITY ---------------------------------------

// ------------------------------------- BODY UTILITY -------------------------------------------
bool MuJoCoHelper::isValidBodyName(string bodyName, int &bodyIndex){
    for(int i = 0; i < bodies.size(); i++){
        if(bodies[i] == bodyName){
            bodyIndex = i;
            return true;
        }
    }
    return false;
}

bool MuJoCoHelper::setBodyPose_quat(string bodyName, pose_7 pose){

    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model->body_jntadr[bodyId];
    const int qposIndex = model->jnt_qposadr[jointIndex];

    for(int i = 0; i < 3; i++){
        mdata->qpos[qposIndex + i] = pose.position(i);
    }

    for(int i = 0; i < 4; i++){
        mdata->qpos[qposIndex + 3 + i] = pose.quat(i);
    }

    return true;

}

bool MuJoCoHelper::setBodyPose_angle(string bodyName, pose_6 pose){

    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model->body_jntadr[bodyId];
    const int qposIndex = model->jnt_qposadr[jointIndex];

    m_quat q = eul2Quat(pose.orientation);

    for(int i = 0; i < 3; i++){
        mdata->qpos[qposIndex + i] = pose.position(i);
    }

    for(int i = 0; i < 4; i++){
        mdata->qpos[qposIndex + 3 + i] = q(i);
    }

    return true;
}

bool MuJoCoHelper::setBodyVelocity(string bodyName, pose_6 velocity){

        int bodyIndex;
        if(!isValidBodyName(bodyName, bodyIndex)){
            cout << "That body doesnt exist in the simulation\n";
            return false;
        }

        int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
        const int jointIndex = model->body_jntadr[bodyId];
        const int qvelIndex = model->jnt_dofadr[jointIndex];

        for(int i = 0; i < 3; i++){
            mdata->qvel[qvelIndex + i] = velocity.position(i);
        }

        for(int i = 0; i < 3; i++){
            mdata->qvel[qvelIndex + 3 + i] = velocity.orientation(i);
        }

        return true;

}

bool MuJoCoHelper::getBodyPose_quat(string bodyName, pose_7 &pose){
    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model->body_jntadr[bodyId];
    const int qposIndex = model->jnt_qposadr[jointIndex];

    for(int i = 0; i < 3; i++){
        pose.position(i) = mdata->qpos[qposIndex + i];
    }

    for(int i = 0; i < 4; i++){
        pose.quat(i) = mdata->qpos[qposIndex + 3 + i];
    }

    return true;
}

bool MuJoCoHelper::getBodyPose_angle(string bodyName, pose_6 &pose){
    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model->body_jntadr[bodyId];
    const int qposIndex = model->jnt_qposadr[jointIndex];

    for(int i = 0; i < 3; i++){
        pose.position(i) = mdata->qpos[qposIndex + i];
    }

    m_quat tempQuat;

    for(int i = 0; i < 4; i++){
        tempQuat(i) = mdata->qpos[qposIndex + 3 + i];
    }

    m_point euler = quat2Eul(tempQuat);

    for(int i = 0; i < 3; i++){
        pose.orientation(i) = euler(i);
    }

    return true;
}

bool MuJoCoHelper::getBodyVelocity(string bodyName, pose_6 &velocity){
    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model->body_jntadr[bodyId];
    const int qvelIndex = model->jnt_dofadr[jointIndex];

    for(int i = 0; i < 3; i++){
        velocity.position(i) = mdata->qvel[qvelIndex + i];
    }

    for(int i = 0; i < 3; i++){
        velocity.orientation(i) = mdata->qvel[qvelIndex + 3 + i];
    }

    return true;
}



// ------------------------------- Keyboard Callback -------------------------------------------------
void MuJoCoHelper::keyboardCallbackWrapper(GLFWwindow* window, int key, int scancode, int action, int mods){
    MuJoCoHelper* mjHelper = static_cast<MuJoCoHelper*>(glfwGetWindowUserPointer(window));
    mjHelper->keyboard(window, key, scancode, action, mods);
}

void MuJoCoHelper::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods){
    // backspace: reset simulation
    pose_7 cheezitPose;
    getBodyPose_quat("goal", cheezitPose);
    m_point euler = quat2Eul(cheezitPose.quat);

    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {

    }

    else if(act == GLFW_PRESS && key == GLFW_KEY_Q){
        euler(0) += 0.2;
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_W){
        euler(0) -= 0.2;
    }
        // left arrow key pressed
    else if(act == GLFW_PRESS && key == GLFW_KEY_A){
        euler(1) += 0.2;
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_S){
        euler(1) -= 0.2;
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_Z){
        euler(2) += 0.2;
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_X){
        euler(2) -= 0.2;
    }

//     if up arrow key pressed
    else if(act == GLFW_PRESS && key == GLFW_KEY_UP){
        cheezitPose.position(0) += 0.2;
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_DOWN){
        cheezitPose.position(0) -= 0.2;
    }
    // left arrow key pressed
    else if(act == GLFW_PRESS && key == GLFW_KEY_LEFT){
        cheezitPose.position(1) += 0.2;
    }
    else if(act == GLFW_PRESS && key == GLFW_KEY_RIGHT){
        cheezitPose.position(1) -= 0.2;
    }

    cheezitPose.quat = eul2Quat(euler);

    setBodyPose_quat("goal", cheezitPose);
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
    window = glfwCreateWindow(1200, 900, "MuJoCo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

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

    model->opt.gravity[2] = 0;
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

            vector<double> pandaJoints;

            getRobotJointsPositions("panda", pandaJoints);
            setRobotJointsControls("panda", {1,0.5,0,-1,0,0.6,1});
            setRobotJointsVelocities("panda", {0.1, 0, 0, 0, 0, 0, 0});
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