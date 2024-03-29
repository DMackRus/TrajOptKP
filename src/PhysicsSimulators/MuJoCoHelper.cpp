//
// Created by dave on 01/03/23.
//

#include "MuJoCoHelper.h"

// Empty constructor
MuJoCoHelper::MuJoCoHelper(vector<robot> _robots, vector<string> _bodies) {
    // Set the robots and bodies
    robots = _robots;
    bodies = _bodies;
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
bool MuJoCoHelper::setRobotJointsPositions(string robotName, vector<double> jointPositions, mjData *d){

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        if(jointPositions.size() != robots[robotIndex].jointNames.size()){
            cout << "Invalid number of joint positions\n";
            return false;
        }
    }
    else{
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }


    for(int i = 0; i < jointPositions.size(); i++){
        int jointId = mj_name2id(model, mjOBJ_JOINT, robots[robotIndex].jointNames[i].c_str());
        if(jointId == -1){
            cout << "Invalid bodyId for robot\n";
            return false;
        }
        int qposIndex = model->jnt_qposadr[jointId];
        d->qpos[qposIndex] = jointPositions[i];
    }

    return true;
}

// Sets a robot joint velocities the given values
bool MuJoCoHelper::setRobotJointsVelocities(string robotName, vector<double> jointVelocities, mjData *d){

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        if(jointVelocities.size() != robots[robotIndex].jointNames.size()){
            cout << "Invalid number of joint positions\n";
            return false;
        }
    }
    else{
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }


    for(int i = 0; i < jointVelocities.size(); i++){
        int jointId = mj_name2id(model, mjOBJ_JOINT, robots[robotIndex].jointNames[i].c_str());
        if(jointId == -1){
            cout << "Invalid bodyId for robot\n";
            return false;
        }
        int qposIndex = model->jnt_qposadr[jointId];
        d->qvel[qposIndex] = jointVelocities[i];
    }

    return true;
}

bool MuJoCoHelper::setRobotJointsControls(string robotName, vector<double> jointControls, mjData *d){

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        if(jointControls.size() != robots[robotIndex].actuatorNames.size()){
            cout << "Invalid number of joint positions\n";
            return false;
        }
    }
    else{
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }


    for(int i = 0; i < jointControls.size(); i++){
        int actuatorId = mj_name2id(model, mjOBJ_ACTUATOR, robots[robotIndex].actuatorNames[i].c_str());
        int qposIndex = model->jnt_dofadr[actuatorId];
        d->ctrl[qposIndex] = jointControls[i];
    }

    return true;
}

bool MuJoCoHelper::getRobotJointsPositions(string robotName, vector<double> &jointPositions, mjData *d){

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(!isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }


    for(int i = 0; i < robots[robotIndex].jointNames.size(); i++){
        int jointId = mj_name2id(model, mjOBJ_JOINT, robots[robotIndex].jointNames[i].c_str());
        if(jointId == -1){
            cout << "Invalid bodyId for robot\n";
            return false;
        }
        int qposIndex = model->jnt_qposadr[jointId];
        jointPositions.push_back(d->qpos[qposIndex]);
    }

    return true;
}

bool MuJoCoHelper::getRobotJointsVelocities(string robotName, vector<double> &jointVelocities, mjData *d) {

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(!isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }


    for(int i = 0; i < robots[robotIndex].jointNames.size(); i++){
        int jointId = mj_name2id(model, mjOBJ_JOINT, robots[robotIndex].jointNames[i].c_str());
        if(jointId == -1){
            cout << "Invalid bodyId for robot\n";
            return false;
        }
        int qposIndex = model->jnt_qposadr[jointId];
        jointVelocities.push_back(d->qvel[qposIndex]);
    }

    return true;
}

bool MuJoCoHelper::getRobotJointsAccelerations(string robotName, vector<double> &jointAccelerations, mjData *d){
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
        jointAccelerations.push_back(d->qacc[startIndex + i]);
    }

    return true;
}

bool MuJoCoHelper::getRobotJointsControls(string robotName, vector<double> &jointControls, mjData *d) {

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(!isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }


    for(int i = 0; i < robots[robotIndex].actuatorNames.size(); i++){
        int actuatorId = mj_name2id(model, mjOBJ_ACTUATOR, robots[robotIndex].actuatorNames[i].c_str());
        int ctrlIndex = model->jnt_dofadr[actuatorId];
        jointControls.push_back(d->ctrl[ctrlIndex]);
    }

    return true;
}

bool MuJoCoHelper::getRobotJointsGravityCompensaionControls(string robotName, vector<double> &jointsControls, mjData *d){

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

    // TODO (DMackRus) - Check if this is needed?
    mj_forward(model, d);

    for(int i = 0; i < robots[robotIndex].jointNames.size(); i++){
        jointsControls.push_back(d->qfrc_bias[startIndex + i]);
    }

    return true;
}

bool MuJoCoHelper::getRobotControlLimits(string robotName, vector<double> &controlLimits){

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

    for(int i = 0; i < 2 * robots[robotIndex].jointNames.size(); i++){
        controlLimits.push_back(model->actuator_ctrlrange[i]);
    }

    return true;
}

bool MuJoCoHelper::getRobotJointLimits(string robotName, vector<double> &jointLimits, mjData *d){
    return false;
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
    // return false;
    return true;
}

bool MuJoCoHelper::setBodyPose_quat(string bodyName, pose_7 pose, mjData *d){

    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model->body_jntadr[bodyId];
    const int qposIndex = model->jnt_qposadr[jointIndex];



    for(int i = 0; i < 3; i++){
        d->qpos[qposIndex + i] = pose.position(i);
    }

    for(int i = 0; i < 4; i++){
        d->qpos[qposIndex + 3 + i] = pose.quat(i);
    }

    return true;
}

bool MuJoCoHelper::setBodyPose_angle(string bodyName, pose_6 pose, mjData *d){

    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation: " << bodyName << "\n";
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model->body_jntadr[bodyId];
    const int qposIndex = model->jnt_qposadr[jointIndex];

    m_quat q = eul2Quat(pose.orientation);



    for(int i = 0; i < 3; i++){
        d->qpos[qposIndex + i] = pose.position(i);
    }

//    cout << "bodyName: " << bodyName << "\n";
//    cout << "pose.position: " << pose.position << "\n";

    for(int i = 0; i < 4; i++){
        d->qpos[qposIndex + 3 + i] = q(i);
    }

    return true;
}

bool MuJoCoHelper::setBodyVelocity(string bodyName, pose_6 velocity, mjData *d){

    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model->body_jntadr[bodyId];
    const int qvelIndex = model->jnt_dofadr[jointIndex];



    for(int i = 0; i < 3; i++){
        d->qvel[qvelIndex + i] = velocity.position(i);
    }

    for(int i = 0; i < 3; i++){
        d->qvel[qvelIndex + 3 + i] = velocity.orientation(i);
    }

    return true;
}

bool MuJoCoHelper::getBodyPose_quat(string bodyName, pose_7 &pose, mjData *d){
    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model->body_jntadr[bodyId];
    const int qposIndex = model->jnt_qposadr[jointIndex];



    for(int i = 0; i < 3; i++){
        pose.position(i) = d->qpos[qposIndex + i];
    }

    for(int i = 0; i < 4; i++){
        pose.quat(i) = d->qpos[qposIndex + 3 + i];
    }

    return true;
}

bool MuJoCoHelper::getBodyPose_angle(string bodyName, pose_6 &pose, mjData *d){
    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation " << bodyName << endl;
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model->body_jntadr[bodyId];
    const int qposIndex = model->jnt_qposadr[jointIndex];



    for(int i = 0; i < 3; i++){
        pose.position(i) = d->qpos[qposIndex + i];
    }

    m_quat tempQuat;

    for(int i = 0; i < 4; i++){
        tempQuat(i) = d->qpos[qposIndex + 3 + i];
    }

    m_point euler = quat2Eul(tempQuat);

    for(int i = 0; i < 3; i++){
        pose.orientation(i) = euler(i);
    }

    return true;
}

bool MuJoCoHelper::getBodyPose_angle_ViaXpos(string bodyName, pose_6 &pose, mjData *d){
    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());



    for(int i = 0; i < 3; i++){
        pose.position(i) = d->xpos[(3 * bodyId) + i];
    }

    m_quat tempQuat;

    for(int i = 0; i < 4; i++){
        tempQuat(i) = d->xpos[(4 * bodyId) + i];
    }

    m_point euler = quat2Eul(tempQuat);

    for(int i = 0; i < 3; i++){
        pose.orientation(i) = euler(i);
    }

    return true;
}

bool MuJoCoHelper::getBodyPose_quat_ViaXpos(string bodyName, pose_7 &pose, mjData *d){

    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());



    for(int i = 0; i < 3; i++){
        pose.position(i) = d->xpos[(3 * bodyId) + i];
    }

    for(int i = 0; i < 4; i++){
        pose.quat(i) = d->xquat[(4 * bodyId) + i];
    }

    return true;
}

bool MuJoCoHelper::getBodyVelocity(string bodyName, pose_6 &velocity, mjData *d){
    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model->body_jntadr[bodyId];
    const int qvelIndex = model->jnt_dofadr[jointIndex];



    for(int i = 0; i < 3; i++){
        velocity.position(i) = d->qvel[qvelIndex + i];
    }

    for(int i = 0; i < 3; i++){
        velocity.orientation(i) = d->qvel[qvelIndex + 3 + i];
    }

    return true;
}

bool MuJoCoHelper::getBodyAcceleration(string bodyName, pose_6 &acceleration, mjData *d){
    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model->body_jntadr[bodyId];
    const int qvelIndex = model->jnt_dofadr[jointIndex];



    for(int i = 0; i < 3; i++){
        acceleration.position(i) = d->qacc[qvelIndex + i];
    }

    for(int i = 0; i < 3; i++){
        acceleration.orientation(i) = d->qacc[qvelIndex + 3 + i];
    }

    return true;
}
// --------------------------------- END OF BODY UTILITY ---------------------------------------

// - TODO create jacobian dynamically for the robot
Eigen::MatrixXd MuJoCoHelper::calculateJacobian(std::string bodyName, mjData *d){
    Eigen::MatrixXd kinematicJacobian(6, 7);



    //mjtNum* J_COMi_temp = mj_stackAlloc(_data, 3*_model->nv);
    Matrix<double, Dynamic, Dynamic, RowMajor> J_p(3, model->nv);
    Matrix<double, Dynamic, Dynamic, RowMajor> J_r(3, model->nv);

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());

    mj_jacBody(model, d, J_p.data(), J_r.data(), bodyId);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            kinematicJacobian(i, j) = J_p(i, j);
            //cout << kinematicJacobian(i, j) << endl;
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            kinematicJacobian(i + 3, j) = J_r(i, j);
        }
    }

    return kinematicJacobian;
}

int MuJoCoHelper::checkSystemForCollisions(mjData *d){

    // TODO(DMackRus) - check if this is needed
    mj_forward(model, d);

    int numContacts = d->ncon;
    int numCollisions = 0;

    for(int i = 0; i < numContacts; i++){
        auto contact = d->contact[i];

        int bodyInContact1 = model->body_rootid[model->geom_bodyid[contact.geom1]];
        int bodyInContact2 = model->body_rootid[model->geom_bodyid[contact.geom2]];

        // Get name of bodies in contact
        string bodyName1 = mj_id2name(model, mjOBJ_BODY, bodyInContact1);
        string bodyName2 = mj_id2name(model, mjOBJ_BODY, bodyInContact2);

        if(bodyInContact1 == 0 || bodyInContact2 == 0){

        }
        else if(bodyInContact1 == bodyInContact2){

        }
        else{
            cout << "bodies in contact: " << bodyName1 << " " << bodyName2 << endl;
            numCollisions++;
        }



    }
//    for (int i = 0; i < numContacts; i++) {
//        auto contact = d->contact[i];
//
//        // Get the ids of the two bodies in contacts
//        int bodyInContact1 = _model->body_rootid[_model->geom_bodyid[contact.geom1]];
//        int bodyInContact2 = _model->body_rootid[_model->geom_bodyid[contact.geom2]];
//
//        // only consider it a collision if robot - robot
//        // or robot - table
//
//        bool contact1Robot = false;
//        bool contact1Table = false;
//        bool contact2Robot = false;
//        bool contact2Table = false;
//        for (int j = 0; j < 11; j++) {
//            if (bodyInContact1 == robotBodyID[j]) {
//                contact1Robot = true;
//            }
//
//            if (bodyInContact2 == robotBodyID[j]) {
//                contact2Robot = true;
//            }
//        }
//
//        if (contact1Robot) {
//            if (contact2Robot || contact2Table) {
//                numCollisions++;
//            }
//        }
//        else if(contact2Robot) {
//            if (contact1Robot || contact1Table) {
//                numCollisions++;
//            }
//        }
//    }

    return numCollisions;
}

bool MuJoCoHelper::checkBodyForCollisions(string bodyName, mjData *d){

    // TODO(DMackRus) - Check if this is necessary
    mj_forward(model, d);

    int numContacts = d->ncon;

    int bodyId = mj_name2id(model, mjOBJ_BODY, bodyName.c_str());
    bool objectCollisionFound = false;

    for(int i = 0; i < numContacts; i++) {
        auto contact = d->contact[i];

        int bodyInContact1 = model->body_rootid[model->geom_bodyid[contact.geom1]];
        int bodyInContact2 = model->body_rootid[model->geom_bodyid[contact.geom2]];

        if(bodyInContact1 == bodyId && bodyInContact2 != 0){
            objectCollisionFound = true;
            break;
        }

        if(bodyInContact2 == bodyId && bodyInContact1 != 0){
            objectCollisionFound = true;
            break;
        }
    }
    return objectCollisionFound;
}

// ------------------------------- System State Functions -----------------------------------------------
bool MuJoCoHelper::appendSystemStateToEnd(mjData *d){

    savedSystemStatesList.push_back(mj_makeData(model));
    fp_rollout_data.push_back(mj_makeData(model));

    cpMjData(model, savedSystemStatesList.back(), d);
    cpMjData(model, fp_rollout_data.back(), d);

    return true;
}

bool MuJoCoHelper::checkIfDataIndexExists(int list_index){
    return (savedSystemStatesList.size() > list_index);
}

bool MuJoCoHelper::copySystemState(mjData *d_dest, mjData *d_src){

    cpMjData(model, d_dest, d_src);

    return true;
}

bool MuJoCoHelper::deleteSystemStateFromIndex(int listIndex){
    mj_deleteData(savedSystemStatesList[listIndex]);
    savedSystemStatesList.erase(savedSystemStatesList.begin() + listIndex);

    return true;
}

bool MuJoCoHelper::clearSystemStateList(){
    for(int i = 0; i < savedSystemStatesList.size(); i++){
        mj_deleteData(savedSystemStatesList[i]);
    }
    savedSystemStatesList.clear();

    return true;
}

void MuJoCoHelper::cpMjData(const mjModel* m, mjData* d_dest, mjData* d_src){
    d_dest->time = d_src->time;
    mju_copy(d_dest->qpos, d_src->qpos, m->nq);
    mju_copy(d_dest->qvel, d_src->qvel, m->nv);
    mju_copy(d_dest->qacc, d_src->qacc, m->nv);
    mju_copy(d_dest->qacc_warmstart, d_src->qacc_warmstart, m->nv);
    mju_copy(d_dest->qfrc_applied, d_src->qfrc_applied, m->nv);
    mju_copy(d_dest->xfrc_applied, d_src->xfrc_applied, 6*m->nbody);
    mju_copy(d_dest->ctrl, d_src->ctrl, m->nu);
}

void MuJoCoHelper::saveDataToRolloutBuffer(mjData *d, int rolloutIndex){

    cpMjData(model, fp_rollout_data[rolloutIndex], d);
}

void MuJoCoHelper::copyRolloutBufferToSavedSystemStatesList(){
    for(int i = 1; i < fp_rollout_data.size(); i++){
        cpMjData(model, savedSystemStatesList[i], fp_rollout_data[i]);
    }
}

// ------------------------------- END OF SYSTEM STATE FUNCTIONS ----------------------------------------

//bool MuJoCoHelper::stepSimulator(int steps, mjData *d){
//
//
//
//    for(int i = 0; i < steps; i++){
//        mj_step(model, d);
//    }
//    return true;
//}

bool MuJoCoHelper::forwardSimulator(mjData *d){

    mj_forward(model, d);

    return true;
}

bool MuJoCoHelper::forwardSimulatorWithSkip(mjData *d, int skipStage, int skipSensor){

        mj_forwardSkip(model, d, skipStage, skipSensor);

        return true;
}

// --------------------------------- Visualization Functions ---------------------------------------
void MuJoCoHelper::initVisualisation() {

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    //mjv_defaultPerturb(&pert);				// what data type for pert?
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);
    mjv_defaultScene(&scn);

//    cam.distance = 1.66269;
//    cam.azimuth = -118.7;
//    cam.elevation = -34.7;
//    cam.lookat[0] = 0.4027;
//    cam.lookat[1] = 0.0169;
//    cam.lookat[2] = 0.1067;

      // Maybe push in clutter?
//    cam.distance = 1.66269;
//    cam.distance = 0.4449;
//    cam.azimuth = 133.7;
//    cam.elevation = -31.9;
//    cam.lookat[0] = 0.635;
//    cam.lookat[1] = -0.03;
//    cam.lookat[2] = 0.275;

    // Walker
//    cam.distance = 5.79918;
//    cam.azimuth = -117.9;
//    cam.elevation = -20.9;
//    cam.lookat[0] = 1.81681;
//    cam.lookat[1] = -0.380067;
//    cam.lookat[2] =  0.669554;

    // BoxSweep
//    cam.distance = 1.16;
//    cam.azimuth = -128.1;
//    cam.elevation = -21.3;
//    cam.lookat[0] = 0.601;
//    cam.lookat[1] = 0.209;
//    cam.lookat[2] =  0.219;

    // Push heavy clutter
    cam.distance = 1.16;
    cam.azimuth = 146.1;
    cam.elevation = -42.9;
    cam.lookat[0] = 0.535744;
    cam.lookat[1] = 0.155588;
    cam.lookat[2] =  0.236069;

    // create scene and context
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);
}

void MuJoCoHelper::updateScene(GLFWwindow *window, const char* label){

    // update scene and render
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjv_updateScene(model, vis_data, &opt, NULL, &cam, mjCAT_ALL, &scn);

    mjr_render(viewport, &scn, &con);

//    cout << "------------------------------------------------- \n";
//    cout << "camera dist: " << cam.distance << endl;
//    cout << "camera azimuth: " << cam.azimuth << endl;
//    cout << "camera elevation: " << cam.elevation << endl;
//    cout << "camera look at: " << cam.lookat[0] << endl;
//    cout << "camera look at: " << cam.lookat[1] << endl;
//    cout << "camera look at: " << cam.lookat[2] << endl;

    mjrRect rect{0, 0, 100, 100};
    mjr_rectangle(rect, 0, 0, 0, 0);

    mjr_overlay(0, mjGRID_TOPLEFT, rect, label, 0, &con);
}

void MuJoCoHelper::mouseMove(double dx, double dy, bool button_left, bool button_right,  GLFWwindow *window){
// get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    //determine action based on mouse button
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

void MuJoCoHelper::scroll(double yoffset){
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// --------------------------------- END OF VISUALIZATION FUNCTIONS ---------------------------------------

void MuJoCoHelper::initSimulator(double timestep, const char* fileName){
    char error[1000];
     cout << "fileName in init: " << fileName << endl;
    auto load_start = std::chrono::high_resolution_clock::now();
    model = mj_loadXML(fileName, NULL, error, 1000);

    if( !model ) {
        printf("%s\n", error);
    }

    // defaults 100 , 1e-8
    model->opt.timestep = timestep;
//    model->opt.iterations = 30;
//    model->opt.tolerance = 1e-1;
    cout << "model iterations: " << model->opt.iterations << endl;
    cout << "model tolerance : " << model->opt.tolerance << endl;

    cout << "model nq: " << model->nq << endl;
    cout << "model nv: " << model->nv << endl;
    cout << "model nu: " << model->nu << endl;
    cout << "model nbody: " << model->nbody << endl;
//    cout << "model memory: " << model->memory

//    for(int i = 0; i < model->nu; i++){
//        cout << "model ctrlrange: " << model->actuator_ctrlrange[2*i] << endl;
//        cout << "model ctrlrange: " << model->actuator_ctrlrange[2*i+1] << endl;
//    }
//
//    std::string names[6] = {"right_hip", "right_knee", "right_ankle", "left_hip", "left_knee", "left_ankle"};
//
////    int id = mj_name2id(model, mjOBJ_BODY, "right_hip");
//    for(int i = 0; i < 6; i++){
//        int id = mj_name2id(model, mjOBJ_ACTUATOR, names[i].c_str());
//        cout << "actuator id: " << id << endl;
//    }

    // make data corresponding to model
    main_data = mj_makeData(model);
    master_reset_data = mj_makeData(model);
    vis_data = mj_makeData(model);

    // Get the number of available cores
    int numCores = std::thread::hardware_concurrency();
    for(int i = 0; i < numCores; i++){
        fd_data.push_back(mj_makeData(model));
    }
    std::cout << "time to load and make data: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - load_start).count() << "ms" << std::endl;
}

void MuJoCoHelper::initModelForFiniteDifferencing(){
    save_iterations = model->opt.iterations;
    save_tolerance = model->opt.tolerance;
    // This used to be 50, 3 seems to be the lowest i can set it without breaking the simulation
    model->opt.iterations = 5;
    model->opt.tolerance = 0;
}

void MuJoCoHelper::resetModelAfterFiniteDifferencing(){
    model->opt.iterations = save_iterations;
    model->opt.tolerance = save_tolerance;

}

double MuJoCoHelper::returnModelTimeStep() {
    return model->opt.timestep;
}

double* MuJoCoHelper::sensorState(mjData *d, std::string sensorName){

    int id = mj_name2id(model, mjOBJ_SENSOR, sensorName.c_str());
    if (id == -1) {
        std::cerr << "sensor \"" << sensorName << "\" not found.\n";
        return nullptr;
    } else {
        return d->sensordata + model->sensor_adr[id];
    }
}

void MuJoCoHelper::_mjdTransitionFD(){
    std::cout << "start? \n";

    mjModel *m;
//    m = mj_loadXML("/home/davidrussell/catkin_ws/src/TrajOptKP/mujoco_models/walker/walker_plane.xml", NULL, NULL, 1000);
    m = mj_loadXML("/home/davidrussell/catkin_ws/src/TrajOptKP/mujoco_models/Franka_emika_scenes_V1/cylinder_pushing.xml", NULL, NULL, 1000);
    mjData *d = mj_makeData(m);
    d->qpos[1] = 0.1;
    mj_kinematics(m, d);

    std::string EE_name = "franka_gripper";
    int EE_id = mj_name2id(m, mjOBJ_BODY, EE_name.c_str());

    std::cout << "EE_id: " << EE_id << "\n";

//    mjtNum body_pos = d->xpos[3 * EE_id];
    mjtNum body_pos[3];
    for (int i = 0; i < 3; ++i) {
        body_pos[i] = d->xpos[(3 * EE_id) + i];
    }

    // Print out the position
    std::cout << "Body Position: (" << body_pos[0] << ", " << body_pos[1] << ", " << body_pos[2] << ")" << std::endl;



//    int T = 1000;
//    int dof = m->nv;
//    int num_ctrl = m->nu;
//    int num_fd = ((dof * 2) + num_ctrl) * 2 * T;
//    std::cout << "num_fd: " << num_fd << "\n";
//
//    auto time_start = std::chrono::high_resolution_clock::now();
//    for(int i = 0; i < T; i++){
//        mj_step(m, d);
//    }
//
//    std::cout << "pretend rollout took: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_start).count() << "ms\n";
//
//    time_start = std::chrono::high_resolution_clock::now();
//    for(int i = 0; i < num_fd; i++){
//        mj_forward(m, d);
//    }
//
//    std::cout << "finite differencing took: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_start).count() << "ms\n";
}
