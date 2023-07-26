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
bool MuJoCoHelper::setRobotJointsPositions(string robotName, vector<double> jointPositions, int dataIndex){

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
    int jointId = mj_name2id(model.get(), mjOBJ_JOINT, robotBaseJointName.c_str());

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }

    
    int startIndex = model.get()->jnt_qposadr[jointId];

    if(startIndex == MAIN_DATA_STATE){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    for(int i = 0; i < jointPositions.size(); i++){
        d->qpos[startIndex + i] = jointPositions[i];
    }

    return true;
}

// Sets a robot joint velocities the given values
bool MuJoCoHelper::setRobotJointsVelocities(string robotName, vector<double> jointVelocities, int dataIndex){

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
    int jointId = mj_name2id(model.get(), mjOBJ_JOINT, robotBaseJointName.c_str());

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }
    int startIndex = model.get()->jnt_dofadr[jointId];

    if(startIndex == -1){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    for(int i = 0; i < jointVelocities.size(); i++){
        d->qvel[startIndex + i] = jointVelocities[i];
    }

    return true;
}

bool MuJoCoHelper::setRobotJointsControls(string robotName, vector<double> jointControls, int dataIndex){

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
    int jointId = mj_name2id(model.get(), mjOBJ_ACTUATOR, robotBaseJointName.c_str());

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }
    int startIndex = model.get()->jnt_dofadr[jointId];

    if(startIndex == -1){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    for(int i = 0; i < jointControls.size(); i++){
        d->ctrl[startIndex + i] = jointControls[i];
    }

    return true;
}

bool MuJoCoHelper::getRobotJointsPositions(string robotName, vector<double> &jointPositions, int dataIndex){

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(!isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }

    // Get the body id of the base link of the robot
    int jointId = mj_name2id(model.get(), mjOBJ_JOINT, robotBaseJointName.c_str());

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }
    int startIndex = model.get()->jnt_dofadr[jointId];

    if(startIndex == -1){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    for(int i = 0; i < robots[robotIndex].jointNames.size(); i++){
        jointPositions.push_back(d->qpos[startIndex + i]);
    }

    return true;
}

bool MuJoCoHelper::getRobotJointsVelocities(string robotName, vector<double> &jointVelocities, int dataIndex) {

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(!isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }

    // Get the body id of the base link of the robot
    int jointId = mj_name2id(model.get(), mjOBJ_JOINT, robotBaseJointName.c_str());

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }
    int startIndex = model.get()->jnt_dofadr[jointId];

    if(startIndex == -1){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    for(int i = 0; i < robots[robotIndex].jointNames.size(); i++){
        jointVelocities.push_back(d->qvel[startIndex + i]);
    }

    return true;
}

bool MuJoCoHelper::getRobotJointsAccelerations(string robotName, vector<double> &jointAccelerations, int dataIndex){
    int robotIndex;
    string robotBaseJointName;
    if(!isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }

    // Get the body id of the base link of the robot
    int jointId = mj_name2id(model.get(), mjOBJ_JOINT, robotBaseJointName.c_str());

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }
    int startIndex = model.get()->jnt_dofadr[jointId];

    if(startIndex == -1){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    for(int i = 0; i < robots[robotIndex].jointNames.size(); i++){
        jointAccelerations.push_back(d->qacc[startIndex + i]);
    }

    return true;
}

bool MuJoCoHelper::getRobotJointsControls(string robotName, vector<double> &jointControls, int dataIndex) {

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(!isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }

    // Get the body id of the base link of the robot
    int jointId = mj_name2id(model.get(), mjOBJ_ACTUATOR, robotBaseJointName.c_str());

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }
    int startIndex = model.get()->jnt_dofadr[jointId];

    if(startIndex == -1){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    for(int i = 0; i < robots[robotIndex].jointNames.size(); i++){
        jointControls.push_back(d->ctrl[startIndex + i]);
    }

    return true;
}

bool MuJoCoHelper::getRobotJointsGravityCompensaionControls(string robotName, vector<double> &jointsControls, int dataIndex){

    // Check if the robot exists in the simulation
    int robotIndex;
    string robotBaseJointName;
    if(!isValidRobotName(robotName, robotIndex, robotBaseJointName)){
        cout << "That robot doesnt exist in the simulation\n";
        return false;
    }

    // Get the body id of the base link of the robot
    int jointId = mj_name2id(model.get(), mjOBJ_JOINT, robotBaseJointName.c_str());

    if(jointId == -1){
        cout << "Base link of robot not found\n";
        return false;
    }
    int startIndex = model.get()->jnt_dofadr[jointId];

    if(startIndex == -1){
        cout << "Invalid bodyId for robot\n";
        return false;
    }

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);
    mj_forward(model.get(), d.get());

    for(int i = 0; i < robots[robotIndex].jointNames.size(); i++){
        jointsControls.push_back(d->qfrc_bias[startIndex + i]);
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
    // return false;
    return true;
}

bool MuJoCoHelper::setBodyPose_quat(string bodyName, pose_7 pose, int dataIndex){

    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model.get(), mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model.get()->body_jntadr[bodyId];
    const int qposIndex = model.get()->jnt_qposadr[jointIndex];

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    for(int i = 0; i < 3; i++){
        d->qpos[qposIndex + i] = pose.position(i);
    }

    for(int i = 0; i < 4; i++){
        d->qpos[qposIndex + 3 + i] = pose.quat(i);
    }

    return true;
}

bool MuJoCoHelper::setBodyPose_angle(string bodyName, pose_6 pose, int dataIndex){

    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation: " << bodyName << "\n";
        return false;
    }

    int bodyId = mj_name2id(model.get(), mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model.get()->body_jntadr[bodyId];
    const int qposIndex = model.get()->jnt_qposadr[jointIndex];

    m_quat q = eul2Quat(pose.orientation);

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

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

bool MuJoCoHelper::setBodyVelocity(string bodyName, pose_6 velocity, int dataIndex){

    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model.get(), mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model.get()->body_jntadr[bodyId];
    const int qvelIndex = model.get()->jnt_dofadr[jointIndex];

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    for(int i = 0; i < 3; i++){
        d->qvel[qvelIndex + i] = velocity.position(i);
    }

    for(int i = 0; i < 3; i++){
        d->qvel[qvelIndex + 3 + i] = velocity.orientation(i);
    }

    return true;
}

bool MuJoCoHelper::getBodyPose_quat(string bodyName, pose_7 &pose, int dataIndex){
    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model.get(), mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model.get()->body_jntadr[bodyId];
    const int qposIndex = model.get()->jnt_qposadr[jointIndex];

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    for(int i = 0; i < 3; i++){
        pose.position(i) = d->xpos[(bodyId * 3) + i];
    }

    for(int i = 0; i < 4; i++){
        pose.quat(i) = d->xquat[(bodyId * 4) + i];
    }
//    for(int i = 0; i < 3; i++){
//        pose.position(i) = d->qpos[qposIndex + i];
//    }
//
//    for(int i = 0; i < 4; i++){
//        pose.quat(i) = d->qpos[qposIndex + 3 + i];
//    }

    return true;
}

bool MuJoCoHelper::getBodyPose_angle(string bodyName, pose_6 &pose, int dataIndex){
    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation " << bodyName << endl;
        return false;
    }

    int bodyId = mj_name2id(model.get(), mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model.get()->body_jntadr[bodyId];
    const int qposIndex = model.get()->jnt_qposadr[jointIndex];

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    for(int i = 0; i < 3; i++){
        pose.position(i) = d->xpos[(bodyId * 3) + i];
    }
//    for(int i = 0; i < 3; i++){
//        pose.position(i) = d->qpos[qposIndex + i];
//    }

    m_quat tempQuat;

    for(int i = 0; i < 4; i++){
        tempQuat(i) = d->xquat[(bodyId * 4) + i];
    }
//    for(int i = 0; i < 4; i++){
//        tempQuat(i) = d->qpos[qposIndex + 3 + i];
//    }

    m_point euler = quat2Eul(tempQuat);

    for(int i = 0; i < 3; i++){
        pose.orientation(i) = euler(i);
    }

    return true;
}

bool MuJoCoHelper::getBodyVelocity(string bodyName, pose_6 &velocity, int dataIndex){
    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model.get(), mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model.get()->body_jntadr[bodyId];
    const int qvelIndex = model.get()->jnt_dofadr[jointIndex];

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    for(int i = 0; i < 3; i++){
        velocity.position(i) = d->qvel[qvelIndex + i];
    }

    for(int i = 0; i < 3; i++){
        velocity.orientation(i) = d->qvel[qvelIndex + 3 + i];
    }

    return true;
}

bool MuJoCoHelper::getBodyAcceleration(string bodyName, pose_6 &acceleration, int dataIndex){
    int bodyIndex;
    if(!isValidBodyName(bodyName, bodyIndex)){
        cout << "That body doesnt exist in the simulation\n";
        return false;
    }

    int bodyId = mj_name2id(model.get(), mjOBJ_BODY, bodyName.c_str());
    const int jointIndex = model.get()->body_jntadr[bodyId];
    const int qvelIndex = model.get()->jnt_dofadr[jointIndex];

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

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
Eigen::MatrixXd MuJoCoHelper::calculateJacobian(std::string bodyName, int dataIndex){
    Eigen::MatrixXd kinematicJacobian(6, 7);

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    //mjtNum* J_COMi_temp = mj_stackAlloc(_data, 3*_model.get()->nv);
    Matrix<double, Dynamic, Dynamic, RowMajor> J_p(3, model.get()->nv);
    Matrix<double, Dynamic, Dynamic, RowMajor> J_r(3, model.get()->nv);

    int bodyId = mj_name2id(model.get(), mjOBJ_BODY, bodyName.c_str());

    mj_jacBody(model.get(), d.get(), J_p.data(), J_r.data(), bodyId);

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

int MuJoCoHelper::checkSystemForCollisions(int dataIndex){

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);
    mj_forward(model.get(), d.get());

    int numContacts = d->ncon;
    int numCollisions = 0;

    for(int i = 0; i < numContacts; i++){
        auto contact = d->contact[i];

        int bodyInContact1 = model.get()->body_rootid[model.get()->geom_bodyid[contact.geom1]];
        int bodyInContact2 = model.get()->body_rootid[model.get()->geom_bodyid[contact.geom2]];

        // Get name of bodies in contact
        string bodyName1 = mj_id2name(model.get(), mjOBJ_BODY, bodyInContact1);
        string bodyName2 = mj_id2name(model.get(), mjOBJ_BODY, bodyInContact2);

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
//        int bodyInContact1 = _model.get()->body_rootid[_model.get()->geom_bodyid[contact.geom1]];
//        int bodyInContact2 = _model.get()->body_rootid[_model.get()->geom_bodyid[contact.geom2]];
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

bool MuJoCoHelper::checkBodyForCollisions(string bodyName, int dataIndex){
    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);
    mj_forward(model.get(), d.get());

    int numContacts = d->ncon;
    // id to name
//    std::string name = mj_id2name(model.get(), mjOBJ_BODY, 10);

    int bodyId = mj_name2id(model.get(), mjOBJ_BODY, bodyName.c_str());
    bool objectCollisionFound = false;

    for(int i = 0; i < numContacts; i++) {
        auto contact = d->contact[i];

        int bodyInContact1 = model.get()->body_rootid[model.get()->geom_bodyid[contact.geom1]];
        int bodyInContact2 = model.get()->body_rootid[model.get()->geom_bodyid[contact.geom2]];

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
bool MuJoCoHelper::appendSystemStateToEnd(int dataIndex){

    auto saveData = returnDesiredDataState(dataIndex);

//    mjData *d = mj_makeData(model.get());
//    auto d_unique = std::shared_ptr<mjData>(d);

//    cpMjData(model, d_unique, saveData);

//    mj_forward(model.get(), d_unique.get());
    savedSystemStatesList.push_back(std::shared_ptr<mjData>(mj_makeData(model.get())));
    cpMjData(model, savedSystemStatesList.back(), saveData);

    return true;
}

bool MuJoCoHelper::checkIfDataIndexExists(int dataIndex){
    return (savedSystemStatesList.size() > dataIndex);
}

bool MuJoCoHelper::copySystemState(int dataDestinationIndex, int dataSourceIndex){

    std::shared_ptr<mjData> dataDestination = returnDesiredDataState(dataDestinationIndex);

    std::shared_ptr<mjData> dataSource = returnDesiredDataState(dataSourceIndex);

    cpMjData(model, dataDestination, dataSource);
//    mj_forward(model.get(), dataDestination.get());

    return true;
}

bool MuJoCoHelper::deleteSystemStateFromIndex(int listIndex){
    mj_deleteData(savedSystemStatesList[listIndex].get());
    savedSystemStatesList.erase(savedSystemStatesList.begin() + listIndex);

    return true;
}

bool MuJoCoHelper::clearSystemStateList(){
    for(int i = 0; i < savedSystemStatesList.size(); i++){
        mj_deleteData(savedSystemStatesList[i].get());
    }
    savedSystemStatesList.clear();

    return true;
}

void MuJoCoHelper::cpMjData(const std::shared_ptr<mjModel> m, std::shared_ptr<mjData> d_dest, const std::shared_ptr<mjData> d_src){
    d_dest->time = d_src->time;
    mju_copy(d_dest->qpos, d_src->qpos, m->nq);
    mju_copy(d_dest->qvel, d_src->qvel, m->nv);
    mju_copy(d_dest->qacc, d_src->qacc, m->nv);
    mju_copy(d_dest->qacc_warmstart, d_src->qacc_warmstart, m->nv);
    mju_copy(d_dest->qfrc_applied, d_src->qfrc_applied, m->nv);
    mju_copy(d_dest->xfrc_applied, d_src->xfrc_applied, 6*m->nbody);
    mju_copy(d_dest->ctrl, d_src->ctrl, m->nu);
}

std::shared_ptr<mjData> MuJoCoHelper::returnDesiredDataState(int dataIndex){
    if(dataIndex == MAIN_DATA_STATE){
        return mdata;
    }
    else if(dataIndex == MASTER_RESET_DATA){
        return d_master_reset;
    }
    else if(dataIndex < MASTER_RESET_DATA){
        int fd_id = -dataIndex - 3;
        return fd_data[fd_id];
    }
    else{
        return savedSystemStatesList[dataIndex];
    }
}
// ------------------------------- END OF SYSTEM STATE FUNCTIONS ----------------------------------------

bool MuJoCoHelper::stepSimulator(int steps, int dataIndex){

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    for(int i = 0; i < steps; i++){
        mj_step(model.get(), d.get());
    }
    return true;
}

// TODO - this is a bit hardcoded, make it less so
bool MuJoCoHelper::forwardSimulator(int dataIndex){

    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

//    mj_forwardSkip(model.get(), d.get(), mjSTAGE_NONE, 1);
    mj_forward(model.get(), d.get());

    return true;
}

bool MuJoCoHelper::forwardSimulatorWithSkip(int dataIndex, int skipStage, int skipSensor){

        std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

        mjtNum skipStages[3] = {mjSTAGE_NONE, mjSTAGE_VEL, mjSTAGE_POS};

        mj_forwardSkip(model.get(), d.get(), skipStages[skipStage], skipSensor);

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
    cam.azimuth = -118.7;
    cam.elevation = 3;
    cam.elevation = -34.7;
    cam.lookat[0] = 0.4027;
    cam.lookat[1] = 0.0169;
    cam.lookat[2] = 0.1067;

    // create scene and context
    mjv_makeScene(model.get(), &scn, 2000);
    mjr_makeContext(model.get(), &con, mjFONTSCALE_150);
}

void MuJoCoHelper::updateScene(GLFWwindow *window, const char* label){
    // update scene and render
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjv_updateScene(model.get(), mdata.get(), &opt, NULL, &cam, mjCAT_ALL, &scn);

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
    mjv_moveCamera(model.get(), action, dx / height, dy / height, &scn, &cam);
}

void MuJoCoHelper::scroll(double yoffset){
    mjv_moveCamera(model.get(), mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// --------------------------------- END OF VISUALIZATION FUNCTIONS ---------------------------------------

void MuJoCoHelper::initSimulator(double timestep, const char* fileName){
    char error[1000];
    // cout << "fileName in init: " << fileName << endl;
    model = shared_ptr<mjModel>(mj_loadXML(fileName, NULL, error, 1000));

    if( !model ) {
        printf("%s\n", error);
    }

    model->opt.timestep = timestep;
//    model->opt.gravity[2] = -2;
//    model->opt.iterations = 30;
//    model->opt.tolerance = 1e-1;
//    cout << "model iterations: " << model->opt.iterations << endl;
//    cout << "model tolerance : " << model->opt.tolerance << endl;

    cout << "model nq: " << model->nq << endl;
    cout << "model nv: " << model->nv << endl;
    cout << "model nu: " << model->nu << endl;
    cout << "model nbody: " << model->nbody << endl;

    for(int i = 0; i < model->nu; i++){
        cout << "model ctrlrange: " << model->actuator_ctrlrange[2*i] << endl;
        cout << "model ctrlrange: " << model->actuator_ctrlrange[2*i+1] << endl;
    }

    std::string names[6] = {"right_hip", "right_knee", "right_ankle", "left_hip", "left_knee", "left_ankle"};

//    int id = mj_name2id(model.get(), mjOBJ_BODY, "right_hip");
    for(int i = 0; i < 6; i++){
        int id = mj_name2id(model.get(), mjOBJ_ACTUATOR, names[i].c_str());
        cout << "actuator id: " << id << endl;
    }

    // make data corresponding to model.get()
    mdata = shared_ptr<mjData>(mj_makeData(model.get()));
    d_master_reset = shared_ptr<mjData>(mj_makeData(model.get()));

    // Get the number of available cores
    int numCores = std::thread::hardware_concurrency();
    for(int i = 0; i < numCores; i++){
        fd_data.push_back(shared_ptr<mjData>(mj_makeData(model.get())));
    }
}

void MuJoCoHelper::initModelForFiniteDifferencing(){
    save_iterations = model->opt.iterations;
    save_tolerance = model->opt.tolerance;
    model->opt.iterations = 30;
    model->opt.tolerance = 0;
}

void MuJoCoHelper::resetModelAfterFiniteDifferencing(){
    model->opt.iterations = save_iterations;
    model->opt.tolerance = save_tolerance;

}

double* MuJoCoHelper::sensorState(int dataIndex, std::string sensorName){
    std::shared_ptr<mjData> d = returnDesiredDataState(dataIndex);

    int id = mj_name2id(model.get(), mjOBJ_SENSOR, sensorName.c_str());
    if (id == -1) {
        std::cerr << "sensor \"" << sensorName << "\" not found.\n";
        return nullptr;
    } else {
        return d->sensordata + model->sensor_adr[id];
    }
}
