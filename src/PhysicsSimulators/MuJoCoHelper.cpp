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
bool MuJoCoHelper::IsValidRobotName(const string& robot_name, int &robot_index, string &robot_base_joint_name){
    bool valid_robot = false;
    for(int i = 0; i < robots.size(); i++){
        if(robots[i].name == robot_name){
            valid_robot = true;
            robot_base_joint_name = robots[i].joint_names[0];
            robot_index = i;
        }
    }

    return valid_robot;
}

// Sets a robot joint positions the given values
void MuJoCoHelper::SetRobotJointPositions(const string& robot_name, vector<double> joint_positions, mjData *d){

    // Check if the robot exists in the simulation
    int robot_index;
    string robot_base_joint_name;
    if(IsValidRobotName(robot_name, robot_index, robot_base_joint_name)){
        if(joint_positions.size() != robots[robot_index].joint_names.size()){
            std::cerr << "Invalid number of joint positions\n";
            exit(1);
        }
    }
    else{
        std::cerr << "That robot doesnt exist in the simulation\n";
        exit(1);
    }

    for(int i = 0; i < joint_positions.size(); i++){
        int joint_id = mj_name2id(model, mjOBJ_JOINT, robots[robot_index].joint_names[i].c_str());
        if(joint_id == -1){
            std::cerr << "Invalid bodyId for robot: " << robots[robot_index].joint_names[i].c_str() << "\n";
            exit(1);
        }
        int q_pos_index = model->jnt_qposadr[joint_id];
        d->qpos[q_pos_index] = joint_positions[i];
    }
}

// Sets a robot joint velocities the given values
void MuJoCoHelper::SetRobotJointsVelocities(const string& robot_name, vector<double> joint_velocities, mjData *d){

    // Check if the robot exists in the simulation
    int robot_index;
    string robot_base_joint_name;
    if(IsValidRobotName(robot_name, robot_index, robot_base_joint_name)){
        if(joint_velocities.size() != robots[robot_index].joint_names.size()){
            std::cerr << "Invalid number of joint positions\n";
            exit(1);
        }
    }
    else{
        std::cerr << "That robot doesnt exist in the simulation\n";
        exit(1);
    }

    for(int i = 0; i < joint_velocities.size(); i++){
        int joint_id = mj_name2id(model, mjOBJ_JOINT, robots[robot_index].joint_names[i].c_str());
        if(joint_id == -1){
            std::cerr << "Invalid bodyId for robot: " << robots[robot_index].joint_names[i].c_str() << "\n";
            exit(1);
        }
        int dof_index = model->jnt_dofadr[joint_id];
        d->qvel[dof_index] = joint_velocities[i];
    }
}

void MuJoCoHelper::SetRobotJointsControls(const string& robot_name, vector<double> joint_controls, mjData *d){

    // Check if the robot exists in the simulation
    int robot_index;
    string robot_base_joint_name;
    if(IsValidRobotName(robot_name, robot_index, robot_base_joint_name)){
        if(joint_controls.size() != robots[robot_index].actuator_names.size()){
            std::cerr << "Invalid number of joint positions\n";
            exit(1);
        }
    }
    else{
        std::cerr << "That robot doesnt exist in the simulation\n";
        exit(1);
    }

    for(int i = 0; i < joint_controls.size(); i++){
        int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, robots[robot_index].actuator_names[i].c_str());
        d->ctrl[actuator_id] = joint_controls[i];
    }
}

void MuJoCoHelper::GetRobotJointsPositions(const string& robot_name, vector<double> &joint_positions, mjData *d){
    int robot_index;
    string robot_base_joint_name;

    if(!IsValidRobotName(robot_name, robot_index, robot_base_joint_name)){
        std::cerr << "That robot doesnt exist in the simulation\n";
        exit(1);
    }

    joint_positions.resize(robots[robot_index].joint_names.size());

    for(int i = 0; i < robots[robot_index].joint_names.size(); i++){
        int joint_id = mj_name2id(model, mjOBJ_JOINT, robots[robot_index].joint_names[i].c_str());
        if(joint_id == -1){
            std::cerr << "Invalid bodyId for robot: " << robots[robot_index].joint_names[i].c_str() << "\n";
            exit(1);
        }
        int qpos_index = model->jnt_qposadr[joint_id];
        joint_positions[i] = d->qpos[qpos_index];
    }
}

void MuJoCoHelper::GetRobotJointsVelocities(const string& robot_name, vector<double> &joint_velocities, mjData *d) {
    int robot_index;
    string robot_base_joint_name;

    if(!IsValidRobotName(robot_name, robot_index, robot_base_joint_name)){
        std::cerr << "That robot doesnt exist in the simulation\n";
        exit(1);
    }

    joint_velocities.resize(robots[robot_index].joint_names.size());

    for(int i = 0; i < robots[robot_index].joint_names.size(); i++){
        int joint_id = mj_name2id(model, mjOBJ_JOINT, robots[robot_index].joint_names[i].c_str());
        if(joint_id == -1){
            std::cerr << "Invalid bodyId for robot: " << robots[robot_index].joint_names[i].c_str() << "\n";
            exit(1);
        }
        int dof_index = model->jnt_dofadr[joint_id];
        joint_velocities[i] = d->qvel[dof_index];
    }
}

void MuJoCoHelper::GetRobotJointsAccelerations(const string& robot_name, vector<double> &joint_accelerations, mjData *d){
    int robot_index;
    string robot_base_joint_name;

    if(!IsValidRobotName(robot_name, robot_index, robot_base_joint_name)){
        std::cerr << "That robot doesnt exist in the simulation\n";
        exit(1);
    }

    joint_accelerations.resize(robots[robot_index].joint_names.size());

    // Get the body id of the base link of the robot
    int jointId = mj_name2id(model, mjOBJ_JOINT, robot_base_joint_name.c_str());

    if(jointId == -1){
        std::cerr << "Base link of robot not found\n";
        exit(1);
    }
    int startIndex = model->jnt_dofadr[jointId];

    if(startIndex == -1){
        std::cerr << "Invalid bodyId for robot\n";
        exit(1);
    }

    for(int i = 0; i < robots[robot_index].joint_names.size(); i++){
        joint_accelerations[i] = d->qacc[startIndex + i];
    }
}

void MuJoCoHelper::GetRobotJointsControls(const string& robot_name, vector<double> &joint_controls, mjData *d) {

    // Check if the robot exists in the simulation
    int robot_index = 0;

    joint_controls.resize(robots[robot_index].actuator_names.size());

    for(int i = 0; i < robots[robot_index].actuator_names.size(); i++){
        int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, robots[robot_index].actuator_names[i].c_str());
        joint_controls[i] = (d->ctrl[actuator_id]);
    }
}

void MuJoCoHelper::GetRobotJointsGravityCompensationControls(const string& robot_name, vector<double> &joint_controls, mjData *d){
    int robot_index;
    string robot_base_joint_name;

    if(!IsValidRobotName(robot_name, robot_index, robot_base_joint_name)){
        std::cerr << "That robot doesnt exist in the simulation\n";
        exit(1);
    }

    // Get the body id of the base link of the robot
    int joint_id = mj_name2id(model, mjOBJ_JOINT, robot_base_joint_name.c_str());

    joint_controls.resize(robots[robot_index].joint_names.size());

    if(joint_id == -1){
        std::cerr << "Base link of robot not found\n";
        exit(1);
    }

    int start_index = model->jnt_dofadr[joint_id];

    if(start_index == -1){
        std::cerr << "Invalid bodyId for robot\n";
        exit(1);
    }

    // TODO (DMackRus) - Check if this is needed?
    mj_forward(model, d);

    for(int i = 0; i < robots[robot_index].joint_names.size(); i++){
        joint_controls[i] = d->qfrc_bias[start_index + i];
    }
}

void MuJoCoHelper::GetRobotControlLimits(const string& robot_name, vector<double> &control_limits){

    // Check if the robot exists in the simulation
    int robot_index;
    string robot_base_joint_name;
    if(!IsValidRobotName(robot_name, robot_index, robot_base_joint_name)){
        std::cerr << "That robot doesnt exist in the simulation\n";
        exit(1);
    }

    // Get the body id of the base link of the robot
    int joint_id = mj_name2id(model, mjOBJ_JOINT, robot_base_joint_name.c_str());

    control_limits.resize(2 * robots[robot_index].actuator_names.size());

    if(joint_id == -1){
        std::cerr << "Base link of robot not found\n";
        exit(1);
    }

    int start_index = model->jnt_dofadr[joint_id];

    if(start_index == -1){
        std::cerr << "Invalid bodyId for robot\n";
        exit(1);
    }

    for(int i = 0; i < 2 * robots[robot_index].actuator_names.size(); i++){
        control_limits[i] = model->actuator_ctrlrange[i];
    }
}

void MuJoCoHelper::GetRobotJointLimits(const string& robot_name, vector<double> &joint_limits, mjData *d){

}

// --------------------------------- END OF ROBOT UTILITY ---------------------------------------

// ------------------------------------- BODY UTILITY -------------------------------------------

bool MuJoCoHelper::BodyExists(const string& body_name, int &body_index){
    bool body_exists = false;

    body_index = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    if(body_index != -1){
        body_exists = true;
    }

    return body_exists;
}

void MuJoCoHelper::SetBodyColor(const string& body_name, const float color[4]) const{
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    int geom_id = model->body_geomadr[body_id];

    model->geom_rgba[geom_id * 4]     = color[0]; // Red
    model->geom_rgba[geom_id * 4 + 1] = color[1]; // Green
    model->geom_rgba[geom_id * 4 + 2] = color[2]; // Blue
    model->geom_rgba[geom_id * 4 + 3] = color[3]; // Alpha
}

void MuJoCoHelper::SetBodyPoseQuat(const string& body_name, pose_7 pose, mjData *d) const{
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    const int joint_index = model->body_jntadr[body_id];
    const int qpos_index = model->jnt_qposadr[joint_index];

    for(int i = 0; i < 3; i++){
        d->qpos[qpos_index + i] = pose.position(i);
    }

    for(int i = 0; i < 4; i++){
        d->qpos[qpos_index + 3 + i] = pose.quat(i);
    }
}

void MuJoCoHelper::SetBodyPoseAngle(const string& body_name, pose_6 pose, mjData *d) const{
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    const int joint_index = model->body_jntadr[body_id];
    const int qpos_index = model->jnt_qposadr[joint_index];

//    m_quat q = eul2Quat(pose.orientation);
    m_quat q = axis2Quat(pose.orientation);

    for(int i = 0; i < 3; i++){
        d->qpos[qpos_index + i] = pose.position(i);
    }

    for(int i = 0; i < 4; i++){
        d->qpos[qpos_index + 3 + i] = q(i);
    }
}

void MuJoCoHelper::SetBodyVelocity(const string& body_name, pose_6 velocity, mjData *d) const{
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    const int joint_index = model->body_jntadr[body_id];
    const int qvel_index = model->jnt_dofadr[joint_index];

    for(int i = 0; i < 3; i++){
        d->qvel[qvel_index + i] = velocity.position(i);
    }

    for(int i = 0; i < 3; i++){
        d->qvel[qvel_index + 3 + i] = velocity.orientation(i);
    }
}

void MuJoCoHelper::GetBodyPoseQuat(const string& body_name, pose_7 &pose, mjData *d) const{
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    const int joint_index = model->body_jntadr[body_id];
    const int qpos_index = model->jnt_qposadr[joint_index];

    for(int i = 0; i < 3; i++){
        pose.position(i) = d->qpos[qpos_index + i];
    }

    for(int i = 0; i < 4; i++){
        pose.quat(i) = d->qpos[qpos_index + 3 + i];
    }
}

void MuJoCoHelper::GetBodyPoseAngle(const string& body_name, pose_6 &pose, mjData *d) const{
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    const int joint_index = model->body_jntadr[body_id];
    const int qpos_index = model->jnt_qposadr[joint_index];

    for(int i = 0; i < 3; i++){
        pose.position(i) = d->qpos[qpos_index + i];
    }

    m_quat quat;

    for(int i = 0; i < 4; i++){
        quat(i) = d->qpos[qpos_index + 3 + i];
    }

//    m_point euler = quat2Eul(quat);
    m_point axis_angle = quat2Axis(quat);

    for(int i = 0; i < 3; i++){
//        pose.orientation(i) = euler(i);
        pose.orientation(i) = axis_angle(i);
    }
}

void MuJoCoHelper::GetBodyPoseAngleViaXpos(const string& body_name, pose_6 &pose, mjData *d) const{
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());

    for(int i = 0; i < 3; i++){
        pose.position(i) = d->xpos[(3 * body_id) + i];
    }

    m_quat quat;

    for(int i = 0; i < 4; i++){
        quat(i) = d->xpos[(4 * body_id) + i];
    }

    // TODO - hmmm not sure about this whether it should be euler or axis
    m_point euler = quat2Eul(quat);

    for(int i = 0; i < 3; i++){
        pose.orientation(i) = euler(i);
    }
}

void MuJoCoHelper::GetBodyPoseQuatViaXpos(const string& body_name, pose_7 &pose, mjData *d) const{
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());

    for(int i = 0; i < 3; i++){
        pose.position(i) = d->xpos[(3 * body_id) + i];
    }

    for(int i = 0; i < 4; i++){
        pose.quat(i) = d->xquat[(4 * body_id) + i];
    }
}

void MuJoCoHelper::GetBodyVelocity(const string& body_name, pose_6 &velocity, mjData *d) const{
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    const int joint_index = model->body_jntadr[body_id];
    const int qvel_index = model->jnt_dofadr[joint_index];

    for(int i = 0; i < 3; i++){
        velocity.position(i) = d->qvel[qvel_index + i];
    }

    for(int i = 0; i < 3; i++){
        velocity.orientation(i) = d->qvel[qvel_index + 3 + i];
    }
}

void MuJoCoHelper::GetBodyAcceleration(const string& body_name, pose_6 &acceleration, mjData *d) const{
    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    const int joint_index = model->body_jntadr[body_id];
    const int qvel_index = model->jnt_dofadr[joint_index];

    for(int i = 0; i < 3; i++){
        acceleration.position(i) = d->qacc[qvel_index + i];
    }

    for(int i = 0; i < 3; i++){
        acceleration.orientation(i) = d->qacc[qvel_index + 3 + i];
    }
}
// --------------------------------- END OF BODY UTILITY ---------------------------------------

// ---------------------------------- SOFT BODY UTILITY ----------------------------------------

void MuJoCoHelper::SetSoftBodyVertexPos(const string& flex_name, int vertex_id, pose_6 &pose, mjData *d) const{
    int flex_id = mj_name2id(model, mjOBJ_FLEX, flex_name.c_str());
    int first_vert_address = model->flex_vertadr[flex_id];
    int num_vertices = model->flex_vertnum[flex_id];

    // Safety check to make sure vertex_id < num_vertices inside flex object
    if(vertex_id > num_vertices){
        std::cerr << "Vertex id in set soft body pos, vertex id: " << vertex_id << "num vertices: " << num_vertices << "\n";
        exit(1);
    }

    int body_id = model->flex_vertbodyid[first_vert_address + vertex_id];
    const int joint_index = model->body_jntadr[body_id];
    const int qpos_index = model->jnt_qposadr[joint_index];

    d->qpos[qpos_index + 0] = pose.position[0];
    d->qpos[qpos_index + 1] = pose.position[1];
    d->qpos[qpos_index + 2] = pose.position[2];
}

void MuJoCoHelper::SetSoftBodyVertexVel(const string& flex_name, int vertex_id, pose_6 &pose, mjData *d) const{
    int flex_id = mj_name2id(model, mjOBJ_FLEX, flex_name.c_str());
    int first_vert_address = model->flex_vertadr[flex_id];
    int num_vertices = model->flex_vertnum[flex_id];

    // Safety check to make sure vertex_id < num_vertices inside flex object
    if(vertex_id > num_vertices){
        std::cerr << "Vertex id in set soft body vel, vertex id: " << vertex_id << "num vertices: " << num_vertices << "\n";
        exit(1);
    }

    int body_id = model->flex_vertbodyid[first_vert_address + vertex_id];
    const int joint_index = model->body_jntadr[body_id];
    const int qvel_index = model->jnt_dofadr[joint_index];

    d->qvel[qvel_index + 0] = pose.position[0];
    d->qvel[qvel_index + 1] = pose.position[1];
    d->qvel[qvel_index + 2] = pose.position[2];
}

void MuJoCoHelper::GetSoftBodyVertexPos(const string& flex_name, int vertex_id, pose_6 &pose, mjData *d) const{

    int flex_id = mj_name2id(model, mjOBJ_FLEX, flex_name.c_str());
    int first_vertex_adr = model->flex_vertadr[flex_id];
    int num_vertices = model->flex_vertnum[flex_id];

    // Safety check to make sure vertex_id < num_vertices inside flex object
    if(vertex_id > num_vertices){
        std::cerr << "Vertex id in get soft body pos, vertex id: " << vertex_id << "num vertices: " << num_vertices << "\n";
        exit(1);
    }

    int body_id = model->flex_vertbodyid[first_vertex_adr + vertex_id];
    const int joint_index = model->body_jntadr[body_id];
    const int qpos_index = model->jnt_qposadr[joint_index];

    pose.position[0] = d->qpos[qpos_index + 0];
    pose.position[1] = d->qpos[qpos_index + 1];
    pose.position[2] = d->qpos[qpos_index + 2];

    // Jus to initialise values to something, not used
    pose.orientation[0] = 0.0;
    pose.orientation[1] = 0.0;
    pose.orientation[2] = 0.0;
}

void MuJoCoHelper::GetSoftBodyVertexPosGlobal(const string& flex_name, int vertex_id, pose_6 &pose, mjData *d) const{
    int flex_id = mj_name2id(model, mjOBJ_FLEX, flex_name.c_str());
    int first_vertex_adr = model->flex_vertadr[flex_id];
    int num_vertices = model->flex_vertnum[flex_id];

    // Safety check to make sure vertex_id < num_vertices inside flex object
    if(vertex_id > num_vertices){
        std::cerr << "Vertex id in get soft body pos, vertex id: " << vertex_id << "num vertices: " << num_vertices << "\n";
        exit(1);
    }

    int body_id = model->flex_vertbodyid[first_vertex_adr + vertex_id];
    const int joint_index = model->body_jntadr[body_id];
    const int qpos_index = model->jnt_qposadr[joint_index];

    pose.position[0] = d->xpos[(3 *body_id) + 0];
    pose.position[1] = d->xpos[(3 * body_id) + 1];
    pose.position[2] = d->xpos[(3 * body_id) + 2];

    // Jus to initialise values to something, not used
    pose.orientation[0] = 0.0;
    pose.orientation[1] = 0.0;
    pose.orientation[2] = 0.0;
}

void MuJoCoHelper::GetSoftBodyVertexVel(const string& flex_name, int vertex_id, pose_6 &pose, mjData *d) const{
    int flex_id = mj_name2id(model, mjOBJ_FLEX, flex_name.c_str());
    int first_vertex_adr = model->flex_vertadr[flex_id];
    int num_vertices = model->flex_vertnum[flex_id];

    // Safety check to make sure vertex_id < num_vertices inside flex object
    if(vertex_id > num_vertices){
        std::cerr << "Vertex id in get soft body vel, vertex id: " << vertex_id << "num vertices: " << num_vertices << "\n";
        exit(1);
    }

    int body_id = model->flex_vertbodyid[first_vertex_adr + vertex_id];
    const int joint_index = model->body_jntadr[body_id];
    const int qpos_index = model->jnt_dofadr[joint_index];

    pose.position[0] = d->qvel[qpos_index + 0];
    pose.position[1] = d->qvel[qpos_index + 1];
    pose.position[2] = d->qvel[qpos_index + 2];

    // Jus to initialise values to something, not used
    pose.orientation[0] = 0.0;
    pose.orientation[1] = 0.0;
    pose.orientation[2] = 0.0;
}

// -------------------------------END OF SOFT BODY UTILITY -------------------------------------

// - TODO create jacobian dynamically for the robot
Eigen::MatrixXd MuJoCoHelper::GetJacobian(const std::string& body_name, mjData *d) const{
    Eigen::MatrixXd jacobian(6, 7);

    Matrix<double, Dynamic, Dynamic, RowMajor> J_p(3, model->nv);
    Matrix<double, Dynamic, Dynamic, RowMajor> J_r(3, model->nv);

    int bodyId = mj_name2id(model, mjOBJ_BODY, body_name.c_str());

    mj_jacBody(model, d, J_p.data(), J_r.data(), bodyId);

    // Linear elements
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            jacobian(i, j) = J_p(i, j);
        }
    }

    // Rotational elements
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            jacobian(i + 3, j) = J_r(i, j);
        }
    }

    return jacobian;
}

int MuJoCoHelper::CheckSystemForCollisions(mjData *d) const{
    // TODO(DMackRus) - check if this is needed
    mj_forward(model, d);

    int num_contacts = d->ncon;
    int num_collisions = 0;

    for(int i = 0; i < num_contacts; i++){
        auto contact = d->contact[i];

        int body_contact_1 = model->body_rootid[model->geom_bodyid[contact.geom1]];
        int body_contact_2 = model->body_rootid[model->geom_bodyid[contact.geom2]];

        // Get name of bodies in contact
//        string body_name_1 = mj_id2name(model, mjOBJ_BODY, bodyInContact1);
//        string body_name_2 = mj_id2name(model, mjOBJ_BODY, bodyInContact2);

        // Checks if bodies in contact are the plane.
        if(body_contact_1 == 0 || body_contact_2 == 0 || (body_contact_1 == body_contact_2)){

        }
        else{
//            cout << "bodies in contact: " << bodyName1 << " " << bodyName2 << endl;
            num_collisions++;
        }
    }

    return num_collisions;
}

bool MuJoCoHelper::CheckBodyForCollisions(const string& body_name, mjData *d) const{

    // TODO(DMackRus) - Check if this is necessary
    mj_forward(model, d);

    int num_contacts = d->ncon;

    int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    bool body_collision_found = false;

    for(int i = 0; i < num_contacts; i++) {
        auto contact = d->contact[i];

        int body_contact_1 = model->body_rootid[model->geom_bodyid[contact.geom1]];
        int body_contact_2 = model->body_rootid[model->geom_bodyid[contact.geom2]];

        if(body_contact_1 == body_id && body_contact_2 != 0){
            body_collision_found = true;
            break;
        }

        if(body_contact_2 == body_id && body_contact_1 != 0){
            body_collision_found = true;
            break;
        }
    }

    return body_collision_found;
}

bool MuJoCoHelper::CheckPairForCollisions(const string& body_name_1, const string& body_name_2, mjData *d) const{

        // TODO (DMackRus) - Check if this is necessary
        mj_forward(model, d);

        int num_contacts = d->ncon;
        bool pair_collision_found = false;

        int body_id_1 = mj_name2id(model, mjOBJ_BODY, body_name_1.c_str());
        int body_id_2 = mj_name2id(model, mjOBJ_BODY, body_name_2.c_str());

        for(int i = 0; i < num_contacts; i++){
            auto contact = d->contact[i];

            int body_contact_1 = model->body_rootid[model->geom_bodyid[contact.geom1]];
            int body_contact_2 = model->body_rootid[model->geom_bodyid[contact.geom2]];

            if((body_contact_1 == body_id_1 && body_contact_2 == body_id_2) || (body_contact_1 == body_id_2 && body_contact_2 == body_id_1)){
                pair_collision_found = true;
                break;
            }
        }

        return pair_collision_found;
}

std::vector<int> MuJoCoHelper::GetContactList(mjData *d) const{
    std::vector<int> contact_list;

    // TODO(DMackRus) - Check if this is necessary
    mj_forward(model, d);
    int num_contacts = d->ncon;

    for(int i = 0; i < num_contacts; i++){
        auto contact = d->contact[i];

        int body_contact_1 = model->body_rootid[model->geom_bodyid[contact.geom1]];
        int body_contact_2 = model->body_rootid[model->geom_bodyid[contact.geom2]];

        contact_list.push_back(body_contact_1);
        contact_list.push_back(body_contact_2);
    }

    return contact_list;
}

// ------------------------------- System State Functions -----------------------------------------------
bool MuJoCoHelper::AppendSystemStateToEnd(mjData *d){

    saved_systems_state_list.push_back(mj_makeData(model));

    CpMjData(model, saved_systems_state_list.back(), d);

    return true;
}

bool MuJoCoHelper::CheckIfDataIndexExists(int list_index) const{
    return (saved_systems_state_list.size() > list_index);
}

bool MuJoCoHelper::CopySystemState(mjData *d_dest, mjData *d_src) const{

    CpMjData(model, d_dest, d_src);

    return true;
}

bool MuJoCoHelper::DeleteSystemStateFromIndex(int list_index){
    mj_deleteData(saved_systems_state_list[list_index]);
    saved_systems_state_list.erase(saved_systems_state_list.begin() + list_index);

    return true;
}

bool MuJoCoHelper::ClearSystemStateList(){
    for(auto & i : saved_systems_state_list){
        mj_deleteData(i);
    }
    saved_systems_state_list.clear();

    return true;
}

void MuJoCoHelper::CpMjData(const mjModel* m, mjData* d_dest, mjData* d_src){
    d_dest->time = d_src->time;
    mju_copy(d_dest->qpos, d_src->qpos, m->nq);
    mju_copy(d_dest->qvel, d_src->qvel, m->nv);
    mju_copy(d_dest->qacc, d_src->qacc, m->nv);
    mju_copy(d_dest->qacc_warmstart, d_src->qacc_warmstart, m->nv);
    mju_copy(d_dest->qfrc_applied, d_src->qfrc_applied, m->nv);
    mju_copy(d_dest->xfrc_applied, d_src->xfrc_applied, 6*m->nbody);
    mju_copy(d_dest->ctrl, d_src->ctrl, m->nu);
}

// ------------------------------- END OF SYSTEM STATE FUNCTIONS ----------------------------------------

bool MuJoCoHelper::ForwardSimulator(mjData *d) const{

    mj_forward(model, d);

    return true;
}

bool MuJoCoHelper::ForwardSimulatorWithSkip(mjData *d, int skip_stage, int skip_sensor) const{

    mj_forwardSkip(model, d, skip_stage, skip_sensor);

    return true;
}

// --------------------------------- Visualization Functions ---------------------------------------
void MuJoCoHelper::InitVisualisation() {

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

void MuJoCoHelper::UpdateScene(GLFWwindow *window, const char* label){

    // update scene and render
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
//    opt.flags[mjVIS_JOINT] = true;
    opt.flags[mjVIS_FLEXFACE] = false;
    opt.flags[mjVIS_FLEXVERT] = true;
//    opt.frame = mjFRAME_BODY;
    mjv_updateScene(model, vis_data, &opt, nullptr, &cam, mjCAT_ALL, &scn);

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

    mjr_overlay(0, mjGRID_TOPLEFT, rect, label, nullptr, &con);
}

void MuJoCoHelper::MouseMove(double dx, double dy, bool button_left, bool button_right, GLFWwindow *window){
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

void MuJoCoHelper::Scroll(double yoffset){
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// --------------------------------- END OF VISUALIZATION FUNCTIONS ---------------------------------------

void MuJoCoHelper::InitSimulator(double timestep, const char* file_name, bool use_plugins){

    // Should make this optional
    if(use_plugins){
        InitialisePlugins();
    }
    
    char error[1000];
    auto load_start = std::chrono::high_resolution_clock::now();
    model = mj_loadXML(file_name, nullptr, error, 1000);

    if( !model ) {
        printf("%s\n", error);
    }

    // defaults 100 , 1e-8
    model->opt.timestep = timestep;
//    model->opt.iterations = 30;
//    model->opt.tolerance = 1e-1;
//    cout << "model iterations: " << model->opt.iterations << endl;
//    cout << "model tolerance : " << model->opt.tolerance << endl;
//
//    cout << "model nq: " << model->nq << endl;
//    cout << "model nv: " << model->nv << endl;
//    cout << "model nu: " << model->nu << endl;
//    cout << "model nbody: " << model->nbody << endl;
//    cout << "model memory: " << model->memory

//    for(int i = 0; i < model->nu; i++){
//        cout << "model ctrlrange: " << model->actuator_ctrlrange[2*i] << endl;
//        cout << "model ctrlrange: " << model->actuator_ctrlrange[2*i+1] << endl;
//    }

    // make data corresponding to model
    main_data = mj_makeData(model);
    master_reset_data = mj_makeData(model);
    vis_data = mj_makeData(model);

    // Get the number of available cores
    int numCores = static_cast<int>(std::thread::hardware_concurrency());
    for(int i = 0; i < numCores; i++){
        fd_data.push_back(mj_makeData(model));
    }
    std::cout << "time to load and make data: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - load_start).count() << "ms" << std::endl;
}

void MuJoCoHelper::InitModelForFiniteDifferencing(){
    save_iterations = model->opt.iterations;
    save_tolerance = model->opt.tolerance;
    // This used to be 50, 3 seems to be the lowest i can set it without breaking the simulation
    model->opt.iterations = 5;
    model->opt.tolerance = 0;
}

void MuJoCoHelper::ResetModelAfterFiniteDifferencing() const{
    model->opt.iterations = save_iterations;
    model->opt.tolerance = save_tolerance;

}

double MuJoCoHelper::ReturnModelTimeStep() const {
    return model->opt.timestep;
}

double* MuJoCoHelper::SensorState(mjData *d, const std::string& sensor_name){

    int id = mj_name2id(model, mjOBJ_SENSOR, sensor_name.c_str());
    if (id == -1) {
        std::cerr << "sensor \"" << sensor_name << "\" not found.\n";
        return nullptr;
    } else {
        return d->sensordata + model->sensor_adr[id];
    }
}

void MuJoCoHelper::SaveDataMin(mjData* d, mujoco_data_min &data_min){
    data_min.time = d->time;
    data_min.q_pos.assign(d->qpos, d->qpos + model->nq);
    data_min.q_vel.assign(d->qvel, d->qvel + model->nv);
    data_min.q_acc.assign(d->qacc, d->qacc + model->nv);
    data_min.q_acc_warmstart.assign(d->qacc_warmstart, d->qacc_warmstart + model->nv);
    data_min.qfrc_applied.assign(d->qfrc_applied, d->qfrc_applied + model->nv);
    data_min.xfrc_applied.assign(d->xfrc_applied, d->xfrc_applied + 6*model->nbody);
    data_min.ctrl.assign(d->ctrl, d->ctrl + model->nu);
}

void MuJoCoHelper::LoadDataMin(mjData* d, const mujoco_data_min &data_min){
    d->time = data_min.time;
    std::copy(data_min.q_pos.begin(), data_min.q_pos.end(), d->qpos);
    std::copy(data_min.q_vel.begin(), data_min.q_vel.end(), d->qvel);
    std::copy(data_min.q_acc.begin(), data_min.q_acc.end(), d->qacc);
    std::copy(data_min.q_acc_warmstart.begin(), data_min.q_acc_warmstart.end(), d->qacc_warmstart);
    std::copy(data_min.qfrc_applied.begin(), data_min.qfrc_applied.end(), d->qfrc_applied);
    std::copy(data_min.xfrc_applied.begin(), data_min.xfrc_applied.end(), d->xfrc_applied);
    std::copy(data_min.ctrl.begin(), data_min.ctrl.end(), d->ctrl);
}

void MuJoCoHelper::InitialisePlugins(){
    int nplugin = mjp_pluginCount();
    if (nplugin) {
        std::printf("Built-in plugins:\n");
        for (int i = 0; i < nplugin; ++i) {
            std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
    }

    std::string path = __FILE__;
    path = path.substr(0, path.find_last_of("/\\"));
    path = path.substr(0, path.find_last_of("/\\"));
    path = path.substr(0, path.find_last_of("/\\"));

    const std::string plugin_dir = path + "/mujoco_plugins";
    std::cout << "plugin dir: " << plugin_dir << "\n";
    mj_loadAllPluginLibraries(
            plugin_dir.c_str(), +[](const char* filename, int first, int count) {
                std::printf("Plugins registered by library '%s':\n", filename);
                for (int i = first; i < first + count; ++i) {
                    std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
                }
            });
}