#include "ModelTranslator.h"

void ModelTranslator::GenerateRandomGoalAndStartState() {
    ReturnRandomStartState();
    ReturnRandomGoalState();
}

void ModelTranslator::ReturnRandomGoalState() {
    std::cerr << "Generate random goal state not overrided for " << model_name << "model, exiting \n";
    exit(1);
}

void ModelTranslator::ReturnRandomStartState() {
    std::cerr << "Generate random start state not overrided for " << model_name << "model, exiting \n";
    exit(1);
}

void ModelTranslator::InitModelTranslator(const std::string& yamlFilePath){
    task taskConfig;

    FileHandler yamlReader;
    yamlReader.readModelConfigFile(yamlFilePath, taskConfig);
    model_file_path = taskConfig.modelFilePath;

    model_name = taskConfig.modelName;
    min_N = taskConfig.minN;
    max_N = taskConfig.maxN;
    keypoint_method = taskConfig.keypointMethod;
    auto_adjust = taskConfig.auto_adjust;
    iterative_error_threshold = taskConfig.iterativeErrorThreshold;
    const char* _modelPath = model_file_path.c_str();

    openloop_horizon = taskConfig.openloop_horizon;
    MPC_horizon = taskConfig.mpc_horizon;

    // Initialise physics simulator
    vector<string> bodyNames;
    for(auto & robot : taskConfig.robots){
        bodyNames.push_back(robot.name);
        for(int j = 0; j < robot.jointNames.size(); j++){
            jerk_thresholds.push_back(robot.jointJerkThresholds[j]);
            // TODO fix this duplicate jerk thresholds
            accel_thresholds.push_back(robot.jointJerkThresholds[j]);
            velocity_change_thresholds.push_back(robot.magVelThresholds[j]);
        }

    }

    for(auto & bodiesState : taskConfig.bodiesStates){
        bodyNames.push_back(bodiesState.name);
        for(int j = 0; j < 3; j++){
            jerk_thresholds.push_back(bodiesState.linearJerkThreshold[j]);
            jerk_thresholds.push_back(bodiesState.angularJerkThreshold[j]);

            // TODO fix this duplicate jerk thresholds
            accel_thresholds.push_back(bodiesState.linearJerkThreshold[j]);
            accel_thresholds.push_back(bodiesState.angularJerkThreshold[j]);

            velocity_change_thresholds.push_back(bodiesState.linearMagVelThreshold[j]);
            velocity_change_thresholds.push_back(bodiesState.angularMagVelThreshold[j]);
        }
    }

    MuJoCo_helper = std::make_shared<MuJoCoHelper>(taskConfig.robots, bodyNames);
    MuJoCo_helper->InitSimulator(taskConfig.modelTimeStep, _modelPath);

    active_state_vector.robots = taskConfig.robots;
    active_state_vector.bodiesStates = taskConfig.bodiesStates;

    // --------- Set size of state vector correctly ------------
    state_vector_size = 0;
    for(auto & robot : active_state_vector.robots){
        state_vector_size += (2 * static_cast<int>(robot.jointNames.size()));
    }

    for(auto & bodiesState : active_state_vector.bodiesStates){
        for(int j = 0; j < 3; j++){
            if(bodiesState.activeLinearDOF[j]){
                state_vector_size += 2;
            }
            if(bodiesState.activeAngularDOF[j]){
                state_vector_size += 2;
            }
        }
    }

    dof = state_vector_size / 2;
//    X_desired.resize(state_vector_size, 1);
//    X_start.resize(state_vector_size, 1);

    // --------- Set size of cost matrices correctly ------------
    num_ctrl = static_cast<int>(active_state_vector.robots[0].actuatorNames.size());
    Q.resize(state_vector_size);
    Q.setZero();
    Q_terminal.resize(state_vector_size);
    Q_terminal.setZero();
    R.resize(num_ctrl);
    R.setZero();

    // -----------------------------------------------------------------------------------------
    //                      Assign cost matrices
    // ------------------------------------------------------------------------------------------
    // Loop through robots and starting assigning state specific costs
    int Q_index = 0;
    for(auto & robot : active_state_vector.robots){
        int robotNumJoints = static_cast<int>(robot.jointNames.size());

        // Loop through the robot joints
        for(int j = 0; j < robotNumJoints; j++){
            Q.diagonal()[Q_index + j] = robot.jointPosCosts[j];
            Q_terminal.diagonal()[Q_index + j] = robot.terminalJointPosCosts[j];

            Q.diagonal()[Q_index + j + dof] = robot.jointVelCosts[j];
            Q_terminal.diagonal()[Q_index + j + dof] = robot.terminalJointVelCosts[j];

//            X_desired(Q_index + j, 0) = robot.goalPos[j];
//            X_desired(Q_index + j + dof, 0) = 0.0f;

//            X_start(Q_index + j, 0) = robot.startPos[j];
//            X_start(Q_index + j + dof, 0) = 0.0f;
        }
        Q_index += robotNumJoints;
    }

    // Loop through bodies
    for(auto & bodiesState : active_state_vector.bodiesStates){

        // Loop through linear states first
        int activeDofCounter = 0;
        for(int j = 0; j < 3; j++){
            if(bodiesState.activeLinearDOF[j]){
                Q.diagonal()[Q_index + activeDofCounter] = bodiesState.linearPosCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter] = bodiesState.terminalLinearPosCost[j];

                Q.diagonal()[Q_index + activeDofCounter + dof] = bodiesState.linearVelCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter + dof] = bodiesState.terminalLinearVelCost[j];

//                X_desired(Q_index + activeDofCounter, 0) = bodiesState.goalLinearPos[j];
//                X_desired(Q_index + activeDofCounter + dof, 0) = 0.0f;

//                X_start(Q_index + activeDofCounter, 0) = bodiesState.startLinearPos[j];
//                X_start(Q_index + activeDofCounter + dof, 0) = 0.0f;

                activeDofCounter++;
            }
        }

        // Loop through angular states second
        for(int j = 0; j < 3; j++){
            if(bodiesState.activeAngularDOF[j]){
                Q.diagonal()[Q_index + activeDofCounter] = bodiesState.angularPosCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter] = bodiesState.terminalAngularPosCost[j];

                Q.diagonal()[Q_index + activeDofCounter + dof] = bodiesState.angularVelCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter + dof] = bodiesState.terminalAngularVelCost[j];

//                X_desired(Q_index + activeDofCounter, 0) = bodiesState.goalAngularPos[j];
//                X_desired(Q_index + activeDofCounter + dof, 0) = 0.0f;

//                X_start(Q_index + activeDofCounter, 0) = bodiesState.startAngularPos[j];
//                X_start(Q_index + activeDofCounter + dof, 0) = 0.0f;

                activeDofCounter++;
            }
        }
        Q_index += activeDofCounter;
    }

    // Loop through robots and starting assigning control specific costs
    int R_index = 0;
    for(auto & robot : active_state_vector.robots){
        int robotNumActuators = static_cast<int>(robot.actuatorNames.size());

        // Loop through the robot joints
        for(int j = 0; j < robotNumActuators; j++){
            R.diagonal()[R_index + j] = robot.jointControlCosts[j];
        }

        R_index += robotNumActuators;
    }
    // ----------------------------------------------------------------------------------------------

    // Assign state vector names
    for(auto & robot : active_state_vector.robots){
        for(const auto & jointName : robot.jointNames){
            state_vector_names.push_back(jointName);
        }
    }

    // bodies
    for(auto & bodiesState : active_state_vector.bodiesStates){

        if(bodiesState.activeLinearDOF[0]){
            state_vector_names.push_back(bodiesState.name + "_x");
        }
        if(bodiesState.activeLinearDOF[1]){
            state_vector_names.push_back(bodiesState.name + "_y");
        }
        if(bodiesState.activeLinearDOF[2]){
            state_vector_names.push_back(bodiesState.name + "_z");
        }

        if(bodiesState.activeAngularDOF[0]){
            state_vector_names.push_back(bodiesState.name + "_roll");
        }
        if(bodiesState.activeAngularDOF[1]){
            state_vector_names.push_back(bodiesState.name + "_pitch");
        }
        if(bodiesState.activeAngularDOF[2]){
            state_vector_names.push_back(bodiesState.name + "_yaw");
        }
    }

//    std::cout << "initial state vector names: ";
//    for(const auto & state_vector_name : state_vector_names){
//        std::cout << state_vector_name << " ";
//    }
//    std::cout << "\n";

    cout << "Q: " << Q.diagonal().transpose() << std::endl;
    cout << "R: " << R.diagonal().transpose() << std::endl;
    cout << "Q_terminal: " << Q_terminal.diagonal().transpose() << endl;
}

void ModelTranslator::UpdateStateVector(std::vector<std::string> state_vector_names, bool add_extra_states){

    // state vector name -
    // robot joint names
    // bodies - {body_name}_x, {body_name}_y, {body_name}_z, {body_name}_roll, {body_name}_pitch, {body_name}_yaw

    // TODO (DMackRus) - This is an assumption but should be fine
    if(add_extra_states){
        dof += state_vector_names.size();
    }
    else{
        dof -= state_vector_names.size();
    }

    state_vector_size = dof * 2;

    // Resize Q matrices
    Q.diagonal().resize(dof * 2);
    Q_terminal.diagonal().resize(dof * 2);

    // Resize X_desired and X_start
//    X_desired.resize(dof * 2, 1);
//    X_start.resize(dof * 2, 1);

    // TODO (DMackRus) - think there is a better way to do this if i rewrite how my state vector is stored
    for(auto & robot : active_state_vector.robots){
        for(int joint = 0; joint < robot.jointNames.size(); joint++){

            for(int i = 0; i < state_vector_names.size(); i++){
                // TODO (DMackRus) - Need to add ability to activate / deactivate joints from state vector
            }
        }
    }

    for(auto & bodiesState : active_state_vector.bodiesStates){
        std::string body_name = bodiesState.name;
        for(auto & state_vector_name : state_vector_names){
            size_t found = state_vector_name.find(body_name);

            if (found != std::string::npos) {
                state_vector_name.erase(found, body_name.length());
                if(state_vector_name == "_x"){
                    bodiesState.activeLinearDOF[0] = add_extra_states;
                }
                else if(state_vector_name == "_y"){
                    bodiesState.activeLinearDOF[1] = add_extra_states;
                }
                else if(state_vector_name == "_z"){
                    bodiesState.activeLinearDOF[2] = add_extra_states;
                }
                // TODO (DMackRus) - Need to add ability to activate / deactivate bodies rotation from state vector
//                else if(state_vector_names[i] == "_roll"){
//                    active_state_vector.bodiesStates[body].activeAngularDOF[0] = true;
//                }
//                else if(state_vector_names[i] == "_pitch"){
//                    active_state_vector.bodiesStates[body].activeAngularDOF[1] = true;
//                }
//                else if(state_vector_names[i] == "_yaw"){
//                    active_state_vector.bodiesStates[body].activeAngularDOF[2] = true;
//                }
            }
        }
    }

    // - Regenerate Q cost matrices
    int Q_index = 0;
    for(auto & robot : active_state_vector.robots){
        int robotNumJoints = static_cast<int>(robot.jointNames.size());

        // Loop through the robot joints
        for(int j = 0; j < robotNumJoints; j++){
            Q.diagonal()[Q_index + j] = robot.jointPosCosts[j];
            Q_terminal.diagonal()[Q_index + j] = robot.terminalJointPosCosts[j];

            Q.diagonal()[Q_index + j + dof] = robot.jointVelCosts[j];
            Q_terminal.diagonal()[Q_index + j + dof] = robot.terminalJointVelCosts[j];

//            X_desired(Q_index + j, 0) = robot.goalPos[j];
//            X_desired(Q_index + j + dof, 0) = 0.0f;

//            X_start(Q_index + j, 0) = robot.startPos[j];
//            X_start(Q_index + j + dof, 0) = 0.0f;
        }
        Q_index += robotNumJoints;
    }

    // Loop through bodies
    for(auto & bodiesState : active_state_vector.bodiesStates){

        int activeDOFs = 0;
        for(int j = 0; j < 3; j++){
            if(bodiesState.activeLinearDOF[j]){
                activeDOFs++;
            }
            if(bodiesState.activeAngularDOF[j]){
                activeDOFs++;
            }
        }

        // Loop through linear states first
        int activeDofCounter = 0;
        for(int j = 0; j < 3; j++){
            if(bodiesState.activeLinearDOF[j]){
                Q.diagonal()[Q_index + activeDofCounter] = bodiesState.linearPosCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter] = bodiesState.terminalLinearPosCost[j];

                Q.diagonal()[Q_index + activeDofCounter + dof] = bodiesState.linearVelCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter + dof] = bodiesState.terminalLinearVelCost[j];

//                X_desired(Q_index + j, 0) = bodiesState.goalLinearPos[j];
//                X_desired(Q_index + j + dof, 0) = 0.0f;

//                X_start(Q_index + j, 0) = bodiesState.startLinearPos[j];
//                X_start(Q_index + j + dof, 0) = 0.0f;

                activeDofCounter++;
            }
        }

        // Loop through angular states second
        for(int j = 0; j < 3; j++){
            if(bodiesState.activeAngularDOF[j]){
                Q.diagonal()[Q_index + activeDofCounter] = bodiesState.angularPosCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter] = bodiesState.terminalAngularPosCost[j];

                Q.diagonal()[Q_index + activeDofCounter + dof] = bodiesState.angularVelCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter + dof] = bodiesState.terminalAngularVelCost[j];

//                X_desired(Q_index + j, 0) = bodiesState.goalAngularPos[j];
//                X_desired(Q_index + j + dof, 0) = 0.0f;

//                X_start(Q_index + j, 0) = bodiesState.startAngularPos[j];
//                X_start(Q_index + j + dof, 0) = 0.0f;

                activeDofCounter++;
            }
        }
        Q_index += activeDOFs;
    }
}

std::vector<std::string> ModelTranslator::GetStateVectorNames(){
    std::vector<std::string> state_vector_names;

    for(auto & robot : active_state_vector.robots){
        for(const auto & jointName : robot.jointNames){
            // TODO (DMackRus) - Need to add ability for joints not to be automatically included in state vector?
            state_vector_names.push_back(jointName);
        }
    }

    for(auto & bodiesState : active_state_vector.bodiesStates){
        if(bodiesState.activeLinearDOF[0]){
            state_vector_names.push_back(bodiesState.name + "_x");
        }

        if(bodiesState.activeLinearDOF[1]){
            state_vector_names.push_back(bodiesState.name + "_y");
        }

        if(bodiesState.activeLinearDOF[2]){
            state_vector_names.push_back(bodiesState.name + "_z");
        }

        if(bodiesState.activeAngularDOF[0]){
            state_vector_names.push_back(bodiesState.name + "_roll");
        }

        if(bodiesState.activeAngularDOF[1]){
            state_vector_names.push_back(bodiesState.name + "_pitch");
        }

        if(bodiesState.activeAngularDOF[2]){
            state_vector_names.push_back(bodiesState.name + "_yaw");
        }
    }

    return state_vector_names;
}

double ModelTranslator::CostFunction(mjData* d, bool terminal){
    double cost = 0.0f;

    // Loop through robots in simulator
    for(auto & robot : active_state_vector.robots){
        int robot_num_joints = static_cast<int>(robot.jointNames.size());

        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        std::vector<double> joint_controls;

        MuJoCo_helper->GetRobotJointsPositions(robot.name, joint_positions, d);
        MuJoCo_helper->GetRobotJointsVelocities(robot.name, joint_velocities, d);
        MuJoCo_helper->GetRobotJointsControls(robot.name, joint_controls, d);

        // Loop through the robot joints
        for(int j = 0; j < robot_num_joints; j++) {

            double cost_pos, cost_vel;

            if(terminal){
                cost_pos = robot.terminalJointPosCosts[j];
                cost_vel = robot.terminalJointVelCosts[j];
            }
            else{
                cost_pos = robot.jointPosCosts[j];
                cost_vel = robot.jointVelCosts[j];
            }

            // Position costs
            cost += cost_pos * pow((joint_positions[j] - robot.goalPos[j]), 2);

            // Velocity costs
            cost += cost_vel * pow(joint_velocities[j] - robot.goalVel[j], 2);

            // Control costs
            cost += robot.jointControlCosts[j] * pow(joint_controls[j], 2);
        }
    }

    // Loop through the bodies in simulation
    for(const auto &body : active_state_vector.bodiesStates){
        cost += CostFunctionBody(body, d, terminal);
    }

    return cost;
}

double ModelTranslator::CostFunctionBody(const bodyStateVec body, mjData *d, bool terminal){
    double cost = 0.0;

    pose_6 body_pose;
    pose_6 body_vel;
    MuJoCo_helper->GetBodyPoseAngle(body.name, body_pose, d);
    MuJoCo_helper->GetBodyVelocity(body.name, body_vel, d);

    // Linear costs
    for(int j = 0; j < 3; j++){
        if(body.activeLinearDOF[j]){

            if(terminal){
                // Position cost
                cost += body.terminalLinearPosCost[j] * pow((body_pose.position[j] - body.goalLinearPos[j]), 2);

                // Velocity cost
                cost += body.terminalLinearVelCost[j] * pow(body_vel.position[j], 2);

            }
            else{
                // Position cost
                cost += body.linearPosCost[j] * pow((body_pose.position[j] - body.goalLinearPos[j]), 2);

                // Velocity cost
                cost += body.linearVelCost[j] * pow(body_vel.position[j], 2);
            }
        }
    }

    // Angular cost
    // Compute rotation matrix of body in world frame
    Eigen::Matrix3d current_rot_mat = eul2RotMat(body_pose.orientation);
////        std::cout << "rot mat: \n" << current_rot_mat << "\n";
//
//    // Convert desired orientation to rotation matrix
    m_point desired_eul;
    desired_eul(0) = body.goalAngularPos[0];
    desired_eul(1) = body.goalAngularPos[1];
    desired_eul(2) = body.goalAngularPos[2];
    Eigen::Matrix3d desired_rot_mat = eul2RotMat(desired_eul);

//    m_quat current, desired, inv_current, diff;
//    current = eul2Quat(desired_eul);
//    desired = eul2Quat(body_pose.orientation);
//    inv_current = invQuat(current);
//    diff = multQuat(inv_current, desired);


    double dot_x, dot_y, dot_z = 0.0f;

    // Axis X
    if(body.activeAngularDOF[0]){
        dot_x = current_rot_mat(0, 0) * desired_rot_mat(0, 0) +
                current_rot_mat(1, 0) * desired_rot_mat(1, 0) +
                current_rot_mat(2, 0) * desired_rot_mat(2, 0);

        if(terminal) {
            cost += body.terminalAngularPosCost[0] * acos(dot_x);
        }
        else{
            cost += body.angularPosCost[0] * acos(dot_x);
        }
    }

    // Axis Y
    if(body.activeAngularDOF[1]){
        dot_y = current_rot_mat(0, 1) * desired_rot_mat(0, 1) +
                current_rot_mat(1, 1) * desired_rot_mat(1, 1) +
                current_rot_mat(2, 1) * desired_rot_mat(2, 1);

        if(terminal) {
            cost += body.terminalAngularPosCost[1] * acos(dot_y);
        }
        else{
            cost += body.angularPosCost[1] * acos(dot_y);
        }
    }

    // Axis Z
    if(body.activeAngularDOF[2]){
        dot_z = current_rot_mat(0, 2) * desired_rot_mat(0, 2) +
                current_rot_mat(1, 2) * desired_rot_mat(1, 2) +
                current_rot_mat(2, 2) * desired_rot_mat(2, 2);

        if(terminal) {
            cost += body.terminalAngularPosCost[2] * acos(dot_z);
        }
        else{
            cost += body.angularPosCost[2] * acos(dot_z);
        }
    }

//        std::cout << "dot x: " << dot_x << " dot y: " << dot_y << " dot z: " << dot_z << std::endl;

//    if(terminal){
//        cost += body.terminalAngularPosCost[0] * acos(dot_x);
//        cost += body.terminalAngularPosCost[1] * acos(dot_y);
//        cost += body.terminalAngularPosCost[2] * acos(dot_z);
//    }
//    else{
//        cost += body.angularPosCost[0] * acos(dot_x);
//        cost += body.angularPosCost[1] * acos(dot_y);
//        cost += body.angularPosCost[2] * acos(dot_z);
//    }

//    cost += body.angularPosCost[0] * acos(dot_x);
//    cost += body.angularPosCost[1] * acos(dot_y);
//    cost += body.angularPosCost[2] * acos(dot_z);

    for(int j = 0; j < 3; j++){
        if(body.activeAngularDOF[j]){

            if(terminal){
                // Velocity cost
                cost += body.terminalLinearVelCost[j] * pow(body_vel.orientation[j], 2);

            }
            else{
                // Velocity cost
                cost += body.linearVelCost[j] * pow(body_vel.orientation[j], 2);
            }
        }
    }

    return cost;
}

void ModelTranslator::CostDerivatives(mjData* d, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal){

    // Size cost derivatives appropriately
    l_x.resize(state_vector_size, 1);
    l_xx.resize(state_vector_size, state_vector_size);

    l_u.resize(num_ctrl, 1);
    l_uu.resize(num_ctrl, num_ctrl);

    // Set matrices to zero as these matrices should mostly be sparse.
    l_u.setZero();
    l_uu.setZero();
    l_x.setZero();
    l_xx.setZero();

    int Q_index = 0;
    int R_index = 0;
    for(auto & robot : active_state_vector.robots){
        int robot_num_joints = static_cast<int>(robot.jointNames.size());
        int robot_num_actuators = static_cast<int>(robot.actuatorNames.size());

        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        std::vector<double> joint_controls;

        MuJoCo_helper->GetRobotJointsPositions(robot.name, joint_positions, d);
        MuJoCo_helper->GetRobotJointsVelocities(robot.name, joint_velocities, d);
        MuJoCo_helper->GetRobotJointsControls(robot.name, joint_controls, d);

        // Loop through the robot joints (position and velocity)
        for(int i = 0; i < robot_num_joints; i++) {

            double cost_pos, cost_vel;

            if(terminal){
                cost_pos = robot.terminalJointPosCosts[i];
                cost_vel = robot.terminalJointVelCosts[i];
            }
            else{
                cost_pos = robot.jointPosCosts[i];
                cost_vel = robot.jointVelCosts[i];
            }

            // l_x = 2 * cost * diff
            // position and velocity
            l_x(Q_index + i) = 2 * cost_pos * (joint_positions[i] - robot.goalPos[i]);
            l_x(Q_index + i + dof) = 2 * cost_vel * (joint_velocities[i] - robot.goalVel[i]);

            // l_xx = 2 * cost
            l_xx(Q_index + i, Q_index + i) = 2 * cost_pos;
            l_xx(Q_index + i + dof, Q_index + i + dof) = 2 * cost_vel;

        }

        for(int i = 0; i < robot_num_actuators; i++){
            // l_u = 2 * cost * diff
            l_u(R_index + i) = 2 * robot.jointControlCosts[i] * joint_controls[i];

            // l_uu = 2 * cost
            l_uu(R_index, R_index) = 2 * robot.jointControlCosts[i];
        }

        // Increment index of Q and R (inside state vector and control vector repsectively)
        Q_index += robot_num_joints;
        R_index += robot_num_actuators;
    }

    for(const auto& body : active_state_vector.bodiesStates){
        pose_6 body_pose;
        pose_6 body_vel;
        MuJoCo_helper->GetBodyPoseAngle(body.name, body_pose, d);
        MuJoCo_helper->GetBodyVelocity(body.name, body_vel, d);

        // Linear cost derivatives
        for(int j = 0; j < 3; j++) {
            if (body.activeLinearDOF[j]) {

                if (terminal) {
                    // Position cost derivatives
                    l_x(Q_index) = 2 * body.terminalLinearPosCost[j] * (body_pose.position[j] - body.goalLinearPos[j]);
                    l_xx(Q_index, Q_index) = 2 * body.terminalLinearPosCost[j];

                    // Velocity cost derivatives
                    l_x(Q_index + dof) = 2 * body.terminalLinearVelCost[j] * (body_vel.position[j]);
                    l_xx(Q_index + dof, Q_index + dof) = 2 * body.terminalLinearVelCost[j];
                } else {
                    // Position cost derivatives
                    l_x(Q_index) = 2 * body.linearPosCost[j] * (body_pose.position[j] - body.goalLinearPos[j]);
                    l_xx(Q_index, Q_index) = 2 * body.linearPosCost[j];

                    // Velocity cost derivatives
                    l_x(Q_index + dof) = 2 * body.linearVelCost[j] * (body_vel.position[j]);
                    l_xx(Q_index + dof, Q_index + dof) = 2 * body.linearVelCost[j];
                }

                Q_index++;
            }
        }

        // TODO - if cost is -,  we should also probbaly skip computation
        // If no angular dofs active for this body, skip!
        if(!body.activeAngularDOF[0] && !body.activeAngularDOF[1] && !body.activeAngularDOF[2]){
            continue;
        }

        // Angular cost derivatives
        // Angular pos cost = w1*acos(dot_x) + w2*acos(dot_y) + w3*acos(dot_z)

        //Compute 3D rotation matrix of current body orientation as well as the desired orientation
//        Eigen::Matrix3d current_rot_mat = eul2RotMat(body_pose.orientation);
//
//        // Convert desired orientation to rotation matrix
//        m_point desired;
//        desired(0) = body.goalAngularPos[0];
//        desired(1) = body.goalAngularPos[1];
//        desired(2) = body.goalAngularPos[2];
//        Eigen::Matrix3d desired_rot_mat = eul2RotMat(desired);
//
//        double dot_prods[3];
//        for(int j = 0; j < 3; j++){
//            dot_prods[j] = current_rot_mat(0, j) * desired_rot_mat(0, j) +
//                    current_rot_mat(1, j) * desired_rot_mat(1, j) +
//                    current_rot_mat(2, j) * desired_rot_mat(2, j);
//        }
//
//        double dot_x, dot_y, dot_z;
//        dot_x = dot_prods[0];
//        dot_y = dot_prods[1];
//        dot_z = dot_prods[2];
//
//        // Compute and store sin and cosine values for
//        // object roll, pitch, yaw (r, p, y)
//        // as well as goal roll, pitch and yaw (gr, gp, gy)
//        double sr, sp, sy, cr, cp, cy;
//        double sgr, sgp, sgy, cgr, cgp, cgy;
//
//        sr = sin(body_pose.orientation[0]);
//        sp = sin(body_pose.orientation[1]);
//        sy = sin(body_pose.orientation[2]);
//        cr = cos(body_pose.orientation[0]);
//        cp = cos(body_pose.orientation[1]);
//        cy = cos(body_pose.orientation[2]);
//
//        sgr = sin(body.goalAngularPos[0]);
//        sgp = sin(body.goalAngularPos[1]);
//        sgy = sin(body.goalAngularPos[2]);
//        cgr = cos(body.goalAngularPos[0]);
//        cgp = cos(body.goalAngularPos[1]);
//        cgy = cos(body.goalAngularPos[2]);
//
//        // first order deriv for dot_z, w.r.t roll, pitch, yaw
//        double ddot_z_dr, ddot_z_dp, ddot_z_dy;
//        // Storing values from computation of derivative dependant on desired goal angles
//        double F, G, H;
//
//        F = cgy*sgp*cgr + sgy*sgr;
//        G = sgy*sgp*cgr + sgy*sgr;
//        H = cgp*cgr;
//
//        ddot_z_dr = F*(-cy*sp*cr + sy*cr) + G*(-sy*sp*cr + sy*cr) + H*(-cp*sr);
//        ddot_z_dy = F*(-sy*sp*cr + cy*sr) + G*(cy*sp*cr + cy*sr);
//        ddot_z_dp = F*(cy*cp*cr) + G*(sy*cp*cr) + H*(-sp*cr);
//
////        std::cout << "ddot_z_dy: " << ddot_z_dy << std::endl;
//
//
//        double dacos_dot_z;
//        double dl_dr, dl_dp, dl_dy;
//
//        // This comes from deriv of acos(u)
//        dacos_dot_z = -1 / (sqrt(1 - pow(dot_z, 2)));
//
//        // TODO - is this the correct cost weighting???
//        if(terminal){
//            dl_dr = body.terminalAngularPosCost[2] * dacos_dot_z * ddot_z_dr;
//            dl_dp = body.terminalAngularPosCost[2] * dacos_dot_z * ddot_z_dp;
//            dl_dy = body.terminalAngularPosCost[2] * dacos_dot_z * ddot_z_dy;
//        }
//        else{
//            dl_dr = body.angularPosCost[2] * dacos_dot_z * ddot_z_dr;
//            dl_dp = body.angularPosCost[2] * dacos_dot_z * ddot_z_dp;
//            dl_dy = body.angularPosCost[2] * dacos_dot_z * ddot_z_dy;
//        }
//
//        // Use these values to plug into l_x
//
//        // TODO Q indexing code????
////        std::cout << "dl_dr" << dl_dr << " dl_dp: " << dl_dp << " dl_dy: " << dl_dy << std::endl;
//        l_x(Q_index) = dl_dr;
//        l_x(Q_index + 1) = dl_dp;
//        l_x(Q_index + 2) = dl_dy;
//
//        l_xx(Q_index, Q_index) = dl_dr*dl_dr;
//        l_xx(Q_index + 1, Q_index + 1) = dl_dp*dl_dp;
//        l_xx(Q_index + 2, Q_index + 2) = dl_dy*dl_dy;


        // TODO - second order l_xx derivs, also add computation from x and y vectors.......

        // use internal F.D of costfunction body to compute derivatives????
        double cost_dec, cost_inc;
        double eps = 1e-5;
        // Perturb 3 orientations

        for(int j = 0; j < 3; j++){

            body_pose.orientation[j] += eps;
            // TODO - possible issue here with using the data object itself.
            MuJoCo_helper->SetBodyPoseAngle(body.name, body_pose, d);

            cost_inc = CostFunctionBody(body, d, terminal);

            // Undo perturbation and apply negative perturb
            body_pose.orientation[j] -= (2 * eps);

            MuJoCo_helper->SetBodyPoseAngle(body.name, body_pose, d);

            cost_dec = CostFunctionBody(body, d, terminal);

            //Undo perturbation
            body_pose.orientation[j] += eps;

            MuJoCo_helper->SetBodyPoseAngle(body.name, body_pose, d);

            l_x(Q_index) = (cost_inc - cost_dec) / (2 * eps);

            l_xx(Q_index, Q_index) = l_x(Q_index) * l_x(Q_index);

            Q_index ++;
        }






    }

//    std::cout << "l_x \n" << l_x << std::endl;
}

bool ModelTranslator::TaskComplete(mjData* d, double &dist){
    dist = 0.0;
    return false;
}

std::vector<MatrixXd> ModelTranslator::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> emptyInitSetupControls;
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
    return emptyInitSetupControls;
}

MatrixXd ModelTranslator::ReturnStateVector(mjData* d){
    MatrixXd position_vector(dof, 1);
    MatrixXd velocity_vector(dof, 1);
    MatrixXd state_vector(state_vector_size, 1);

    position_vector = returnPositionVector(d);
    velocity_vector = returnVelocityVector(d);

    state_vector.block(0, 0, dof, 1) =
            position_vector.block(0, 0, dof, 1);

    state_vector.block(dof, 0, dof, 1) =
            velocity_vector.block(0, 0, dof, 1);

    return state_vector;
}

bool ModelTranslator::SetStateVector(MatrixXd state_vector, mjData* d){

    if(state_vector.rows() != state_vector_size){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    MatrixXd position_vector(state_vector_size / 2, 1);
    MatrixXd velocity_vector(state_vector_size / 2, 1);

    position_vector = state_vector.block(0, 0, state_vector_size / 2, 1);
    velocity_vector = state_vector.block(state_vector_size / 2, 0, state_vector_size / 2, 1);

    setPositionVector(position_vector, d);
    setVelocityVector(velocity_vector, d);

    return true;
}

MatrixXd ModelTranslator::ReturnControlVector(mjData* d){
    MatrixXd controlVector(num_ctrl, 1);
    int currentStateIndex = 0;

    // loop through all the present robots
    for(auto & robot : active_state_vector.robots){
        vector<double> jointControls;
        MuJoCo_helper->GetRobotJointsControls(robot.name, jointControls, d);
        for(int j = 0; j < robot.actuatorNames.size(); j++){

            controlVector(currentStateIndex + j, 0) = jointControls[j];
        }

        currentStateIndex += static_cast<int>(robot.actuatorNames.size());
    }

    return controlVector;
}

MatrixXd ModelTranslator::ReturnControlLimits(){
    MatrixXd control_limits(num_ctrl*2, 1);
    int current_state_index = 0;

    // loop through all the present robots
    for(auto & robot : active_state_vector.robots){
        vector<double> robot_control_limits;
        MuJoCo_helper->GetRobotControlLimits(robot.name, robot_control_limits);
        for(int j = 0; j < robot.actuatorNames.size() * 2; j++){
            control_limits(current_state_index + j, 0) = robot_control_limits[j];
        }

        current_state_index += static_cast<int>(robot.actuatorNames.size());
    }

    return control_limits;
}

bool ModelTranslator::SetControlVector(MatrixXd control_vector, mjData* d){
    if(control_vector.rows() != num_ctrl){
        cout << "ERROR: control vector size does not match the size of the control vector in the model translator" << endl;
        return false;
    }

    int currentStateIndex = 0;

    // loop through all the present robots
    for(auto & robot : active_state_vector.robots){
        vector<double> jointControls;
        for(int j = 0; j < robot.actuatorNames.size(); j++){

            jointControls.push_back(control_vector(currentStateIndex + j));
        }

        MuJoCo_helper->SetRobotJointsControls(robot.name, jointControls, d);

        currentStateIndex += static_cast<int>(robot.actuatorNames.size());
    }

    return true;
}

MatrixXd ModelTranslator::returnPositionVector(mjData* d){
    MatrixXd position_vector(dof, 1);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(auto & robot : active_state_vector.robots){
        vector<double> jointPositions;
        MuJoCo_helper->GetRobotJointsPositions(robot.name, jointPositions, d);

        for(int j = 0; j < robot.jointNames.size(); j++){
            position_vector(j, 0) = jointPositions[j];
        }

        // Increment the current state index by the number of joints in the robot
        currentStateIndex += static_cast<int>(robot.jointNames.size());
    }

    // Loop through all bodies in the state vector
    for(auto & bodiesState : active_state_vector.bodiesStates){
        // Get the body's position and orientation
        pose_6 bodyPose;
        MuJoCo_helper->GetBodyPoseAngle(bodiesState.name, bodyPose, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.activeLinearDOF[j]) {
                position_vector(currentStateIndex, 0) = bodyPose.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.activeAngularDOF[j]){
                position_vector(currentStateIndex, 0) = bodyPose.orientation[j];
                currentStateIndex++;
            }
        }
    }

    return position_vector;
}

MatrixXd ModelTranslator::returnVelocityVector(mjData* d){
    MatrixXd velocity_vector(dof, 1);
    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(auto & robot : active_state_vector.robots){
        vector<double> jointVelocities;
        MuJoCo_helper->GetRobotJointsVelocities(robot.name, jointVelocities, d);

        for(int j = 0; j < robot.jointNames.size(); j++){
            velocity_vector(j, 0) = jointVelocities[j];
        }

        // Increment the current state index by the number of joints in the robot
        currentStateIndex += static_cast<int>(robot.jointNames.size());
    }

    // Loop through all bodies in the state vector
    for(auto & bodiesState : active_state_vector.bodiesStates){
        // Get the body's position and orientation
        pose_6 bodyVelocities;
        MuJoCo_helper->GetBodyVelocity(bodiesState.name, bodyVelocities, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.activeLinearDOF[j]) {
                velocity_vector(currentStateIndex, 0) = bodyVelocities.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.activeAngularDOF[j]){
                velocity_vector(currentStateIndex, 0) = bodyVelocities.orientation[j];
                currentStateIndex++;
            }
        }
    }

    return velocity_vector;
}

//MatrixXd ModelTranslator::returnAccelerationVector(mjData* d){
//    MatrixXd accel_vector(dof, 1);
//
//    int currentStateIndex = 0;
//
//    // Loop through all robots in the state vector
//    for(auto & robot : active_state_vector.robots){
//        vector<double> jointAccelerations;
//        MuJoCo_helper->getRobotJointsAccelerations(robot.name, jointAccelerations, d);
//
//        for(int j = 0; j < robot.jointNames.size(); j++){
//            accel_vector(j, 0) = jointAccelerations[j];
//        }
//
//        // Increment the current state index by the number of joints in the robot
//        currentStateIndex += static_cast<int>(robot.jointNames.size());
//    }
//
//    // Loop through all bodies in the state vector
//    for(auto & bodiesState : active_state_vector.bodiesStates){
//        // Get the body's position and orientation
//        pose_6 bodyAccelerations;
//        MuJoCo_helper->getBodyAcceleration(bodiesState.name, bodyAccelerations, d);
//
//        for(int j = 0; j < 3; j++) {
//            // Linear positions
//            if (bodiesState.activeLinearDOF[j]) {
//                accel_vector(currentStateIndex, 0) = bodyAccelerations.position[j];
//                currentStateIndex++;
//            }
//        }
//        for(int j = 0; j < 3; j++) {
//            // angular positions
//            if(bodiesState.activeAngularDOF[j]){
//                accel_vector(currentStateIndex, 0) = bodyAccelerations.orientation[j];
//                currentStateIndex++;
//            }
//        }
//    }
//
//    return accel_vector;
//}

bool ModelTranslator::setPositionVector(MatrixXd position_vector, mjData* d){
    if(position_vector.rows() != (state_vector_size / 2)){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(auto & robot : active_state_vector.robots){
        vector<double> jointPositions;

        for(int j = 0; j < robot.jointNames.size(); j++){
            jointPositions.push_back(position_vector(j, 0));
        }

        MuJoCo_helper->SetRobotJointPositions(robot.name, jointPositions, d);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += static_cast<int>(robot.jointNames.size());
    }

    // Loop through all bodies in the state vector
    for(auto & bodiesState : active_state_vector.bodiesStates){
        // Get the body's position and orientation
        pose_6 bodyPose;
        MuJoCo_helper->GetBodyPoseAngle(bodiesState.name, bodyPose, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.activeLinearDOF[j]) {
                bodyPose.position[j] = position_vector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.activeAngularDOF[j]){
                bodyPose.orientation[j] = position_vector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }

        MuJoCo_helper->SetBodyPoseAngle(bodiesState.name, bodyPose, d);
    }

    return true;
}

bool ModelTranslator::setVelocityVector(MatrixXd velocity_vector, mjData* d){
    if(velocity_vector.rows() != (state_vector_size / 2)){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(auto & robot : active_state_vector.robots){
        vector<double> jointVelocities;

        for(int j = 0; j < robot.jointNames.size(); j++){
            jointVelocities.push_back(velocity_vector(j, 0));
        }
        
        MuJoCo_helper->SetRobotJointsVelocities(robot.name, jointVelocities, d);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += static_cast<int>(robot.jointNames.size());
    }


    // Loop through all bodies in the state vector
    for(auto & bodiesState : active_state_vector.bodiesStates){
        // Get the body's position and orientation
        pose_6 bodyVelocity;
        MuJoCo_helper->GetBodyVelocity(bodiesState.name, bodyVelocity, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.activeLinearDOF[j]) {
                bodyVelocity.position[j] = velocity_vector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.activeAngularDOF[j]){
                bodyVelocity.orientation[j] = velocity_vector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }

        MuJoCo_helper->SetBodyVelocity(bodiesState.name, bodyVelocity, d);
    }

    return true;
}

int ModelTranslator::StateIndexToQposIndex(int state_index){
    std::string state_name = state_vector_names[state_index];

    bool found_body_tag = false;
    int joint_index;
    int body_index_offset = 0;
    std::string joint_name;

    std::string body_tags[6]{"_x", "_y", "_z", "_roll", "_pitch", "_yaw"};
    for(int i = 0; i < 6; i++){
        found_body_tag = endsWith(state_name, body_tags[i]);

        if(found_body_tag){
            // Remove body tag from string
            state_name.erase(state_name.length() - body_tags[i].length(), body_tags[i].length());
            body_index_offset = i;
            break;
        }
    }

    if(found_body_tag){
        int bodyId = mj_name2id(MuJoCo_helper->model, mjOBJ_BODY, state_name.c_str());
        joint_index = MuJoCo_helper->model->jnt_dofadr[MuJoCo_helper->model->body_jntadr[bodyId]];
    }
    else{
        joint_index = mj_name2id(MuJoCo_helper->model, mjOBJ_JOINT, state_name.c_str());
    }

    return joint_index + body_index_offset;
}

MatrixXd ModelTranslator::StartStateVector() {
    MatrixXd start_state(state_vector_size, 1);

    MatrixXd position_vector(dof, 1);
    MatrixXd velocity_vector(dof, 1);

    int state_counter = 0;
    for(auto & robot : active_state_vector.robots){

        for(int j = 0; j < robot.jointNames.size(); j++){
            position_vector(state_counter, 0) = robot.startPos[j];
            state_counter++;
        }
    }

    // Loop through all bodies in the state vector
    for(auto & bodiesState : active_state_vector.bodiesStates){

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.activeLinearDOF[j]) {
                position_vector(state_counter, 0) = bodiesState.startLinearPos[j];
                state_counter++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.activeAngularDOF[j]){
                position_vector(state_counter, 0) = bodiesState.startAngularPos[j];
                state_counter++;
            }
        }
    }

    velocity_vector.setZero();

    start_state.block(0, 0, dof, 1) = position_vector;
    start_state.block(dof, 0, dof, 1) = velocity_vector;

    return start_state;
}

std::vector<MatrixXd> ModelTranslator::CreateInitOptimisationControls(int horizon_length) {
    std::vector<MatrixXd> initControls;

    for(int i = 0; i < horizon_length; i++){
        MatrixXd emptyControl(num_ctrl, 1);
        for(int j = 0; j < num_ctrl; j++){
            emptyControl(j) = 0.0f;
        }
        initControls.push_back(emptyControl);
    }

    return initControls;
}