#include "ModelTranslator.h"

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
    MuJoCo_helper->initSimulator(taskConfig.modelTimeStep, _modelPath);

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
    X_start.resize(state_vector_size, 1);

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

            X_start(Q_index + j, 0) = robot.startPos[j];
            X_start(Q_index + j + dof, 0) = 0.0f;
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

                X_start(Q_index + activeDofCounter, 0) = bodiesState.startLinearPos[j];
                X_start(Q_index + activeDofCounter + dof, 0) = 0.0f;

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

                X_start(Q_index + activeDofCounter, 0) = bodiesState.startAngularPos[j];
                X_start(Q_index + activeDofCounter + dof, 0) = 0.0f;

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
    X_start.resize(dof * 2, 1);

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

            X_start(Q_index + j, 0) = robot.startPos[j];
            X_start(Q_index + j + dof, 0) = 0.0f;
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

                X_start(Q_index + j, 0) = bodiesState.startLinearPos[j];
                X_start(Q_index + j + dof, 0) = 0.0f;

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

                X_start(Q_index + j, 0) = bodiesState.startAngularPos[j];
                X_start(Q_index + j + dof, 0) = 0.0f;

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
//    MatrixXd Xt = ReturnStateVector(d);
//    MatrixXd Ut = ReturnControlVector(d);
//
//    MatrixXd X_diff = Xt - X_desired;
//    MatrixXd temp;
//
//    if(terminal){
//        temp = ((X_diff.transpose() * Q_terminal * X_diff)) + (Ut.transpose() * R * Ut);
//    }
//    else{
//        temp = ((X_diff.transpose() * Q * X_diff)) + (Ut.transpose() * R * Ut);
//    }
//    cost = temp(0);

    // Loop through position part of vector
    for(auto & robot : active_state_vector.robots){
        int robot_num_joints = static_cast<int>(robot.jointNames.size());

        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        MuJoCo_helper->getRobotJointsPositions(robot.name, joint_positions, d);
        MuJoCo_helper->getRobotJointsVelocities(robot.name, joint_velocities, d);

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

            // Cost += joint pos cost * (current pos - desired pos)^2
            cost += cost_pos * pow((joint_positions[j] - robot.goalPos[j]), 2);

            // Cost += joint vel cost * (current vel - desired vel)^2
            // TODO - this assumes we want zero velocity, will break walker model
            cost += cost_vel * pow(joint_velocities[j], 2);
        }
    }

    // Loop through the bodies in simulation
    for(auto body : active_state_vector.bodiesStates){

        pose_6 body_pose;
        pose_6 body_vel;
        MuJoCo_helper->getBodyPose_angle(body.name, body_pose, d);
        MuJoCo_helper->getBodyVelocity(body.name, body_vel, d);

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

        // Angular costs
        // Convert pose.orientation to quaternion
        // compute dot prodict between desired orientation and current orientation
        for(int j = 0; j < 3; j++){

        }

    }

    return cost;
}

void ModelTranslator::CostDerivatives(mjData* d, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal){
//    MatrixXd Xt = ReturnStateVector(d);
//    MatrixXd Ut = ReturnControlVector(d);
//
//    MatrixXd X_diff = Xt - X_desired;

    // Size cost derivatives appropriately
    l_x.resize(state_vector_size, 1);
    l_xx.resize(state_vector_size, state_vector_size);

    l_u.resize(num_ctrl, 1);
    l_uu.resize(num_ctrl, num_ctrl);

    // TODO - remove as this is temporary
    l_u.setZero();
    l_uu.setZero();

    int Q_index = 0;
    for(auto & robot : active_state_vector.robots){
        int robot_num_joints = static_cast<int>(robot.jointNames.size());

        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        MuJoCo_helper->getRobotJointsPositions(robot.name, joint_positions, d);
        MuJoCo_helper->getRobotJointsVelocities(robot.name, joint_velocities, d);

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

            // l_x = 2 * cost * diff
            // position and velocity
            l_x(Q_index + j) = 2 * cost_pos * (joint_positions[j] - robot.goalPos[j]);
            // TODO - assumes we alsways want velocity = 0
            l_x(Q_index + j + dof) = 2 * cost_vel * (joint_velocities[j]);

            // l_u = 2 * cost * diff
            // TODO - implement

            // l_xx = 2 * cost
            l_xx(Q_index + j, Q_index + j) = 2 * cost_pos;
            l_xx(Q_index + j + dof, Q_index + j + dof) = 2 * cost_vel;

            // l_uu = 2 * cost
            // TODO - implement

        }

        Q_index += robot_num_joints;
    }

    for(auto body : active_state_vector.bodiesStates){
        pose_6 body_pose;
        pose_6 body_vel;
        MuJoCo_helper->getBodyPose_angle(body.name, body_pose, d);
        MuJoCo_helper->getBodyVelocity(body.name, body_vel, d);

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

        // Angular cost derivatives
        for(int j = 0; j < 3; j++) {
            if(body.activeAngularDOF[j]){

            }
        }
    }
//    std::cout << l_xx << std::endl;
//    std::cout << "l_x \n" << l_x << "\n";

//    if(terminal){
//        l_x = 2 * Q_terminal * X_diff;
//        l_xx = 2 * Q_terminal;
//    }
//    else{
//        l_x = 2 * Q * X_diff;
//        l_xx = 2 * Q;
//    }
//
////    std::cout << "old l_x: " << l_x << "\n";
//
//    l_u = 2 * R * Ut;
//    l_uu = 2 * R;

}

bool ModelTranslator::TaskComplete(mjData* d, double &dist){
    return false;
}

std::vector<MatrixXd> ModelTranslator::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> emptyInitSetupControls;
    MuJoCo_helper->copySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
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
        MuJoCo_helper->getRobotJointsControls(robot.name, jointControls, d);
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
        MuJoCo_helper->getRobotControlLimits(robot.name, robot_control_limits);
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

        MuJoCo_helper->setRobotJointsControls(robot.name, jointControls, d);

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
        MuJoCo_helper->getRobotJointsPositions(robot.name, jointPositions, d);

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
        MuJoCo_helper->getBodyPose_angle(bodiesState.name, bodyPose, d);

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
        MuJoCo_helper->getRobotJointsVelocities(robot.name, jointVelocities, d);

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
        MuJoCo_helper->getBodyVelocity(bodiesState.name, bodyVelocities, d);

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

        MuJoCo_helper->setRobotJointsPositions(robot.name, jointPositions, d);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += static_cast<int>(robot.jointNames.size());
    }

    // Loop through all bodies in the state vector
    for(auto & bodiesState : active_state_vector.bodiesStates){
        // Get the body's position and orientation
        pose_6 bodyPose;
        MuJoCo_helper->getBodyPose_angle(bodiesState.name, bodyPose, d);

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

        MuJoCo_helper->setBodyPose_angle(bodiesState.name, bodyPose, d);
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
        
        MuJoCo_helper->setRobotJointsVelocities(robot.name, jointVelocities, d);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += static_cast<int>(robot.jointNames.size());
    }


    // Loop through all bodies in the state vector
    for(auto & bodiesState : active_state_vector.bodiesStates){
        // Get the body's position and orientation
        pose_6 bodyVelocity;
        MuJoCo_helper->getBodyVelocity(bodiesState.name, bodyVelocity, d);

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

        MuJoCo_helper->setBodyVelocity(bodiesState.name, bodyVelocity, d);
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

    int state_counter = 0;
    for(auto & robot : active_state_vector.robots){

        for(int j = 0; j < robot.jointNames.size(); j++){
            start_state(state_counter, 0) = robot.startPos[j];
            state_counter++;
        }
    }

    // Loop through all bodies in the state vector
    for(auto & bodiesState : active_state_vector.bodiesStates){

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.activeLinearDOF[j]) {
                start_state(state_counter, 0) = bodiesState.startLinearPos[j];
                state_counter++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.activeAngularDOF[j]){
                start_state(state_counter, 0) = bodiesState.startAngularPos[j];
                state_counter++;
            }
        }
    }

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