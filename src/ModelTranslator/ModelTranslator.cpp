#include "ModelTranslator.h"

void ModelTranslator::InitModelTranslator(std::string yamlFilePath){
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
    for(int i = 0; i < taskConfig.robots.size(); i++){
        bodyNames.push_back(taskConfig.robots[i].name);
        for(int j = 0; j < taskConfig.robots[i].jointNames.size(); j++){
            jerk_thresholds.push_back(taskConfig.robots[i].jointJerkThresholds[j]);
            // TODO fix this duplicate jerk thresholds
            accel_thresholds.push_back(taskConfig.robots[i].jointJerkThresholds[j]);
            velocity_change_thresholds.push_back(taskConfig.robots[i].magVelThresholds[j]);
        }

    }

    for(int i = 0; i < taskConfig.bodiesStates.size(); i++){
        bodyNames.push_back(taskConfig.bodiesStates[i].name);
        for(int j = 0; j < 3; j++){
            jerk_thresholds.push_back(taskConfig.bodiesStates[i].linearJerkThreshold[j]);
            jerk_thresholds.push_back(taskConfig.bodiesStates[i].angularJerkThreshold[j]);

            // TODO fix this duplicate jerk thresholds
            accel_thresholds.push_back(taskConfig.bodiesStates[i].linearJerkThreshold[j]);
            accel_thresholds.push_back(taskConfig.bodiesStates[i].angularJerkThreshold[j]);

            velocity_change_thresholds.push_back(taskConfig.bodiesStates[i].linearMagVelThreshold[j]);
            velocity_change_thresholds.push_back(taskConfig.bodiesStates[i].angularMagVelThreshold[j]);
        }
    }

    MuJoCo_helper = std::make_shared<MuJoCoHelper>(taskConfig.robots, bodyNames);
    MuJoCo_helper->initSimulator(taskConfig.modelTimeStep, _modelPath);

    active_state_vector.robots = taskConfig.robots;
    active_state_vector.bodiesStates = taskConfig.bodiesStates;

    // --------- Set size of state vector correctly ------------
    state_vector_size = 0;
    for(int i = 0; i < active_state_vector.robots.size(); i++){
        state_vector_size += (2 * active_state_vector.robots[i].jointNames.size());
    }

    for(int i = 0; i < active_state_vector.bodiesStates.size(); i++){
        for(int j = 0; j < 3; j++){
            if(active_state_vector.bodiesStates[i].activeLinearDOF[j]){
                state_vector_size += 2;
            }
            if(active_state_vector.bodiesStates[i].activeAngularDOF[j]){
                state_vector_size += 2;
            }
        }
    }

    dof = state_vector_size / 2;
    X_desired.resize(state_vector_size, 1);
    X_start.resize(state_vector_size, 1);

    // --------- Set size of cost matrices correctly ------------
    num_ctrl = active_state_vector.robots[0].actuatorNames.size();
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
    for(int i = 0; i < active_state_vector.robots.size(); i++){
        int robotNumJoints = active_state_vector.robots[i].jointNames.size();

        // Loop through the robot joints
        for(int j = 0; j < robotNumJoints; j++){
            Q.diagonal()[Q_index + j, Q_index + j] = active_state_vector.robots[i].jointPosCosts[j];
            Q_terminal.diagonal()[Q_index + j, Q_index + j] = active_state_vector.robots[i].terminalJointPosCosts[j];

            Q.diagonal()[Q_index + j + dof, Q_index + j + dof] = active_state_vector.robots[i].jointVelCosts[j];
            Q_terminal.diagonal()[Q_index + j + dof, Q_index + j + dof] = active_state_vector.robots[i].terminalJointVelCosts[j];

            X_desired(Q_index + j, 0) = active_state_vector.robots[i].goalPos[j];
            X_desired(Q_index + j + dof, 0) = 0.0f;

            X_start(Q_index + j, 0) = active_state_vector.robots[i].startPos[j];
            X_start(Q_index + j + dof, 0) = 0.0f;
        }
        Q_index += robotNumJoints;
    }

    // Loop through bodies
    for(int i = 0; i < active_state_vector.bodiesStates.size(); i++){
        
        int activeDOFs = 0;
        for(int j = 0; j < 3; j++){
            if(active_state_vector.bodiesStates[i].activeLinearDOF[j]){
                activeDOFs++;
            }
            if(active_state_vector.bodiesStates[i].activeAngularDOF[j]){
                activeDOFs++;
            }
        }

        // Loop through linear states first
        int activeDofCounter = 0;
        for(int j = 0; j < 3; j++){
            if(active_state_vector.bodiesStates[i].activeLinearDOF[j]){
                Q.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = active_state_vector.bodiesStates[i].linearPosCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = active_state_vector.bodiesStates[i].terminalLinearPosCost[j];

                Q.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = active_state_vector.bodiesStates[i].linearVelCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = active_state_vector.bodiesStates[i].terminalLinearVelCost[j];

                X_desired(Q_index + j, 0) = active_state_vector.bodiesStates[i].goalLinearPos[j];
                X_desired(Q_index + j + dof, 0) = 0.0f;

                X_start(Q_index + j, 0) = active_state_vector.bodiesStates[i].startLinearPos[j];
                X_start(Q_index + j + dof, 0) = 0.0f;

                activeDofCounter++;
            }
        }

        // Loop through angular states second
        for(int j = 0; j < 3; j++){
            if(active_state_vector.bodiesStates[i].activeAngularDOF[j]){
                Q.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = active_state_vector.bodiesStates[i].angularPosCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = active_state_vector.bodiesStates[i].terminalAngularPosCost[j];

                Q.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = active_state_vector.bodiesStates[i].angularVelCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = active_state_vector.bodiesStates[i].terminalAngularVelCost[j];

                X_desired(Q_index + j, 0) = active_state_vector.bodiesStates[i].goalAngularPos[j];
                X_desired(Q_index + j + dof, 0) = 0.0f;

                X_start(Q_index + j, 0) = active_state_vector.bodiesStates[i].startAngularPos[j];
                X_start(Q_index + j + dof, 0) = 0.0f;

                activeDofCounter++;
            }
        }
        Q_index += activeDOFs;
    }

    // Loop through robots and starting assigning control specific costs
    int R_index = 0;
    for(int i = 0; i < active_state_vector.robots.size(); i++){
        int robotNumActuators = active_state_vector.robots[i].actuatorNames.size();

        // Loop through the robot joints
        for(int j = 0; j < robotNumActuators; j++){
            R.diagonal()[R_index + j, R_index + j] = active_state_vector.robots[i].jointControlCosts[j];
        }

        R_index += robotNumActuators;
    }
    // ----------------------------------------------------------------------------------------------

    cout << "Q: " << Q.diagonal().transpose() << std::endl;
    cout << "R: " << R.diagonal().transpose() << std::endl;
    cout << "Q_terminal: " << Q_terminal.diagonal() << endl;
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
    X_desired.resize(dof * 2, 1);
    X_start.resize(dof * 2, 1);

    // TODO (DMackRus) - think there is a better way to do this if i rewrite how my state vector is stored
    for(int robot = 0; robot < active_state_vector.robots.size(); robot++){
        for(int joint = 0; joint < active_state_vector.robots[robot].jointNames.size(); joint++){

            for(int i = 0; i < state_vector_names.size(); i++){
                // TODO (DMackRus) - Need to add ability to activate / deactivate joints from state vector
            }
        }
    }

    for(int body = 0; body < active_state_vector.bodiesStates.size(); body++){
        std::string body_name = active_state_vector.bodiesStates[body].name;
        for(int i = 0; i < state_vector_names.size(); i++){
            size_t found = state_vector_names[i].find(body_name);

            if (found != std::string::npos) {
                state_vector_names[i].erase(found, body_name.length());
                if(state_vector_names[i] == "_x"){
                    active_state_vector.bodiesStates[body].activeLinearDOF[0] = add_extra_states;
                }
                else if(state_vector_names[i] == "_y"){
                    active_state_vector.bodiesStates[body].activeLinearDOF[1] = add_extra_states;
                }
                else if(state_vector_names[i] == "_z"){
                    active_state_vector.bodiesStates[body].activeLinearDOF[2] = add_extra_states;
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
    for(int i = 0; i < active_state_vector.robots.size(); i++){
        int robotNumJoints = active_state_vector.robots[i].jointNames.size();

        // Loop through the robot joints
        for(int j = 0; j < robotNumJoints; j++){
            Q.diagonal()[Q_index + j, Q_index + j] = active_state_vector.robots[i].jointPosCosts[j];
            Q_terminal.diagonal()[Q_index + j, Q_index + j] = active_state_vector.robots[i].terminalJointPosCosts[j];

            Q.diagonal()[Q_index + j + dof, Q_index + j + dof] = active_state_vector.robots[i].jointVelCosts[j];
            Q_terminal.diagonal()[Q_index + j + dof, Q_index + j + dof] = active_state_vector.robots[i].terminalJointVelCosts[j];

            X_desired(Q_index + j, 0) = active_state_vector.robots[i].goalPos[j];
            X_desired(Q_index + j + dof, 0) = 0.0f;

            X_start(Q_index + j, 0) = active_state_vector.robots[i].startPos[j];
            X_start(Q_index + j + dof, 0) = 0.0f;
        }
        Q_index += robotNumJoints;
    }

    // Loop through bodies
    for(int i = 0; i < active_state_vector.bodiesStates.size(); i++){

        int activeDOFs = 0;
        for(int j = 0; j < 3; j++){
            if(active_state_vector.bodiesStates[i].activeLinearDOF[j]){
                activeDOFs++;
            }
            if(active_state_vector.bodiesStates[i].activeAngularDOF[j]){
                activeDOFs++;
            }
        }

        // Loop through linear states first
        int activeDofCounter = 0;
        for(int j = 0; j < 3; j++){
            if(active_state_vector.bodiesStates[i].activeLinearDOF[j]){
                Q.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = active_state_vector.bodiesStates[i].linearPosCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = active_state_vector.bodiesStates[i].terminalLinearPosCost[j];

                Q.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = active_state_vector.bodiesStates[i].linearVelCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = active_state_vector.bodiesStates[i].terminalLinearVelCost[j];

                X_desired(Q_index + j, 0) = active_state_vector.bodiesStates[i].goalLinearPos[j];
                X_desired(Q_index + j + dof, 0) = 0.0f;

                X_start(Q_index + j, 0) = active_state_vector.bodiesStates[i].startLinearPos[j];
                X_start(Q_index + j + dof, 0) = 0.0f;

                activeDofCounter++;
            }
        }

        // Loop through angular states second
        for(int j = 0; j < 3; j++){
            if(active_state_vector.bodiesStates[i].activeAngularDOF[j]){
                Q.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = active_state_vector.bodiesStates[i].angularPosCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = active_state_vector.bodiesStates[i].terminalAngularPosCost[j];

                Q.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = active_state_vector.bodiesStates[i].angularVelCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = active_state_vector.bodiesStates[i].terminalAngularVelCost[j];

                X_desired(Q_index + j, 0) = active_state_vector.bodiesStates[i].goalAngularPos[j];
                X_desired(Q_index + j + dof, 0) = 0.0f;

                X_start(Q_index + j, 0) = active_state_vector.bodiesStates[i].startAngularPos[j];
                X_start(Q_index + j + dof, 0) = 0.0f;

                activeDofCounter++;
            }
        }
        Q_index += activeDOFs;
    }
}

std::vector<std::string> ModelTranslator::GetStateVectorNames(){
    std::vector<std::string> state_vector_names;

    for(int robot = 0; robot < active_state_vector.robots.size(); robot++){
        for(int joint = 0; joint < active_state_vector.robots[robot].jointNames.size(); joint++){
            // TODO (DMackRus) - Need to add ability for joints not to be automatically included in state vector?
            state_vector_names.push_back(active_state_vector.robots[robot].jointNames[joint]);
        }
    }

    for(int body = 0; body < active_state_vector.bodiesStates.size(); body++){
        if(active_state_vector.bodiesStates[body].activeLinearDOF[0]){
            state_vector_names.push_back(active_state_vector.bodiesStates[body].name + "_x");
        }

        if(active_state_vector.bodiesStates[body].activeLinearDOF[1]){
            state_vector_names.push_back(active_state_vector.bodiesStates[body].name + "_y");
        }

        if(active_state_vector.bodiesStates[body].activeLinearDOF[2]){
            state_vector_names.push_back(active_state_vector.bodiesStates[body].name + "_z");
        }

        if(active_state_vector.bodiesStates[body].activeAngularDOF[0]){
            state_vector_names.push_back(active_state_vector.bodiesStates[body].name + "_roll");
        }

        if(active_state_vector.bodiesStates[body].activeAngularDOF[1]){
            state_vector_names.push_back(active_state_vector.bodiesStates[body].name + "_pitch");
        }

        if(active_state_vector.bodiesStates[body].activeAngularDOF[2]){
            state_vector_names.push_back(active_state_vector.bodiesStates[body].name + "_yaw");
        }
    }

    return state_vector_names;
}

double ModelTranslator::CostFunction(mjData* d, bool terminal){
    double cost;
    MatrixXd Xt = ReturnStateVector(d);
    MatrixXd Ut = ReturnControlVector(d);
//    cout << "X_desired: " << X_desired << endl;

    MatrixXd X_diff = Xt - X_desired;
    MatrixXd temp;

    if(terminal){
        temp = ((X_diff.transpose() * Q_terminal * X_diff)) + (Ut.transpose() * R * Ut);
    }
    else{
        temp = ((X_diff.transpose() * Q * X_diff)) + (Ut.transpose() * R * Ut);
    }

    cost = temp(0);

    return cost;
}

void ModelTranslator::CostDerivatives(mjData* d, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal){
    MatrixXd Xt = ReturnStateVector(d);
    MatrixXd Ut = ReturnControlVector(d);

    MatrixXd X_diff = Xt - X_desired;

    // Size cost derivatives appropriately
    l_x.resize(state_vector_size, 1);
    l_xx.resize(state_vector_size, state_vector_size);

    l_u.resize(num_ctrl, 1);
    l_uu.resize(num_ctrl, num_ctrl);

    if(terminal){
        l_x = 2 * Q_terminal * X_diff;
        l_xx = 2 * Q_terminal;
    }
    else{
        l_x = 2 * Q * X_diff;
        l_xx = 2 * Q;
    }    

    l_u = 2 * R * Ut;
    l_uu = 2 * R;

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
    MatrixXd stateVector(state_vector_size, 1);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < active_state_vector.robots.size(); i++){
        vector<double> jointPositions;
        vector<double> jointVelocities;
        MuJoCo_helper->getRobotJointsPositions(active_state_vector.robots[i].name, jointPositions, d);
        MuJoCo_helper->getRobotJointsVelocities(active_state_vector.robots[i].name, jointVelocities, d);

        for(int j = 0; j < active_state_vector.robots[i].jointNames.size(); j++){
            stateVector(j, 0) = jointPositions[j];
            stateVector(j + (state_vector_size / 2), 0) = jointVelocities[j];
        }

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += active_state_vector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < active_state_vector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyPose;
        pose_6 bodyVelocity;
        MuJoCo_helper->getBodyPose_angle(active_state_vector.bodiesStates[i].name, bodyPose, d);
        MuJoCo_helper->getBodyVelocity(active_state_vector.bodiesStates[i].name, bodyVelocity, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (active_state_vector.bodiesStates[i].activeLinearDOF[j]) {
                stateVector(currentStateIndex, 0) = bodyPose.position[j];
                stateVector(currentStateIndex + (state_vector_size / 2), 0) = bodyVelocity.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(active_state_vector.bodiesStates[i].activeAngularDOF[j]){
                stateVector(currentStateIndex, 0) = bodyPose.orientation[j];
                stateVector(currentStateIndex + (state_vector_size / 2), 0) = bodyVelocity.orientation[j];
                currentStateIndex++;
            }
        }
    }

    return stateVector;
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
    for(int i = 0; i < active_state_vector.robots.size(); i++){
        vector<double> jointControls;
        MuJoCo_helper->getRobotJointsControls(active_state_vector.robots[i].name, jointControls, d);
        for(int j = 0; j < active_state_vector.robots[i].actuatorNames.size(); j++){

            controlVector(currentStateIndex + j, 0) = jointControls[j];
        }

        currentStateIndex += active_state_vector.robots[i].actuatorNames.size();
    }

    return controlVector;
}

bool ModelTranslator::SetControlVector(MatrixXd control_vector, mjData* d){
    if(control_vector.rows() != num_ctrl){
        cout << "ERROR: control vector size does not match the size of the control vector in the model translator" << endl;
        return false;
    }

    int currentStateIndex = 0;

    // loop through all the present robots
    for(int i = 0; i < active_state_vector.robots.size(); i++){
        vector<double> jointControls;
        for(int j = 0; j < active_state_vector.robots[i].actuatorNames.size(); j++){

            jointControls.push_back(control_vector(currentStateIndex + j));
        }

        MuJoCo_helper->setRobotJointsControls(active_state_vector.robots[i].name, jointControls, d);

        currentStateIndex += active_state_vector.robots[i].actuatorNames.size();
    }

    return true;
}

MatrixXd ModelTranslator::returnPositionVector(mjData* d){
    MatrixXd position_vector(dof, 1);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < active_state_vector.robots.size(); i++){
        vector<double> jointPositions;
        MuJoCo_helper->getRobotJointsPositions(active_state_vector.robots[i].name, jointPositions, d);

        for(int j = 0; j < active_state_vector.robots[i].jointNames.size(); j++){
            position_vector(j, 0) = jointPositions[j];
        }

        // Increment the current state index by the number of joints in the robot
        currentStateIndex += active_state_vector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < active_state_vector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyPose;
        MuJoCo_helper->getBodyPose_angle(active_state_vector.bodiesStates[i].name, bodyPose, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (active_state_vector.bodiesStates[i].activeLinearDOF[j]) {
                position_vector(currentStateIndex, 0) = bodyPose.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(active_state_vector.bodiesStates[i].activeAngularDOF[j]){
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
    for(int i = 0; i < active_state_vector.robots.size(); i++){
        vector<double> jointVelocities;
        MuJoCo_helper->getRobotJointsVelocities(active_state_vector.robots[i].name, jointVelocities, d);

        for(int j = 0; j < active_state_vector.robots[i].jointNames.size(); j++){
            velocity_vector(j, 0) = jointVelocities[j];
        }

        // Increment the current state index by the number of joints in the robot
        currentStateIndex += active_state_vector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < active_state_vector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyVelocities;
        MuJoCo_helper->getBodyVelocity(active_state_vector.bodiesStates[i].name, bodyVelocities, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (active_state_vector.bodiesStates[i].activeLinearDOF[j]) {
                velocity_vector(currentStateIndex, 0) = bodyVelocities.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(active_state_vector.bodiesStates[i].activeAngularDOF[j]){
                velocity_vector(currentStateIndex, 0) = bodyVelocities.orientation[j];
                currentStateIndex++;
            }
        }
    }

    return velocity_vector;
}

MatrixXd ModelTranslator::returnAccelerationVector(mjData* d){
    MatrixXd accel_vector(dof, 1);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < active_state_vector.robots.size(); i++){
        vector<double> jointAccelerations;
        MuJoCo_helper->getRobotJointsAccelerations(active_state_vector.robots[i].name, jointAccelerations, d);

        for(int j = 0; j < active_state_vector.robots[i].jointNames.size(); j++){
            accel_vector(j, 0) = jointAccelerations[j];
        }

        // Increment the current state index by the number of joints in the robot
        currentStateIndex += active_state_vector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < active_state_vector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyAccelerations;
        MuJoCo_helper->getBodyAcceleration(active_state_vector.bodiesStates[i].name, bodyAccelerations, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (active_state_vector.bodiesStates[i].activeLinearDOF[j]) {
                accel_vector(currentStateIndex, 0) = bodyAccelerations.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(active_state_vector.bodiesStates[i].activeAngularDOF[j]){
                accel_vector(currentStateIndex, 0) = bodyAccelerations.orientation[j];
                currentStateIndex++;
            }
        }
    }

    return accel_vector;
}

bool ModelTranslator::setPositionVector(MatrixXd position_vector, mjData* d){
    if(position_vector.rows() != (state_vector_size / 2)){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < active_state_vector.robots.size(); i++){
        vector<double> jointPositions;

        for(int j = 0; j < active_state_vector.robots[i].jointNames.size(); j++){
            jointPositions.push_back(position_vector(j, 0));
        }

        MuJoCo_helper->setRobotJointsPositions(active_state_vector.robots[i].name, jointPositions, d);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += active_state_vector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < active_state_vector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyPose;

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (active_state_vector.bodiesStates[i].activeLinearDOF[j]) {
                bodyPose.position[j] = position_vector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(active_state_vector.bodiesStates[i].activeAngularDOF[j]){
                bodyPose.orientation[j] = position_vector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }

        MuJoCo_helper->setBodyPose_angle(active_state_vector.bodiesStates[i].name, bodyPose, d);
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
    for(int i = 0; i < active_state_vector.robots.size(); i++){
        vector<double> jointVelocities;

        for(int j = 0; j < active_state_vector.robots[i].jointNames.size(); j++){
            jointVelocities.push_back(velocity_vector(j, 0));
        }
        
        MuJoCo_helper->setRobotJointsVelocities(active_state_vector.robots[i].name, jointVelocities, d);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += active_state_vector.robots[i].jointNames.size();
    }


    // Loop through all bodies in the state vector
    for(int i = 0; i < active_state_vector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyVelocity;

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (active_state_vector.bodiesStates[i].activeLinearDOF[j]) {
                bodyVelocity.position[j] = velocity_vector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(active_state_vector.bodiesStates[i].activeAngularDOF[j]){
                bodyVelocity.orientation[j] = velocity_vector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }

        MuJoCo_helper->setBodyVelocity(active_state_vector.bodiesStates[i].name, bodyVelocity, d);
    }

    return true;
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