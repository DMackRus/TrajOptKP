#include "ModelTranslator.h"

ModelTranslator::ModelTranslator(){

}

void ModelTranslator::InitModelTranslator(std::string yamlFilePath){
    task taskConfig;

    FileHandler yamlReader;
    yamlReader.readModelConfigFile(yamlFilePath, taskConfig);
    model_file_path = taskConfig.modelFilePath;
    model_name = taskConfig.modelName;
    min_N = taskConfig.minN;
    max_N = taskConfig.maxN;
    keypoint_method = taskConfig.keypointMethod;
    iterative_error_threshold = taskConfig.iterativeErrorThreshold;
    const char* _modelPath = model_file_path.c_str();

    // Initialise physics simulator
    vector<string> bodyNames;
    for(int i = 0; i < taskConfig.robots.size(); i++){
        bodyNames.push_back(taskConfig.robots[i].name);
        for(int j = 0; j < taskConfig.robots[i].jointNames.size(); j++){
            jerk_thresholds.push_back(taskConfig.robots[i].jointJerkThresholds[j]);
            // TODO fix this dupliate jerk thresholds
            accel_thresholds.push_back(taskConfig.robots[i].jointJerkThresholds[j]);
            velocity_change_thresholds.push_back(taskConfig.robots[i].magVelThresholds[j]);
        }

    }

    for(int i = 0; i < taskConfig.bodiesStates.size(); i++){
        bodyNames.push_back(taskConfig.bodiesStates[i].name);
        for(int j = 0; j < 3; j++){
            jerk_thresholds.push_back(taskConfig.bodiesStates[i].linearJerkThreshold[j]);
            jerk_thresholds.push_back(taskConfig.bodiesStates[i].angularJerkThreshold[j]);

            // TODO fix this dupliate jerk thresholds
            accel_thresholds.push_back(taskConfig.bodiesStates[i].linearJerkThreshold[j]);
            accel_thresholds.push_back(taskConfig.bodiesStates[i].angularJerkThreshold[j]);

            velocity_change_thresholds.push_back(taskConfig.bodiesStates[i].linearMagVelThreshold[j]);
            velocity_change_thresholds.push_back(taskConfig.bodiesStates[i].angularMagVelThreshold[j]);
        }
    }

    mujoco_helper = std::make_shared<MuJoCoHelper>(taskConfig.robots, bodyNames);
    active_physics_simulator = mujoco_helper;
    active_physics_simulator->initSimulator(taskConfig.modelTimeStep, _modelPath);

    state_vector.robots = taskConfig.robots;
    state_vector.bodiesStates = taskConfig.bodiesStates;

    // --------- Set size of state vector correctly ------------
    state_vector_size = 0;
    for(int i = 0; i < state_vector.robots.size(); i++){
        state_vector_size += (2 * state_vector.robots[i].jointNames.size());
    }

    for(int i = 0; i < state_vector.bodiesStates.size(); i++){
        for(int j = 0; j < 3; j++){
            if(state_vector.bodiesStates[i].activeLinearDOF[j]){
                state_vector_size += 2;
            }
            if(state_vector.bodiesStates[i].activeAngularDOF[j]){
                state_vector_size += 2;
            }
        }
    }

    dof = state_vector_size / 2;
    X_desired.resize(state_vector_size, 1);
    X_start.resize(state_vector_size, 1);

    // --------- Set size of cost matrices correctly ------------
    num_ctrl = state_vector.robots[0].actuatorNames.size();
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
    for(int i = 0; i < state_vector.robots.size(); i++){
        int robotNumJoints = state_vector.robots[i].jointNames.size();

        // Loop through the robot joints
        for(int j = 0; j < robotNumJoints; j++){
            Q.diagonal()[Q_index + j, Q_index + j] = state_vector.robots[i].jointPosCosts[j];
            Q_terminal.diagonal()[Q_index + j, Q_index + j] = state_vector.robots[i].terminalJointPosCosts[j];

            Q.diagonal()[Q_index + j + dof, Q_index + j + dof] = state_vector.robots[i].jointVelCosts[j];
            Q_terminal.diagonal()[Q_index + j + dof, Q_index + j + dof] = state_vector.robots[i].terminalJointVelCosts[j];

            X_desired(Q_index + j, 0) = state_vector.robots[i].goalPos[j];
            X_desired(Q_index + j + dof, 0) = 0.0f;

            X_start(Q_index + j, 0) = state_vector.robots[i].startPos[j];
            X_start(Q_index + j + dof, 0) = 0.0f;
        }
        Q_index += robotNumJoints;
    }

    // Loop through bodies
    for(int i = 0; i < state_vector.bodiesStates.size(); i++){
        
        int activeDOFs = 0;
        for(int j = 0; j < 3; j++){
            if(state_vector.bodiesStates[i].activeLinearDOF[j]){
                activeDOFs++;
            }
            if(state_vector.bodiesStates[i].activeAngularDOF[j]){
                activeDOFs++;
            }
        }

        // Loop through linear states first
        int activeDofCounter = 0;
        for(int j = 0; j < 3; j++){
            if(state_vector.bodiesStates[i].activeLinearDOF[j]){
                Q.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = state_vector.bodiesStates[i].linearPosCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = state_vector.bodiesStates[i].terminalLinearPosCost[j];

                Q.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = state_vector.bodiesStates[i].linearVelCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = state_vector.bodiesStates[i].terminalLinearVelCost[j];

                X_desired(Q_index + j, 0) = state_vector.bodiesStates[i].goalLinearPos[j];
                X_desired(Q_index + j + dof, 0) = 0.0f;

                X_start(Q_index + j, 0) = state_vector.bodiesStates[i].startLinearPos[j];
                X_start(Q_index + j + dof, 0) = 0.0f;

                activeDofCounter++;
            }
        }

        // Loop through angular states second
        for(int j = 0; j < 3; j++){
            if(state_vector.bodiesStates[i].activeAngularDOF[j]){
                Q.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = state_vector.bodiesStates[i].angularPosCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = state_vector.bodiesStates[i].terminalAngularPosCost[j];

                Q.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = state_vector.bodiesStates[i].angularVelCost[j];
                Q_terminal.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = state_vector.bodiesStates[i].terminalAngularVelCost[j];

                X_desired(Q_index + j, 0) = state_vector.bodiesStates[i].goalAngularPos[j];
                X_desired(Q_index + j + dof, 0) = 0.0f;

                X_start(Q_index + j, 0) = state_vector.bodiesStates[i].startAngularPos[j];
                X_start(Q_index + j + dof, 0) = 0.0f;

                activeDofCounter++;
            }
        }
        Q_index += activeDOFs;
    }

    // Loop through robots and starting assinging control specific costs
    int R_index = 0;
    for(int i = 0; i < state_vector.robots.size(); i++){
        int robotNumActuators = state_vector.robots[i].actuatorNames.size();

        // Loop through the robot joints
        for(int j = 0; j < robotNumActuators; j++){
            R.diagonal()[R_index + j, R_index + j] = state_vector.robots[i].jointControlCosts[j];
        }

        R_index += robotNumActuators;
    }
    // ----------------------------------------------------------------------------------------------

    cout << "Q: " << Q.diagonal() << std::endl;
    cout << "R: " << R.diagonal() << std::endl;
    cout << "Q_terminal: " << Q_terminal.diagonal() << endl;
}

double ModelTranslator::CostFunction(int data_index, bool terminal){
    double cost;
    MatrixXd Xt = ReturnStateVector(data_index);
    MatrixXd Ut = ReturnControlVector(data_index);

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

void ModelTranslator::CostDerivatives(int data_index, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal){
    MatrixXd Xt = ReturnStateVector(data_index);
    MatrixXd Ut = ReturnControlVector(data_index);

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

bool ModelTranslator::TaskComplete(int data_index, double &dist){
    return false;
}

std::vector<MatrixXd> ModelTranslator::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> emptyInitSetupControls;
    active_physics_simulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
    return emptyInitSetupControls;
}

MatrixXd ModelTranslator::ReturnStateVector(int data_index){
    MatrixXd stateVector(state_vector_size, 1);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < state_vector.robots.size(); i++){
        vector<double> jointPositions;
        vector<double> jointVelocities;
        active_physics_simulator->getRobotJointsPositions(state_vector.robots[i].name, jointPositions, data_index);
        active_physics_simulator->getRobotJointsVelocities(state_vector.robots[i].name, jointVelocities, data_index);

        for(int j = 0; j < state_vector.robots[i].jointNames.size(); j++){
            stateVector(j, 0) = jointPositions[j];
            stateVector(j + (state_vector_size / 2), 0) = jointVelocities[j];
        }

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += state_vector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < state_vector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyPose;
        pose_6 bodyVelocity;
        active_physics_simulator->getBodyPose_angle(state_vector.bodiesStates[i].name, bodyPose, data_index);
        active_physics_simulator->getBodyVelocity(state_vector.bodiesStates[i].name, bodyVelocity, data_index);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (state_vector.bodiesStates[i].activeLinearDOF[j]) {
                stateVector(currentStateIndex, 0) = bodyPose.position[j];
                stateVector(currentStateIndex + (state_vector_size / 2), 0) = bodyVelocity.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(state_vector.bodiesStates[i].activeAngularDOF[j]){
                stateVector(currentStateIndex, 0) = bodyPose.orientation[j];
                stateVector(currentStateIndex + (state_vector_size / 2), 0) = bodyVelocity.orientation[j];
                currentStateIndex++;
            }
        }
    }

    return stateVector;
}

bool ModelTranslator::SetStateVector(MatrixXd state_vector, int data_index){

    if(state_vector.rows() != state_vector_size){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    MatrixXd position_vector(state_vector_size / 2, 1);
    MatrixXd velocity_vector(state_vector_size / 2, 1);

    position_vector = state_vector.block(0, 0, state_vector_size / 2, 1);
    velocity_vector = state_vector.block(state_vector_size / 2, 0, state_vector_size / 2, 1);

    setPositionVector(position_vector, data_index);
    setVelocityVector(velocity_vector, data_index);

    return true;
}

MatrixXd ModelTranslator::ReturnControlVector(int data_index){
    MatrixXd controlVector(num_ctrl, 1);
    int currentStateIndex = 0;

    // loop through all the present robots
    for(int i = 0; i < state_vector.robots.size(); i++){
        vector<double> jointControls;
        active_physics_simulator->getRobotJointsControls(state_vector.robots[i].name, jointControls, data_index);
        for(int j = 0; j < state_vector.robots[i].actuatorNames.size(); j++){

            controlVector(currentStateIndex + j, 0) = jointControls[j];
        }

        currentStateIndex += state_vector.robots[i].actuatorNames.size();
    }

    return controlVector;
}

bool ModelTranslator::SetControlVector(MatrixXd control_vector, int data_index){
    if(control_vector.rows() != num_ctrl){
        cout << "ERROR: control vector size does not match the size of the control vector in the model translator" << endl;
        return false;
    }

    int currentStateIndex = 0;

    // loop through all the present robots
    for(int i = 0; i < state_vector.robots.size(); i++){
        vector<double> jointControls;
        for(int j = 0; j < state_vector.robots[i].actuatorNames.size(); j++){

            jointControls.push_back(control_vector(currentStateIndex + j));
        }

        active_physics_simulator->setRobotJointsControls(state_vector.robots[i].name, jointControls, data_index);

        currentStateIndex += state_vector.robots[i].actuatorNames.size();
    }

    return true;
}

MatrixXd ModelTranslator::returnPositionVector(int data_index){
    MatrixXd position_vector(dof, 1);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < state_vector.robots.size(); i++){
        vector<double> jointPositions;
        active_physics_simulator->getRobotJointsPositions(state_vector.robots[i].name, jointPositions, data_index);

        for(int j = 0; j < state_vector.robots[i].jointNames.size(); j++){
            position_vector(j, 0) = jointPositions[j];
        }

        // Increment the current state index by the number of joints in the robot
        currentStateIndex += state_vector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < state_vector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyPose;
        active_physics_simulator->getBodyPose_angle(state_vector.bodiesStates[i].name, bodyPose, data_index);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (state_vector.bodiesStates[i].activeLinearDOF[j]) {
                position_vector(currentStateIndex, 0) = bodyPose.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(state_vector.bodiesStates[i].activeAngularDOF[j]){
                position_vector(currentStateIndex, 0) = bodyPose.orientation[j];
                currentStateIndex++;
            }
        }
    }

    return position_vector;
}

MatrixXd ModelTranslator::returnVelocityVector(int data_index){
    MatrixXd velocity_vector(dof, 1);
//    active_physics_simulator->forwardSimulator(data_index);
    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < state_vector.robots.size(); i++){
        vector<double> jointVelocities;
        active_physics_simulator->getRobotJointsVelocities(state_vector.robots[i].name, jointVelocities, data_index);

        for(int j = 0; j < state_vector.robots[i].jointNames.size(); j++){
            velocity_vector(j, 0) = jointVelocities[j];
        }

        // Increment the current state index by the number of joints in the robot
        currentStateIndex += state_vector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < state_vector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyVelocities;
        active_physics_simulator->getBodyVelocity(state_vector.bodiesStates[i].name, bodyVelocities, data_index);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (state_vector.bodiesStates[i].activeLinearDOF[j]) {
                velocity_vector(currentStateIndex, 0) = bodyVelocities.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(state_vector.bodiesStates[i].activeAngularDOF[j]){
                velocity_vector(currentStateIndex, 0) = bodyVelocities.orientation[j];
                currentStateIndex++;
            }
        }
    }

    return velocity_vector;
}

MatrixXd ModelTranslator::returnAccelerationVector(int data_index){
    MatrixXd accel_vector(dof, 1);
    active_physics_simulator->forwardSimulator(data_index);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < state_vector.robots.size(); i++){
        vector<double> jointAccelerations;
        active_physics_simulator->getRobotJointsAccelerations(state_vector.robots[i].name, jointAccelerations, data_index);

        for(int j = 0; j < state_vector.robots[i].jointNames.size(); j++){
            accel_vector(j, 0) = jointAccelerations[j];
        }

        // Increment the current state index by the number of joints in the robot
        currentStateIndex += state_vector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < state_vector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyAccelerations;
        active_physics_simulator->getBodyAcceleration(state_vector.bodiesStates[i].name, bodyAccelerations, data_index);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (state_vector.bodiesStates[i].activeLinearDOF[j]) {
                accel_vector(currentStateIndex, 0) = bodyAccelerations.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(state_vector.bodiesStates[i].activeAngularDOF[j]){
                accel_vector(currentStateIndex, 0) = bodyAccelerations.orientation[j];
                currentStateIndex++;
            }
        }
    }


    return accel_vector;
}

bool ModelTranslator::setPositionVector(MatrixXd position_vector, int data_index){
    if(position_vector.rows() != (state_vector_size / 2)){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < state_vector.robots.size(); i++){
        vector<double> jointPositions;

        for(int j = 0; j < state_vector.robots[i].jointNames.size(); j++){
            jointPositions.push_back(position_vector(j, 0));
        }

        active_physics_simulator->setRobotJointsPositions(state_vector.robots[i].name, jointPositions, data_index);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += state_vector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < state_vector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyPose;

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (state_vector.bodiesStates[i].activeLinearDOF[j]) {
                bodyPose.position[j] = position_vector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(state_vector.bodiesStates[i].activeAngularDOF[j]){
                bodyPose.orientation[j] = position_vector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }

        active_physics_simulator->setBodyPose_angle(state_vector.bodiesStates[i].name, bodyPose, data_index);
    }

    return true;
}

bool ModelTranslator::setVelocityVector(MatrixXd velocity_vector, int data_index){
    if(velocity_vector.rows() != (state_vector_size / 2)){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < state_vector.robots.size(); i++){
        vector<double> jointVelocities;

        for(int j = 0; j < state_vector.robots[i].jointNames.size(); j++){
            jointVelocities.push_back(velocity_vector(j, 0));
        }
        
        active_physics_simulator->setRobotJointsVelocities(state_vector.robots[i].name, jointVelocities, data_index);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += state_vector.robots[i].jointNames.size();
    }


    // Loop through all bodies in the state vector
    for(int i = 0; i < state_vector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyVelocity;

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (state_vector.bodiesStates[i].activeLinearDOF[j]) {
                bodyVelocity.position[j] = velocity_vector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(state_vector.bodiesStates[i].activeAngularDOF[j]){
                bodyVelocity.orientation[j] = velocity_vector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }

        active_physics_simulator->setBodyVelocity(state_vector.bodiesStates[i].name, bodyVelocity, data_index);
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