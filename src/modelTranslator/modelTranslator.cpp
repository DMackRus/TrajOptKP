//
// Created by dave on 03/03/23.
//

#include "modelTranslator.h"

modelTranslator::modelTranslator(){
}

void modelTranslator::initModelTranslator(std::string yamlFilePath){
    task taskConfig;

    fileHandler yamlReader;
    yamlReader.readModelConfigFile(yamlFilePath, taskConfig);
    modelFilePath = taskConfig.modelFilePath;
    const char* _modelPath = modelFilePath.c_str();

    // initialise physics simulator
    vector<string> bodyNames;
    for(int i = 0; i < taskConfig.robots.size(); i++){
        bodyNames.push_back(taskConfig.robots[i].name);
        cout << "robot names: " << taskConfig.robots[i].name << endl;
        for(int j = 0; j < taskConfig.robots[i].jointNames.size(); j++){
            cout << "joint names: " << taskConfig.robots[i].jointNames[j] << endl;
        }
    }

    for(int i = 0; i < taskConfig.bodiesStates.size(); i++){
        bodyNames.push_back(taskConfig.bodiesStates[i].name);
        cout << "body names: " << taskConfig.bodiesStates[i].name << endl;
    }
    
    mujocoHelper = std::make_shared<MuJoCoHelper>(taskConfig.robots, bodyNames);
    mujocoHelper->initSimulator(taskConfig.modelTimeStep, _modelPath);

    myStateVector.robots = taskConfig.robots;
    myStateVector.bodiesStates = taskConfig.bodiesStates;

    // --------- Set size of state vector correctly ------------
    stateVectorSize = 0;
    for(int i = 0; i < myStateVector.robots.size(); i++){
        stateVectorSize += (2 * myStateVector.robots[i].jointNames.size());
    }

    for(int i = 0; i < myStateVector.bodiesStates.size(); i++){
        for(int j = 0; j < 3; j++){
            if(myStateVector.bodiesStates[i].activeLinearDOF[j]){
                stateVectorSize += 2;
            }
            if(myStateVector.bodiesStates[i].activeAngularDOF[j]){
                stateVectorSize += 2;
            }
        }
    }

    dof = stateVectorSize / 2;
    X_desired.resize(stateVectorSize, 1);
    X_start.resize(stateVectorSize, 1);

    // --------- Set size of cost matrices correctly ------------
    num_ctrl = myStateVector.robots[0].actuatorNames.size();
    Q.resize(stateVectorSize);
    Q.setZero();
    R.resize(num_ctrl);
    R.setZero();

    // -----------------------------------------------------------------------------------------
    //                      Assign cost matrices
    // ------------------------------------------------------------------------------------------
    // Loop through robots and starting assinging state specific costs
    int Q_index = 0;
    for(int i = 0; i < myStateVector.robots.size(); i++){
        int robotNumJoints = myStateVector.robots[i].jointNames.size();

        // Loop through the robot joints
        for(int j = 0; j < robotNumJoints; j++){
            Q.diagonal()[Q_index + j, Q_index + j] = myStateVector.robots[i].jointPosCosts[j];

            Q.diagonal()[Q_index + j + dof, Q_index + j + dof] = myStateVector.robots[i].jointVelCosts[j];

            X_desired(Q_index + j, 0) = myStateVector.robots[i].goalPos[j];
            X_desired(Q_index + j + dof, 0) = 0.0f;

            X_start(Q_index + j, 0) = myStateVector.robots[i].startPos[j];
            X_start(Q_index + j + dof, 0) = 0.0f;
        }
        Q_index += robotNumJoints;
    }

    // Loop through bodies
    for(int i = 0; i < myStateVector.bodiesStates.size(); i++){
        
        int activeDOFs = 0;
        for(int j = 0; j < 3; j++){
            if(myStateVector.bodiesStates[i].activeLinearDOF[j]){
                activeDOFs++;
            }
            if(myStateVector.bodiesStates[i].activeAngularDOF[j]){
                activeDOFs++;
            }
        }

        // Loop through linear states first
        int activeDofCounter = 0;
        for(int j = 0; j < 3; j++){
            if(myStateVector.bodiesStates[i].activeLinearDOF[j]){
                Q.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = myStateVector.bodiesStates[i].linearPosCost[j];

                Q.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = myStateVector.bodiesStates[i].linearVelCost[j];

                X_desired(Q_index + j, 0) = myStateVector.bodiesStates[i].goalLinearPos[j];
                X_desired(Q_index + j + dof, 0) = 0.0f;

                X_start(Q_index + j, 0) = myStateVector.bodiesStates[i].startLinearPos[j];
                X_start(Q_index + j + dof, 0) = 0.0f;

                activeDofCounter++;
            }
        }

        // Loop through angular states second
        for(int j = 0; j < 3; j++){
            if(myStateVector.bodiesStates[i].activeAngularDOF[j]){
                Q.diagonal()[Q_index + activeDofCounter, Q_index + activeDofCounter] = myStateVector.bodiesStates[i].angularPosCost[j];

                Q.diagonal()[Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof] = myStateVector.bodiesStates[i].angularVelCost[j];

                X_desired(Q_index + j, 0) = myStateVector.bodiesStates[i].goalAngularPos[j];
                X_desired(Q_index + j + dof, 0) = 0.0f;

                X_start(Q_index + j, 0) = myStateVector.bodiesStates[i].startAngularPos[j];
                X_start(Q_index + j + dof, 0) = 0.0f;

                activeDofCounter++;
            }
        }
        Q_index += activeDOFs;
    }

    // Loop through robots and starting assinging control specific costs
    int R_index = 0;
    for(int i = 0; i < myStateVector.robots.size(); i++){
        int robotNumActuators = myStateVector.robots[i].actuatorNames.size();

        // Loop through the robot joints
        for(int j = 0; j < robotNumActuators; j++){
            R.diagonal()[R_index + j, R_index + j] = myStateVector.robots[i].jointControlCosts[j];
        }

        R_index += robotNumActuators;
    }
    // ----------------------------------------------------------------------------------------------

//    cout << "Q: " << Q << std::endl;
//    cout << "R: " << R << std::endl;

    // Set terminal cost matrix
    Q_terminal.resize(stateVectorSize);
    Q_terminal.setZero();
    for(int i = 0; i < dof; i++){
        Q_terminal.diagonal()[i] = Q.diagonal()[i] * 1000;
    }

//   cout << "Q_terminal: " << Q_terminal.diagonal() << endl;
}

double modelTranslator::costFunction(std::shared_ptr<mjData> d, bool terminal){
    double cost = 0.0f;
    MatrixXd Xt = returnStateVector(d);
    MatrixXd Ut = returnControlVector(d);

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

void modelTranslator::costDerivatives(std::shared_ptr<mjData> d, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal){
    MatrixXd Xt = returnStateVector(d);
    MatrixXd Ut = returnControlVector(d);

    MatrixXd X_diff = Xt - X_desired;

    // Size cost derivatives appropriately
    l_x.resize(stateVectorSize, 1);
    l_xx.resize(stateVectorSize, stateVectorSize);

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

bool modelTranslator::taskComplete(std::shared_ptr<mjData> d, double &dist){
    return false;
}

std::vector<MatrixXd> modelTranslator::createInitSetupControls(int horizonLength){
    std::vector<MatrixXd> emptyInitSetupControls;
//    mujocoHelper->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
    return emptyInitSetupControls;
}

MatrixXd modelTranslator::returnStateVector(std::shared_ptr<mjData> d){
    MatrixXd stateVector(stateVectorSize, 1);
//    mujocoHelper->forwardSimulator(d);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointPositions;
        vector<double> jointVelocities;
        mujocoHelper->getRobotJointsPositions(myStateVector.robots[i].name, jointPositions, d);
        mujocoHelper->getRobotJointsVelocities(myStateVector.robots[i].name, jointVelocities, d);

        for(int j = 0; j < myStateVector.robots[i].jointNames.size(); j++){
            stateVector(j, 0) = jointPositions[j];
            stateVector(j + (stateVectorSize/2), 0) = jointVelocities[j];
        }

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += myStateVector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < myStateVector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyPose;
        pose_6 bodyVelocity;
        mujocoHelper->getBodyPose_angle(myStateVector.bodiesStates[i].name, bodyPose, d);
        mujocoHelper->getBodyVelocity(myStateVector.bodiesStates[i].name, bodyVelocity, d);
//        cout << "bodyPose: " << bodyPose.position[0] << ", " << bodyPose.position[1] << ", " << bodyPose.position[2] << endl;
//        cout << "bodyVelocity: " << bodyVelocity.position[0] << ", " << bodyVelocity.position[1] << ", " << bodyVelocity.position[2] << endl;
//        cout << "bodyPose: " << bodyPose.orientation[0] << ", " << bodyPose.orientation[1] << ", " << bodyPose.orientation[2] << endl;
//        cout << "bodyVelocity: " << bodyVelocity.orientation[0] << ", " << bodyVelocity.orientation[1] << ", " << bodyVelocity.orientation[2] << endl;

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (myStateVector.bodiesStates[i].activeLinearDOF[j]) {
                stateVector(currentStateIndex, 0) = bodyPose.position[j];
                stateVector(currentStateIndex + (stateVectorSize/2), 0) = bodyVelocity.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(myStateVector.bodiesStates[i].activeAngularDOF[j]){
                stateVector(currentStateIndex, 0) = bodyPose.orientation[j];
                stateVector(currentStateIndex + (stateVectorSize/2), 0) = bodyVelocity.orientation[j];
                currentStateIndex++;
            }
        }
    }

    return stateVector;
}

bool modelTranslator::setStateVector(MatrixXd _stateVector, std::shared_ptr<mjData> d){

    if(_stateVector.rows() != stateVectorSize){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    MatrixXd posVector(stateVectorSize/2, 1);
    MatrixXd velVector(stateVectorSize/2, 1);

    posVector = _stateVector.block(0, 0, stateVectorSize/2, 1);
    velVector = _stateVector.block(stateVectorSize/2, 0, stateVectorSize/2, 1);

    setPositionVector(posVector, d);
    setVelocityVector(velVector, d);

//    int currentStateIndex = 0;
//
//    // Loop through all robots in the state vector
//    for(int i = 0; i < myStateVector.robots.size(); i++){
//        vector<double> jointPositions;
//        vector<double> jointVelocities;
//
//        for(int j = 0; j < myStateVector.robots[i].jointNames.size(); j++){
//            jointPositions.push_back(_stateVector(j, 0));
//            jointVelocities.push_back(_stateVector(j + (stateVectorSize/2), 0));
//        }
//
//        mujocoHelper->setRobotJointsPositions(myStateVector.robots[i].name, jointPositions, d);
//        mujocoHelper->setRobotJointsVelocities(myStateVector.robots[i].name, jointVelocities, d);
//
//        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
//        currentStateIndex += myStateVector.robots[i].jointNames.size();
//    }
//
//    // Loop through all bodies in the state vector
//    for(int i = 0; i < myStateVector.bodiesStates.size(); i++){
//        // Get the body's position and orientation
//        pose_6 bodyPose;
//        pose_6 bodyVelocity;
//
//        mujocoHelper->getBodyPose_angle(myStateVector.bodiesStates[i].name, bodyPose, d);
//        mujocoHelper->getBodyVelocity(myStateVector.bodiesStates[i].name, bodyVelocity, d);
//
//        for(int j = 0; j < 3; j++) {
//            // Linear positions
//            if (myStateVector.bodiesStates[i].activeLinearDOF[j]) {
//                bodyPose.position[j] = _stateVector(currentStateIndex, 0);
//                bodyVelocity.position[j] = _stateVector(currentStateIndex + (stateVectorSize/2), 0);
//                currentStateIndex++;
//            }
//        }
//        for(int j = 0; j < 3; j++) {
//            // angular positions
//            if(myStateVector.bodiesStates[i].activeAngularDOF[j]){
//                bodyPose.orientation[j] = _stateVector(currentStateIndex, 0);
//                bodyVelocity.orientation[j] = _stateVector(currentStateIndex + (stateVectorSize/2), 0);
//                currentStateIndex++;
//            }
//        }
//
//        mujocoHelper->setBodyPose_angle(myStateVector.bodiesStates[i].name, bodyPose, d);
//        mujocoHelper->setBodyVelocity(myStateVector.bodiesStates[i].name, bodyVelocity, d);
//    }

    return true;
}

MatrixXd modelTranslator::returnControlVector(std::shared_ptr<mjData> d){
    MatrixXd controlVector(num_ctrl, 1);
    int currentStateIndex = 0;

    // loop through all the present robots
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointControls;
        mujocoHelper->getRobotJointsControls(myStateVector.robots[i].name, jointControls, d);
        for(int j = 0; j < myStateVector.robots[i].actuatorNames.size(); j++){

            controlVector(currentStateIndex + j, 0) = jointControls[j];
        }

        currentStateIndex += myStateVector.robots[i].actuatorNames.size();

    }

    return controlVector;
}

bool modelTranslator::setControlVector(MatrixXd _controlVector, std::shared_ptr<mjData> d){
    if(_controlVector.rows() != num_ctrl){
        cout << "ERROR: control vector size does not match the size of the control vector in the model translator" << endl;
        return false;
    }

    int currentStateIndex = 0;

    // loop through all the present robots
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointControls;
        for(int j = 0; j < myStateVector.robots[i].actuatorNames.size(); j++){

            jointControls.push_back(_controlVector(currentStateIndex + j));
        }

        mujocoHelper->setRobotJointsControls(myStateVector.robots[i].name, jointControls, d);

        currentStateIndex += myStateVector.robots[i].actuatorNames.size();

    }

    return true;
}

MatrixXd modelTranslator::returnPositionVector(std::shared_ptr<mjData> d){
    MatrixXd posVector(dof, 1);
//    mujocoHelper->forwardSimulator(d);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointPositions;
        mujocoHelper->getRobotJointsPositions(myStateVector.robots[i].name, jointPositions, d);

        for(int j = 0; j < myStateVector.robots[i].jointNames.size(); j++){
            posVector(j, 0) = jointPositions[j];
        }

        // Increment the current state index by the number of joints in the robot
        currentStateIndex += myStateVector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < myStateVector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyPose;
        mujocoHelper->getBodyPose_angle(myStateVector.bodiesStates[i].name, bodyPose, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (myStateVector.bodiesStates[i].activeLinearDOF[j]) {
                posVector(currentStateIndex, 0) = bodyPose.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(myStateVector.bodiesStates[i].activeAngularDOF[j]){
                posVector(currentStateIndex, 0) = bodyPose.orientation[j];
                currentStateIndex++;
            }
        }
    }

    return posVector;
}

MatrixXd modelTranslator::returnVelocityVector(std::shared_ptr<mjData> d){
    MatrixXd velVector(dof, 1);
//    mujocoHelper->forwardSimulator(d);
    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointVelocities;
        mujocoHelper->getRobotJointsVelocities(myStateVector.robots[i].name, jointVelocities, d);

        for(int j = 0; j < myStateVector.robots[i].jointNames.size(); j++){
            velVector(j, 0) = jointVelocities[j];
        }

        // Increment the current state index by the number of joints in the robot
        currentStateIndex += myStateVector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < myStateVector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyVelocities;
        mujocoHelper->getBodyVelocity(myStateVector.bodiesStates[i].name, bodyVelocities, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (myStateVector.bodiesStates[i].activeLinearDOF[j]) {
                velVector(currentStateIndex, 0) = bodyVelocities.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(myStateVector.bodiesStates[i].activeAngularDOF[j]){
                velVector(currentStateIndex, 0) = bodyVelocities.orientation[j];
                currentStateIndex++;
            }
        }
    }


    return velVector;
}

MatrixXd modelTranslator::returnAccelerationVector(std::shared_ptr<mjData> d){
    MatrixXd accelVector(dof, 1);
    mujocoHelper->forwardSimulator(d);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointAccelerations;
        mujocoHelper->getRobotJointsAccelerations(myStateVector.robots[i].name, jointAccelerations, d);

        for(int j = 0; j < myStateVector.robots[i].jointNames.size(); j++){
            accelVector(j, 0) = jointAccelerations[j];
        }

        // Increment the current state index by the number of joints in the robot
        currentStateIndex += myStateVector.robots[i].jointNames.size();
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < myStateVector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyAccelerations;
        mujocoHelper->getBodyAcceleration(myStateVector.bodiesStates[i].name, bodyAccelerations, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (myStateVector.bodiesStates[i].activeLinearDOF[j]) {
                accelVector(currentStateIndex, 0) = bodyAccelerations.position[j];
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(myStateVector.bodiesStates[i].activeAngularDOF[j]){
                accelVector(currentStateIndex, 0) = bodyAccelerations.orientation[j];
                currentStateIndex++;
            }
        }
    }


    return accelVector;
}

bool modelTranslator::setPositionVector(MatrixXd _positionVector, std::shared_ptr<mjData> d){
    if(_positionVector.rows() != (stateVectorSize/2)){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointPositions;

        for(int j = 0; j < myStateVector.robots[i].jointNames.size(); j++){
            jointPositions.push_back(_positionVector(j, 0));
        }

        mujocoHelper->setRobotJointsPositions(myStateVector.robots[i].name, jointPositions, d);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += myStateVector.robots[i].jointNames.size();
    }


    // Loop through all bodies in the state vector
    for(int i = 0; i < myStateVector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyPose;

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (myStateVector.bodiesStates[i].activeLinearDOF[j]) {
                bodyPose.position[j] = _positionVector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(myStateVector.bodiesStates[i].activeAngularDOF[j]){
                bodyPose.orientation[j] = _positionVector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }

        mujocoHelper->setBodyPose_angle(myStateVector.bodiesStates[i].name, bodyPose, d);
    }

    return true;
}

bool modelTranslator::setVelocityVector(MatrixXd _velocityVector, std::shared_ptr<mjData> d){
    if(_velocityVector.rows() != (stateVectorSize/2)){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointVelocities;

        for(int j = 0; j < myStateVector.robots[i].jointNames.size(); j++){
            jointVelocities.push_back(_velocityVector(j, 0));
        }
        
        mujocoHelper->setRobotJointsVelocities(myStateVector.robots[i].name, jointVelocities, d);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += myStateVector.robots[i].jointNames.size();
    }


    // Loop through all bodies in the state vector
    for(int i = 0; i < myStateVector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyVelocity;

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (myStateVector.bodiesStates[i].activeLinearDOF[j]) {
                bodyVelocity.position[j] = _velocityVector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(myStateVector.bodiesStates[i].activeAngularDOF[j]){
                bodyVelocity.orientation[j] = _velocityVector(currentStateIndex, 0);
                currentStateIndex++;
            }
        }

        mujocoHelper->setBodyVelocity(myStateVector.bodiesStates[i].name, bodyVelocity, d);
    }

    return true;
}

std::vector<MatrixXd> modelTranslator::createInitOptimisationControls(int horizonLength) {
    std::vector<MatrixXd> initControls;

    for(int i = 0; i < horizonLength; i++){
        MatrixXd emptyControl(num_ctrl, 1);
        for(int j = 0; j < num_ctrl; j++){
            emptyControl(j) = 0.0f;
        }
        initControls.push_back(emptyControl);
    }

    return initControls;
}