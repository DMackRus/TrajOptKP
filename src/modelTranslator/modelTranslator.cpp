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
    modelName = taskConfig.modelName;
    minN = taskConfig.minN;
    maxN = taskConfig.maxN;
    keypointMethod = taskConfig.keypointMethod;
    iterativeErrorThreshold = taskConfig.iterativeErrorThreshold;
    const char* _modelPath = modelFilePath.c_str();

    // initialise physics simulator
    vector<string> bodyNames;
    for(int i = 0; i < taskConfig.robots.size(); i++){
        bodyNames.push_back(taskConfig.robots[i].name);
//        cout << "robot names: " << taskConfig.robots[i].name << endl;
        for(int j = 0; j < taskConfig.robots[i].jointNames.size(); j++){
//            cout << "joint names: " << taskConfig.robots[i].jointNames[j] << endl;
            jerkThresholds.push_back(taskConfig.robots[i].jointJerkThresholds[j]);
            // TODO fix this dupliate jerk thresholds
            accelThresholds.push_back(taskConfig.robots[i].jointJerkThresholds[j]);
            magVelChangeThresholds.push_back(taskConfig.robots[i].magVelThresholds[j]);
        }

    }

    for(int i = 0; i < taskConfig.bodiesStates.size(); i++){
        bodyNames.push_back(taskConfig.bodiesStates[i].name);
//        cout << "body names: " << taskConfig.bodiesStates[i].name << endl;
        for(int j = 0; j < 3; j++){
            jerkThresholds.push_back(taskConfig.bodiesStates[i].linearJerkThreshold[j]);
            jerkThresholds.push_back(taskConfig.bodiesStates[i].angularJerkThreshold[j]);

            // TODO fix this dupliate jerk thresholds
            accelThresholds.push_back(taskConfig.bodiesStates[i].linearJerkThreshold[j]);
            accelThresholds.push_back(taskConfig.bodiesStates[i].angularJerkThreshold[j]);

            magVelChangeThresholds.push_back(taskConfig.bodiesStates[i].linearMagVelThreshold[j]);
            magVelChangeThresholds.push_back(taskConfig.bodiesStates[i].angularMagVelThreshold[j]);
        }
    }
    
    myHelper = std::make_shared<MuJoCoHelper>(taskConfig.robots, bodyNames);
    activePhysicsSimulator = myHelper;
    activePhysicsSimulator->initSimulator(taskConfig.modelTimeStep, _modelPath);

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
    Q_terminal.resize(stateVectorSize);
    Q_terminal.setZero();
    R.resize(num_ctrl);
    R.setZero();

    // -----------------------------------------------------------------------------------------
    //                      Assign cost matrices
    // ------------------------------------------------------------------------------------------
    // Loop through robots and starting assigning state specific costs
    cout << "befroe assigning Q matrices" << endl;
    int Q_index = 0;
    for(int i = 0; i < myStateVector.robots.size(); i++){
        int robotNumJoints = myStateVector.robots[i].jointNames.size();

        // Loop through the robot joints
        for(int j = 0; j < robotNumJoints; j++){
            Q.diagonal()[Q_index + j, Q_index + j] = myStateVector.robots[i].jointPosCosts[j];
            Q_terminal.diagonal()[Q_index + j, Q_index + j] = myStateVector.robots[i].terminalJointPosCosts[j];
            cout << "terminal joint pos costs: " << myStateVector.robots[i].terminalJointPosCosts[j] << endl;

            Q.diagonal()[Q_index + j + dof, Q_index + j + dof] = myStateVector.robots[i].jointVelCosts[j];
            Q_terminal.diagonal()[Q_index + j + dof, Q_index + j + dof] = myStateVector.robots[i].terminalJointVelCosts[j];
            cout << "terminal joint vel costs: " << myStateVector.robots[i].terminalJointVelCosts[j] << endl;

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

    cout << "Q: " << Q.diagonal() << std::endl;
    cout << "R: " << R.diagonal() << std::endl;

    // Set terminal cost matrix

//    for(int i = 0; i < dof; i++){
//        Q_terminal.diagonal()[i] = Q.diagonal()[i] * 10000;
//        // 1000 is a magic number that seems to work well
//    }

   cout << "Q_terminal: " << Q_terminal.diagonal() << endl;
}

double modelTranslator::costFunction(int dataIndex, bool terminal){
    double cost;
    MatrixXd Xt = returnStateVector(dataIndex);
    MatrixXd Ut = returnControlVector(dataIndex);

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

void modelTranslator::costDerivatives(int dataIndex, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal){
    MatrixXd Xt = returnStateVector(dataIndex);
    MatrixXd Ut = returnControlVector(dataIndex);

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

    // Do I need to time the cost derivatives by the time step?
//    l_x = l_x * activePhysicsSimulator->returnModelTimeStep();
//    l_xx = l_xx * activePhysicsSimulator->returnModelTimeStep();
//
//    l_u = l_u * activePhysicsSimulator->returnModelTimeStep();
//    l_uu = l_uu * activePhysicsSimulator->returnModelTimeStep();
}

bool modelTranslator::taskComplete(int dataIndex, double &dist){
    return false;
}

std::vector<MatrixXd> modelTranslator::createInitSetupControls(int horizonLength){
    std::vector<MatrixXd> emptyInitSetupControls;
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
    return emptyInitSetupControls;
}

MatrixXd modelTranslator::returnStateVector(int dataIndex){
    MatrixXd stateVector(stateVectorSize, 1);
//    activePhysicsSimulator->forwardSimulator(dataIndex);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointPositions;
        vector<double> jointVelocities;
        activePhysicsSimulator->getRobotJointsPositions(myStateVector.robots[i].name, jointPositions, dataIndex);
        activePhysicsSimulator->getRobotJointsVelocities(myStateVector.robots[i].name, jointVelocities, dataIndex);

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
        activePhysicsSimulator->getBodyPose_angle(myStateVector.bodiesStates[i].name, bodyPose, dataIndex);
        activePhysicsSimulator->getBodyVelocity(myStateVector.bodiesStates[i].name, bodyVelocity, dataIndex);
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

bool modelTranslator::setStateVector(MatrixXd _stateVector, int dataIndex){

    if(_stateVector.rows() != stateVectorSize){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    MatrixXd posVector(stateVectorSize/2, 1);
    MatrixXd velVector(stateVectorSize/2, 1);

    posVector = _stateVector.block(0, 0, stateVectorSize/2, 1);
    velVector = _stateVector.block(stateVectorSize/2, 0, stateVectorSize/2, 1);

    setPositionVector(posVector, dataIndex);
    setVelocityVector(velVector, dataIndex);

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
//        activePhysicsSimulator->setRobotJointsPositions(myStateVector.robots[i].name, jointPositions, dataIndex);
//        activePhysicsSimulator->setRobotJointsVelocities(myStateVector.robots[i].name, jointVelocities, dataIndex);
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
//        activePhysicsSimulator->getBodyPose_angle(myStateVector.bodiesStates[i].name, bodyPose, dataIndex);
//        activePhysicsSimulator->getBodyVelocity(myStateVector.bodiesStates[i].name, bodyVelocity, dataIndex);
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
//        activePhysicsSimulator->setBodyPose_angle(myStateVector.bodiesStates[i].name, bodyPose, dataIndex);
//        activePhysicsSimulator->setBodyVelocity(myStateVector.bodiesStates[i].name, bodyVelocity, dataIndex);
//    }

    return true;
}

MatrixXd modelTranslator::returnControlVector(int dataIndex){
    MatrixXd controlVector(num_ctrl, 1);
    int currentStateIndex = 0;

    // loop through all the present robots
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointControls;
        activePhysicsSimulator->getRobotJointsControls(myStateVector.robots[i].name, jointControls, dataIndex);
        for(int j = 0; j < myStateVector.robots[i].actuatorNames.size(); j++){

            controlVector(currentStateIndex + j, 0) = jointControls[j];
        }

        currentStateIndex += myStateVector.robots[i].actuatorNames.size();

    }

    return controlVector;
}

bool modelTranslator::setControlVector(MatrixXd _controlVector, int dataIndex){
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

        activePhysicsSimulator->setRobotJointsControls(myStateVector.robots[i].name, jointControls, dataIndex);

        currentStateIndex += myStateVector.robots[i].actuatorNames.size();

    }

    return true;
}

MatrixXd modelTranslator::returnPositionVector(int dataIndex){
    MatrixXd posVector(dof, 1);
//    activePhysicsSimulator->forwardSimulator(dataIndex);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointPositions;
        activePhysicsSimulator->getRobotJointsPositions(myStateVector.robots[i].name, jointPositions, dataIndex);

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
        activePhysicsSimulator->getBodyPose_angle(myStateVector.bodiesStates[i].name, bodyPose, dataIndex);

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

MatrixXd modelTranslator::returnVelocityVector(int dataIndex){
    MatrixXd velVector(dof, 1);
//    activePhysicsSimulator->forwardSimulator(dataIndex);
    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointVelocities;
        activePhysicsSimulator->getRobotJointsVelocities(myStateVector.robots[i].name, jointVelocities, dataIndex);

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
        activePhysicsSimulator->getBodyVelocity(myStateVector.bodiesStates[i].name, bodyVelocities, dataIndex);

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

MatrixXd modelTranslator::returnAccelerationVector(int dataIndex){
    MatrixXd accelVector(dof, 1);
    activePhysicsSimulator->forwardSimulator(dataIndex);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointAccelerations;
        activePhysicsSimulator->getRobotJointsAccelerations(myStateVector.robots[i].name, jointAccelerations, dataIndex);

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
        activePhysicsSimulator->getBodyAcceleration(myStateVector.bodiesStates[i].name, bodyAccelerations, dataIndex);

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

bool modelTranslator::setPositionVector(MatrixXd _positionVector, int dataIndex){
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

        activePhysicsSimulator->setRobotJointsPositions(myStateVector.robots[i].name, jointPositions, dataIndex);

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

        activePhysicsSimulator->setBodyPose_angle(myStateVector.bodiesStates[i].name, bodyPose, dataIndex);
    }

    return true;
}

bool modelTranslator::setVelocityVector(MatrixXd _velocityVector, int dataIndex){
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
        
        activePhysicsSimulator->setRobotJointsVelocities(myStateVector.robots[i].name, jointVelocities, dataIndex);

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

        activePhysicsSimulator->setBodyVelocity(myStateVector.bodiesStates[i].name, bodyVelocity, dataIndex);
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