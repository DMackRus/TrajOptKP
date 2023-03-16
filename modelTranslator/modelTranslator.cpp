//
// Created by dave on 03/03/23.
//

#include "modelTranslator.h"

modelTranslator::modelTranslator(){

}

void modelTranslator::initModelTranslator(const char* filePath, int _num_ctrl, vector<robot> _robots, vector<bodyStateVec> _bodies){
    //initialise robot
    // Seed random number generator
    srand(time(0));


    // initialise physics simulator
    vector<string> bodyNames;
    for(int i = 0; i < _bodies.size(); i++){
        bodyNames.push_back(_bodies[i].name);
    }
    MuJoCoHelper *myHelper = new MuJoCoHelper(_robots, bodyNames);
    activePhysicsSimulator = myHelper;
    activePhysicsSimulator->initSimulator(0.004, filePath);

    myStateVector.robots = _robots;
    myStateVector.bodiesStates = _bodies;

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

    cout << "state vector size: " << stateVectorSize << endl;
    dof = stateVectorSize / 2;

    // --------- Set size of cost matrices correctly ------------
    num_ctrl = _num_ctrl;
    Q.resize(stateVectorSize, stateVectorSize);
    Q.setZero();
    R.resize(_num_ctrl, _num_ctrl);
    R.setZero();
    X_desired.resize(stateVectorSize, 1);

    // -----------------------------------------------------------------------------------------
    //                      Assign cost matrices
    // ------------------------------------------------------------------------------------------
    // Loop through robots and starting assinging state specific costs
    int Q_index = 0;
    for(int i = 0; i < myStateVector.robots.size(); i++){
        int robotNumJoints = myStateVector.robots[i].jointNames.size();

        // Loop through the robot joints
        for(int j = 0; j < robotNumJoints; j++){
            Q(Q_index + j, Q_index + j) = myStateVector.robots[i].jointPosCosts[j];

            Q(Q_index + j + robotNumJoints, Q_index + j + robotNumJoints) = myStateVector.robots[i].jointVelCosts[j];
        }
        Q_index +=(2*robotNumJoints);
        
    }

    // Loop through robots and starting assinging control specific costs
    int R_index = 0;
    for(int i = 0; i < myStateVector.robots.size(); i++){
        int robotNumJoints = myStateVector.robots[i].jointNames.size();

        // Loop through the robot joints
        for(int j = 0; j < robotNumJoints; j++){
            R(R_index + j, R_index + j) = myStateVector.robots[i].jointControlCosts[j];
        }
        
    }
    // ----------------------------------------------------------------------------------------------

    cout << "Q: " << Q << std::endl;
    cout << "R: " << R << std::endl;

    
}

MatrixXd modelTranslator::returnStateVector(int dataIndex){
    MatrixXd stateVector(stateVectorSize, 1);

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

    cout << "setting state vector" << _stateVector << endl;

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointPositions;
        vector<double> jointVelocities;

        for(int j = 0; j < myStateVector.robots[i].jointNames.size(); j++){
            jointPositions.push_back(_stateVector(j, 0));
            jointVelocities.push_back(_stateVector(j + (stateVectorSize/2), 0));
        }

        activePhysicsSimulator->setRobotJointsPositions(myStateVector.robots[i].name, jointPositions, dataIndex);
        activePhysicsSimulator->setRobotJointsVelocities(myStateVector.robots[i].name, jointVelocities, dataIndex);

        cout << "set pos and velocities for robot " << i << endl;

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += myStateVector.robots[i].jointNames.size();
    }


    // Loop through all bodies in the state vector
    for(int i = 0; i < myStateVector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyPose;
        pose_6 bodyVelocity;

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (myStateVector.bodiesStates[i].activeLinearDOF[j]) {
                bodyPose.position[j] = _stateVector(currentStateIndex, 0);
                bodyVelocity.position[j] = _stateVector(currentStateIndex + (stateVectorSize/2), 0);
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(myStateVector.bodiesStates[i].activeAngularDOF[j]){
                bodyPose.orientation[j] = _stateVector(currentStateIndex, 0);
                bodyVelocity.orientation[j] = _stateVector(currentStateIndex + (stateVectorSize/2), 0);
                currentStateIndex++;
            }
        }

        activePhysicsSimulator->setBodyPose_angle(myStateVector.bodiesStates[i].name, bodyPose, dataIndex);
        activePhysicsSimulator->setBodyVelocity(myStateVector.bodiesStates[i].name, bodyVelocity, dataIndex);
    }

    return true;
}

MatrixXd modelTranslator::returnControlVector(int dataIndex){
    MatrixXd controlVector(num_ctrl, 1);
    int currentStateIndex = 0;

    // loop through all the present robots
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointControls;
        activePhysicsSimulator->getRobotJointsControls(myStateVector.robots[i].name, jointControls, dataIndex);
        for(int j = 0; j < myStateVector.robots[i].numActuators; j++){

            controlVector(currentStateIndex + j, 0) = jointControls[j];
        }

        currentStateIndex += myStateVector.robots[i].numActuators;

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
        for(int j = 0; j < myStateVector.robots[i].numActuators; j++){

            jointControls.push_back(_controlVector(currentStateIndex + j));
        }

        activePhysicsSimulator->setRobotJointsControls(myStateVector.robots[i].name, jointControls, dataIndex);

        currentStateIndex += myStateVector.robots[i].numActuators;

    }

    return true;
}

MatrixXd modelTranslator::returnPositionVector(int dataIndex){
    MatrixXd posVector(dof, 1);

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