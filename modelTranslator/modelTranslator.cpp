//
// Created by dave on 03/03/23.
//

#include "modelTranslator.h"

modelTranslator::modelTranslator(){

}

void modelTranslator::initModelTranslator(const char* filePath, int _num_ctrl, vector<robot> _robots, vector<string> _bodies){
    //initialise robot

    MuJoCoHelper *myHelper = new MuJoCoHelper(_robots, _bodies);
    activePhysicsSimulator = myHelper;

    activePhysicsSimulator->initSimulator(0.004, filePath);

    myStateVector.robots = _robots;
    // bodyStateVec goalState;
    // goalState.name = "goal";
    // for(int i = 0; i < 3; i++){
    //     goalState.activeLinearDOF[i] = true;
    //     goalState.activeAngularDOF[i] = true;
    // }
    // myStateVector.bodiesStates.push_back(goalState);

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

    num_ctrl = _num_ctrl;
    Q.resize(stateVectorSize, stateVectorSize);
    R.resize(_num_ctrl, _num_ctrl);
    X_desired.resize(stateVectorSize, 1);

    cout << "Q rows: " << Q.rows() << " cols: " << Q.cols() << std::endl;
    cout << "R rows: " << R.rows() << " cols: " << R.cols() << std::endl;

    cout << "state vector size: " << stateVectorSize << endl;
}

MatrixXd modelTranslator::returnStateVector(){
    MatrixXd stateVector(stateVectorSize, 1);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointPositions;
        vector<double> jointVelocities;
        activePhysicsSimulator->getRobotJointsPositions(myStateVector.robots[i].name, jointPositions);
        activePhysicsSimulator->getRobotJointsVelocities(myStateVector.robots[i].name, jointVelocities);

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
        activePhysicsSimulator->getBodyPose_angle(myStateVector.bodiesStates[i].name, bodyPose);
        activePhysicsSimulator->getBodyVelocity(myStateVector.bodiesStates[i].name, bodyVelocity);
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

bool modelTranslator::setStateVector(MatrixXd _stateVector){

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

        activePhysicsSimulator->setRobotJointsPositions(myStateVector.robots[i].name, jointPositions);
        activePhysicsSimulator->setRobotJointsVelocities(myStateVector.robots[i].name, jointVelocities);

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

        activePhysicsSimulator->setBodyPose_angle(myStateVector.bodiesStates[i].name, bodyPose);
        activePhysicsSimulator->setBodyVelocity(myStateVector.bodiesStates[i].name, bodyVelocity);
    }

    return true;
}

MatrixXd modelTranslator::returnControlVector(){
    MatrixXd controlVector(num_ctrl, 1);
    int currentStateIndex = 0;

    // loop through all the present robots
    for(int i = 0; i < myStateVector.robots.size(); i++){
        vector<double> jointControls;
        activePhysicsSimulator->getRobotJointsControls(myStateVector.robots[i].name, jointControls);
        for(int j = 0; j < myStateVector.robots[i].numActuators; j++){

            controlVector(currentStateIndex + j, 0) = jointControls[j];
        }

        currentStateIndex += myStateVector.robots[i].numActuators;

    }

    return controlVector;
}

bool modelTranslator::setControlVector(MatrixXd _controlVector){
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

        activePhysicsSimulator->setRobotJointsControls(myStateVector.robots[i].name, jointControls);

        currentStateIndex += myStateVector.robots[i].numActuators;

    }

    return true;
}
