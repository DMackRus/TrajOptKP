//
// Created by dave on 03/03/23.
//

#include "modelTranslator.h"

modelTranslator::modelTranslator(physicsSimulator *_physicsSimulator, stateVectorList _stateVector): myPhysicsSimulator(_physicsSimulator){
    mystateVector = _stateVector;

    stateVectorSize = 0;
    for(int i = 0; i < mystateVector.robots.size(); i++){
        stateVectorSize += (2 * mystateVector.robots[i].jointNames.size());
    }

    for(int i = 0; i < mystateVector.bodiesStates.size(); i++){
        for(int j = 0; j < 3; j++){
            if(mystateVector.bodiesStates[i].activeLinearDOF[j]){
                stateVectorSize += 2;
            }
            if(mystateVector.bodiesStates[i].activeAngularDOF[j]){
                stateVectorSize += 2;
            }
        }
    }

    cout << "state vector size: " << stateVectorSize << endl;

}

MatrixXd modelTranslator::returnStateVector(){
    MatrixXd stateVector(stateVectorSize, 1);

    int currentStateIndex = 0;

    // Loop through all robots in the state vector
    for(int i = 0; i < mystateVector.robots.size(); i++){
        vector<double> jointPositions;
        vector<double> jointVelocities;
        myPhysicsSimulator->getRobotJointsPositions(mystateVector.robots[i].name, jointPositions);
        myPhysicsSimulator->getRobotJointsVelocities(mystateVector.robots[i].name, jointVelocities);
        for(int j = 0; j < mystateVector.robots[i].jointNames.size(); j++){
            stateVector(i, 0) = jointPositions[j];
            stateVector(i + mystateVector.robots[i].jointNames.size(), 0) = jointVelocities[j];
        }

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += (2 * mystateVector.robots[i].jointNames.size());
    }

    // Loop through all bodies in the state vector
    for(int i = 0; i < mystateVector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyPose;
        pose_6 bodyVelocity;
        myPhysicsSimulator->getBodyPose_angle(mystateVector.bodiesStates[i].name, bodyPose);
        myPhysicsSimulator->getBodyVelocity(mystateVector.bodiesStates[i].name, bodyVelocity);

        for(int i = 0; i < 3; i++) {
            if (mystateVector.bodiesStates[i].activeLinearDOF[i]) {
                stateVector(currentStateIndex, 0) = bodyPose.position[i];
                currentStateIndex++;
            }
        }
        for(int i = 0; i < 3; i++) {
            if(mystateVector.bodiesStates[i].activeAngularDOF[i]){
                stateVector(currentStateIndex, 0) = bodyPose.orientation[i];
                currentStateIndex++;
            }
        }
        for(int i = 0; i < 3; i++) {
            if (mystateVector.bodiesStates[i].activeLinearDOF[i]) {
                stateVector(currentStateIndex, 0) = bodyVelocity.position[i];
                currentStateIndex++;
            }
        }
        for(int i = 0; i < 3; i++) {
            if(mystateVector.bodiesStates[i].activeAngularDOF[i]){
                stateVector(currentStateIndex, 0) = bodyVelocity.orientation[i];
                currentStateIndex++;
            }
        }
    }

    std::cout << "stateVector: " << stateVector << std::endl;

    return stateVector;
}