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
            stateVector(j, 0) = jointPositions[j];
            stateVector(j + (stateVectorSize/2), 0) = jointVelocities[j];
        }

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += mystateVector.robots[i].jointNames.size();
    }

    cout << "currentStateIndex: " << currentStateIndex << endl;

    // Loop through all bodies in the state vector
    for(int i = 0; i < mystateVector.bodiesStates.size(); i++){
        // Get the body's position and orientation
        pose_6 bodyPose;
        pose_6 bodyVelocity;
        myPhysicsSimulator->getBodyPose_angle(mystateVector.bodiesStates[i].name, bodyPose);
        myPhysicsSimulator->getBodyVelocity(mystateVector.bodiesStates[i].name, bodyVelocity);
//        cout << "bodyPose: " << bodyPose.position[0] << ", " << bodyPose.position[1] << ", " << bodyPose.position[2] << endl;
//        cout << "bodyVelocity: " << bodyVelocity.position[0] << ", " << bodyVelocity.position[1] << ", " << bodyVelocity.position[2] << endl;
//        cout << "bodyPose: " << bodyPose.orientation[0] << ", " << bodyPose.orientation[1] << ", " << bodyPose.orientation[2] << endl;
//        cout << "bodyVelocity: " << bodyVelocity.orientation[0] << ", " << bodyVelocity.orientation[1] << ", " << bodyVelocity.orientation[2] << endl;

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (mystateVector.bodiesStates[i].activeLinearDOF[j]) {
                stateVector(currentStateIndex, 0) = bodyPose.position[j];
                stateVector(currentStateIndex + (stateVectorSize/2), 0) = bodyVelocity.position[j];
                cout << "bodyPose.position[j]: " << bodyPose.position[j] << endl;
                cout << "bodyVelocity.position[j]: " << bodyVelocity.position[j] << endl;
                currentStateIndex++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(mystateVector.bodiesStates[i].activeAngularDOF[j]){
                stateVector(currentStateIndex, 0) = bodyPose.orientation[j];
                stateVector(currentStateIndex + (stateVectorSize/2), 0) = bodyVelocity.orientation[j];
                currentStateIndex++;
            }
        }
    }

    return stateVector;
}