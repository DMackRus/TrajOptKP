//
// Created by dave on 03/03/23.
//

#include "modelTranslator.h"

modelTranslator::modelTranslator(){

}
//
//void modelTranslator::loadRobotsandBodiesFromYAML(std::string yamlFilePath, vector<robot> &_robots, vector<bodyStateVec> &_bodies){
//    std::string mainPath = __FILE__;
//    mainPath = mainPath.substr(0, mainPath.find_last_of("/\\"));
//    mainPath = mainPath.substr(0, mainPath.find_last_of("/\\"));
//    mainPath = mainPath.substr(0, mainPath.find_last_of("/\\"));
//    YAML::Node node = YAML::LoadFile(mainPath + yamlFilePath);
//
//    int counter = 0;
//
//    modelFilePath = mainPath + node["modelFile"].as<std::string>();
//
//    for(YAML::const_iterator it=node.begin(); it!=node.end(); ++it) {
//        // Robots
//        if(counter == 1){
//            // Loop through robots list
//            robot tempRobot;
//            string robotName;
//            vector<string> jointNames;
//            int numActuators;
//            bool torqueControlled;
//            vector<double> torqueLimits;
//            vector<double> jointPosCosts;
//            vector<double> jointVelCosts;
//            vector<double> jointControlCosts;
//
//            for(YAML::const_iterator robot_it=it->second.begin(); robot_it!=it->second.end(); ++robot_it){
//
//                robotName = robot_it->first.as<string>();
//
//                for(int i = 0; i < robot_it->second["jointNames"].size(); i++){
//                    jointNames.push_back(robot_it->second["jointNames"][i].as<std::string>());
//                }
//
//                numActuators = robot_it->second["numActuators"].as<int>();
//                torqueControlled = robot_it->second["torqueControl"].as<bool>();
//
//                for(int i = 0; i < robot_it->second["torqueLimits"].size(); i++){
//                    torqueLimits.push_back(robot_it->second["torqueLimits"][i].as<double>());
//                }
//
//                for(int i = 0; i < robot_it->second["jointPosCosts"].size(); i++){
//                    jointPosCosts.push_back(robot_it->second["jointPosCosts"][i].as<double>());
//                }
//
//                for(int i = 0; i < robot_it->second["jointVelCosts"].size(); i++){
//                    jointVelCosts.push_back(robot_it->second["jointVelCosts"][i].as<double>());
//                }
//
//                for(int i = 0; i < robot_it->second["jointControlCosts"].size(); i++){
//                    jointControlCosts.push_back(robot_it->second["jointControlCosts"][i].as<double>());
//                }
//            }
//
//            tempRobot.name = robotName;
//            tempRobot.jointNames = jointNames;
//            tempRobot.numActuators = numActuators;
//            tempRobot.torqueControlled = torqueControlled;
//            tempRobot.torqueLimits = torqueLimits;
//            tempRobot.jointPosCosts = jointPosCosts;
//            tempRobot.jointVelCosts = jointVelCosts;
//            tempRobot.jointControlCosts = jointControlCosts;
//
//             _robots.push_back(tempRobot);
//        }
//        // Loop through bodies
//        else if(counter == 2){
//            bodyStateVec tempBody;
//            std::string bodyName;
//            bool activeLinearDOF[3];
//            bool activeAngularDOF[3];
//            double linearPosCosts[3];
//            double linearVelCosts[3];
//            double angularPosCosts[3];
//            double angularVelCosts[3];
//            // Loop through bodies list
//            for(YAML::const_iterator robot_it=it->second.begin(); robot_it!=it->second.end(); ++robot_it){
//
//                bodyName = robot_it->first.as<string>();
//
//                for(int i = 0; i < robot_it->second["activeLinearDOF"].size(); i++){
//                    activeLinearDOF[i] = robot_it->second["activeLinearDOF"][i].as<bool>();
//                }
//
//                for(int i = 0; i < robot_it->second["activeAngularDOF"].size(); i++){
//                    activeAngularDOF[i] = robot_it->second["activeAngularDOF"][i].as<bool>();
//                }
//
//                for(int i = 0; i < robot_it->second["linearPosCost"].size(); i++){
//                    linearPosCosts[i] = robot_it->second["linearPosCost"][i].as<double>();
//                }
//
//                for(int i = 0; i < robot_it->second["linearVelCost"].size(); i++){
//                    linearVelCosts[i] = robot_it->second["linearVelCost"][i].as<double>();
//                }
//
//                for(int i = 0; i < robot_it->second["angularPosCost"].size(); i++){
//                    angularPosCosts[i] = robot_it->second["angularPosCost"][i].as<double>();
//                }
//
//                for(int i = 0; i < robot_it->second["angularVelCost"].size(); i++){
//                    angularVelCosts[i] = robot_it->second["angularVelCost"][i].as<double>();
//                }
//
//                tempBody.name = bodyName;
//                for(int i = 0; i < 3; i++){
//                    tempBody.activeLinearDOF[i] = activeLinearDOF[i];
//                    tempBody.activeAngularDOF[i] = activeAngularDOF[i];
//                    tempBody.linearPosCost[i] = linearPosCosts[i];
//                    tempBody.linearVelCost[i] = linearVelCosts[i];
//                    tempBody.angularPosCost[i] = angularPosCosts[i];
//                    tempBody.angularVelCost[i] = angularVelCosts[i];
//                }
//
//                _bodies.push_back(tempBody);
//            }
//        }
//        counter ++;
//    }
//}

void modelTranslator::initModelTranslator(std::string yamlFilePath){
    //initialise robot

    vector<robot> robots;
    vector<bodyStateVec> bodies;

    fileHandler yamlReader;
    yamlReader.readModelConfigFile(yamlFilePath, robots, bodies, modelFilePath);
    //loadRobotsandBodiesFromYAML(yamlFilePath, robots, bodies);
    const char* _modelPath = modelFilePath.c_str();

    // initialise physics simulator
    vector<string> bodyNames;
    for(int i = 0; i < bodies.size(); i++){
        bodyNames.push_back(bodies[i].name);
        cout << "body names: " << bodies[i].name << endl;
    }
    
    myHelper = new MuJoCoHelper(robots, bodyNames);
    activePhysicsSimulator = myHelper;
    activePhysicsSimulator->initSimulator(0.004, _modelPath);

    myStateVector.robots = robots;
    myStateVector.bodiesStates = bodies;

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
    num_ctrl = myStateVector.robots[0].numActuators;
    Q.resize(stateVectorSize, stateVectorSize);
    Q.setZero();
    R.resize(num_ctrl, num_ctrl);
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
        cout << "num of active dofs: " << activeDOFs << endl;

        // Loop through linear states first
        int activeDofCounter = 0;
        for(int j = 0; j < 3; j++){
            if(myStateVector.bodiesStates[i].activeLinearDOF[j]){
                Q(Q_index + activeDofCounter, Q_index + activeDofCounter) = myStateVector.bodiesStates[i].linearPosCost[j];

                Q(Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof) = myStateVector.bodiesStates[i].linearVelCost[j];

                activeDofCounter++;
            }
        }

        // Loop through angular states second
        for(int j = 0; j < 3; j++){
            if(myStateVector.bodiesStates[i].activeAngularDOF[j]){
                Q(Q_index + activeDofCounter, Q_index + activeDofCounter) = myStateVector.bodiesStates[i].angularPosCost[j];

                Q(Q_index + activeDofCounter + dof, Q_index + activeDofCounter + dof) = myStateVector.bodiesStates[i].angularVelCost[j];

                activeDofCounter++;
            }
        }
        Q_index += activeDOFs;
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

    Q_terminal = Q.replicate(1, 1);
    for(int i = 0; i < dof; i++){
        Q_terminal(i, i) *= 10000;
    }

    cout << "Q_terminal: " << Q_terminal << endl;
}

double modelTranslator::costFunction(MatrixXd Xt, MatrixXd Ut, MatrixXd X_last, MatrixXd U_last, bool terminal){
    double cost = 0.0f;

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

void modelTranslator::costDerivatives(MatrixXd Xt, MatrixXd Ut, MatrixXd X_last, MatrixXd U_last, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal){
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

bool modelTranslator::taskComplete(int dataIndex){
    return false;
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

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        currentStateIndex += myStateVector.robots[i].jointNames.size();
    }
    cout << "current State Index << " << currentStateIndex << endl;

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
        }bool setPositionVector(MatrixXd _positionVector, int dataIndex);
    bool setVelocityVector(MatrixXd _velocityVector, int dataIndex);
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

std::vector<MatrixXd> modelTranslator::createInitControls(int horizonLength){
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