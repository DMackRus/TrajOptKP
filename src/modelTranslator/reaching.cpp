#include "reaching.h"

pandaReaching::pandaReaching(): modelTranslator(){
    std::string yamlFilePath = "/taskConfigs/reachingConfig.yaml";

    initModelTranslator(yamlFilePath);

}

bool pandaReaching::taskComplete(int dataIndex, double &dist){
    double cumError = 0.0f;
    MatrixXd X = returnStateVector(dataIndex);

    for(int i = 0; i < dof; i++){
        double diff = X_desired(i) - X(i);
        cumError += diff;
    }

    dist = cumError;

    if(cumError < 0.05){
        return true;
    }
    else{
        return false;
    }
}

MatrixXd pandaReaching::returnRandomStartState(){
    MatrixXd randomStartState(stateVectorSize, 1);

    bool validStartState = false;
    vector<double> jointPositions;

    for(int i = 0; i < 7; i++){
        jointPositions.push_back(0.0f);
    }

    while(!validStartState){
        // Generate a random robot configuration
        std::string robotName = "panda";
        for(int i = 0; i < 7; i++){
            double randomJoint = randFloat(jointLimsMin[i], jointLimsMax[i]);
            jointPositions[i] = randomJoint;
        }

        activePhysicsSimulator->setRobotJointsPositions(robotName, jointPositions, MAIN_DATA_STATE);

        // Check if current configuration is valid
        if(activePhysicsSimulator->checkSystemForCollisions(MAIN_DATA_STATE)){
            cout << "invalid robot position \n";
        }
        else{
            validStartState = true;
        }
    }

    randomStartState << jointPositions[0], jointPositions[1], jointPositions[2], jointPositions[3], jointPositions[4], jointPositions[5], jointPositions[6],
                        0, 0, 0, 0, 0, 0, 0;

    return randomStartState;
}

MatrixXd pandaReaching::returnRandomGoalState(MatrixXd X0){
    MatrixXd randomGoalState(stateVectorSize, 1);

    double jointOffsets[7] = {0.5, 0.5, 0.5, 0.5, 0.8, 0.8, 0.8};
    double jointOffsetNoise[7] = {0.1, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2};

    bool validStartState = false;
    vector<double> jointPositions;

    for(int i = 0; i < 7; i++){
        jointPositions.push_back(0.0f);
    }

    while(!validStartState){
        // Generate a random robot configuration
        std::string robotName = "panda";
        for(int i = 0; i < 7; i++){
            double targetJointVal;
            if(X0(i) - jointOffsets[i] > jointLimsMin[i]){
                targetJointVal = X0(i) - jointOffsets[i];
            }
            else{
                targetJointVal = X0(i) + jointOffsets[i];
            }

            double randomJoint = randFloat(targetJointVal - jointOffsetNoise[i], targetJointVal + jointOffsetNoise[i]);
            jointPositions[i] = randomJoint;
        }

        activePhysicsSimulator->setRobotJointsPositions(robotName, jointPositions, MAIN_DATA_STATE);

        // Check if current configuration is valid
        if(activePhysicsSimulator->checkSystemForCollisions(MAIN_DATA_STATE)){
            cout << "invalid robot position \n";
        }
        else{
            validStartState = true;
        }

    }

    randomGoalState << jointPositions[0], jointPositions[1], jointPositions[2], jointPositions[3], jointPositions[4], jointPositions[5], jointPositions[6],
            0, 0, 0, 0, 0, 0, 0;

    return randomGoalState;
}

std::vector<MatrixXd> pandaReaching::createInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    if(myStateVector.robots[0].torqueControlled){

        MatrixXd control(num_ctrl, 1);
        double gains[7] = {10, 10, 10, 10, 5, 5, 5};
        MatrixXd Xt;
        vector<double> gravCompensation;
        for(int i = 0; i < horizonLength; i++){

            activePhysicsSimulator->getRobotJointsGravityCompensaionControls(myStateVector.robots[0].name, gravCompensation, MAIN_DATA_STATE);

            Xt = returnStateVector(MAIN_DATA_STATE);

            for(int j = 0; j < num_ctrl; j++){
                control(j) = gravCompensation[j];
//                double diff = X_desired(j) - Xt(j);
//                control(j) += diff * gains[j];
            }

            setControlVector(control, MAIN_DATA_STATE);
            activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
            initControls.push_back(control);
        }
    }
    else{
        MatrixXd startState = returnStateVector(MAIN_DATA_STATE);
        MatrixXd goalState = X_desired.replicate(1, 1);
        MatrixXd difference = goalState - startState;


        MatrixXd controlDiff(num_ctrl, 1);

        MatrixXd control(num_ctrl, 1);
        for(int i = 0; i < num_ctrl; i++){
            control(i) = startState(i);
            controlDiff(i) = difference(i);
        }

        for(int i = 0; i < horizonLength; i++){
            initControls.push_back(control);

            control += (controlDiff / horizonLength);
            //cout << "control: " << control << endl;
        }

    }

    

    return initControls;
}