#include "Reaching.h"

pandaReaching::pandaReaching(): ModelTranslator(){
    std::string yamlFilePath = "/taskConfigs/reachingConfig.yaml";

    InitModelTranslator(yamlFilePath);

}

bool pandaReaching::TaskComplete(int dataIndex, double &dist){
    double cumError = 0.0f;
    MatrixXd X = ReturnStateVector(dataIndex);

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

void pandaReaching::GenerateRandomGoalAndStartState() {
    bool validStartAndGoal = false;
    vector<double> jointStartPositions;
    vector<double> jointGoalPositions;

    while(!validStartAndGoal){

        // Generate a random starting state
        bool validStartState = false;


        for(int i = 0; i < 7; i++){
            jointStartPositions.push_back(0.0f);
        }

        while(!validStartState){
            // Generate a random robot configuration
            std::string robotName = "panda";
            for(int i = 0; i < 7; i++){
                double randomJoint = randFloat(jointLimsMin[i], jointLimsMax[i]);
                jointStartPositions[i] = randomJoint;
            }

            active_physics_simulator->setRobotJointsPositions(robotName, jointStartPositions, MAIN_DATA_STATE);

            // Check if current configuration is valid
            if(active_physics_simulator->checkSystemForCollisions(MAIN_DATA_STATE)){
                cout << "invalid robot position \n";
            }
            else{
                validStartState = true;
            }
        }

        // Generate a random goal state
        double jointOffsets[7] = {0.5, 0.5, 0.5, 0.5, 0.8, 0.8, 0.8};
        double jointOffsetNoise[7] = {0.1, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2};
        int resetCounter = 0;

        bool validGoalState = false;

        for(int i = 0; i < 7; i++){
            jointGoalPositions.push_back(0.0f);
        }

        while(!validGoalState){
            // Generate a random robot configuration
            std::string robotName = "panda";
            for(int i = 0; i < 7; i++){
                double targetJointVal;
                if(jointStartPositions[i] - jointOffsets[i] > jointLimsMin[i]){
                    targetJointVal = jointStartPositions[i] - jointOffsets[i];
                }
                else{
                    targetJointVal = jointStartPositions[i] + jointOffsets[i];
                }

                double randomJoint = randFloat(targetJointVal - jointOffsetNoise[i], targetJointVal + jointOffsetNoise[i]);
                jointGoalPositions[i] = randomJoint;
            }

            active_physics_simulator->setRobotJointsPositions(robotName, jointGoalPositions, MAIN_DATA_STATE);

            // Check if current configuration is valid
            if(active_physics_simulator->checkSystemForCollisions(MAIN_DATA_STATE)){
                cout << "invalid robot position \n";
                resetCounter++;
            }
            else{
                validGoalState = true;
            }

            if(resetCounter > 100){
                cout << "regenerate random start state \n";
            }

        }

        if(validStartState && validGoalState){
            validStartAndGoal = true;
        }

    }

    X_start << jointStartPositions[0], jointStartPositions[1], jointStartPositions[2], jointStartPositions[3], jointStartPositions[4], jointStartPositions[5], jointStartPositions[6],
            0, 0, 0, 0, 0, 0, 0;
    X_desired << jointGoalPositions[0], jointGoalPositions[1], jointGoalPositions[2], jointGoalPositions[3], jointGoalPositions[4], jointGoalPositions[5], jointGoalPositions[6],
            0, 0, 0, 0, 0, 0, 0;
}

MatrixXd pandaReaching::ReturnRandomStartState(){
    MatrixXd randomStartState(state_vector_size, 1);

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

        active_physics_simulator->setRobotJointsPositions(robotName, jointPositions, MAIN_DATA_STATE);

        // Check if current configuration is valid
        if(active_physics_simulator->checkSystemForCollisions(MAIN_DATA_STATE)){
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

MatrixXd pandaReaching::ReturnRandomGoalState(MatrixXd X0){
    MatrixXd randomGoalState(state_vector_size, 1);

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

        active_physics_simulator->setRobotJointsPositions(robotName, jointPositions, MAIN_DATA_STATE);

        // Check if current configuration is valid
        if(active_physics_simulator->checkSystemForCollisions(MAIN_DATA_STATE)){
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

std::vector<MatrixXd> pandaReaching::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    if(active_state_vector.robots[0].torqueControlled){

        MatrixXd control(num_ctrl, 1);
        double gains[7] = {10, 10, 10, 10, 5, 5, 5};
        MatrixXd Xt;
        vector<double> gravCompensation;
        for(int i = 0; i < horizonLength; i++){

            active_physics_simulator->getRobotJointsGravityCompensaionControls(active_state_vector.robots[0].name, gravCompensation, MAIN_DATA_STATE);

            Xt = ReturnStateVector(MAIN_DATA_STATE);

            for(int j = 0; j < num_ctrl; j++){
                control(j) = gravCompensation[j];
//                double diff = X_desired(j) - Xt(j);
//                control(j) += diff * gains[j];
            }

            SetControlVector(control, MAIN_DATA_STATE);
            active_physics_simulator->stepSimulator(1, MAIN_DATA_STATE);
            initControls.push_back(control);
        }
    }
    else{
        MatrixXd startState = ReturnStateVector(MAIN_DATA_STATE);
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