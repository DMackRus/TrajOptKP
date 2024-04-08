#include "Reaching.h"

pandaReaching::pandaReaching(): ModelTranslator(){
    std::string yamlFilePath = "/taskConfigs/reachingConfig.yaml";

    InitModelTranslator(yamlFilePath);
}

bool pandaReaching::TaskComplete(mjData *d, double &dist){
    double cumError = 0.0f;
    MatrixXd X = ReturnStateVector(d);

    for(int i = 0; i < dof; i++){
        double diff = active_state_vector.robots[0].goalPos[i] - X(i);
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
                float randomJoint = randFloat(jointLimsMin[i], jointLimsMax[i]);
                jointStartPositions[i] = randomJoint;
            }

            MuJoCo_helper->setRobotJointsPositions(robotName, jointStartPositions, MuJoCo_helper->main_data);

            // Check if current configuration is valid
            if(MuJoCo_helper->checkSystemForCollisions(MuJoCo_helper->main_data)){
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
                float targetJointVal;
                if(jointStartPositions[i] - jointOffsets[i] > jointLimsMin[i]){
                    targetJointVal = jointStartPositions[i] - jointOffsets[i];
                }
                else{
                    targetJointVal = jointStartPositions[i] + jointOffsets[i];
                }

                float randomJoint = randFloat(targetJointVal - jointOffsetNoise[i], targetJointVal + jointOffsetNoise[i]);
                jointGoalPositions[i] = randomJoint;
            }

            MuJoCo_helper->setRobotJointsPositions(robotName, jointGoalPositions, MuJoCo_helper->main_data);

            // Check if current configuration is valid
            if(MuJoCo_helper->checkSystemForCollisions(MuJoCo_helper->main_data)){
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

    for(int i = 0; i < dof; i++){
        active_state_vector.robots[0].goalPos[i] = jointGoalPositions[i];
    }
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

        MuJoCo_helper->setRobotJointsPositions(robotName, jointPositions, MuJoCo_helper->main_data);

        // Check if current configuration is valid
        if(MuJoCo_helper->checkSystemForCollisions(MuJoCo_helper->main_data)){
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

        MuJoCo_helper->setRobotJointsPositions(robotName, jointPositions, MuJoCo_helper->main_data);

        // Check if current configuration is valid
        if(MuJoCo_helper->checkSystemForCollisions(MuJoCo_helper->main_data)){
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

            MuJoCo_helper->getRobotJointsGravityCompensaionControls(active_state_vector.robots[0].name, gravCompensation, MuJoCo_helper->main_data);

            Xt = ReturnStateVector(MuJoCo_helper->main_data);

            for(int j = 0; j < num_ctrl; j++){
                control(j) = gravCompensation[j];
//                double diff = X_desired(j) - Xt(j);
//                control(j) += diff * gains[j];
            }

            SetControlVector(control, MuJoCo_helper->main_data);
            mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);
            initControls.push_back(control);
        }
    }
    else{

    }

    

    return initControls;
}