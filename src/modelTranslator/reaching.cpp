#include "reaching.h"

pandaReaching::pandaReaching(): modelTranslator(){
    std::string yamlFilePath = "/taskConfigs/reachingConfig.yaml";

    initModelTranslator(yamlFilePath);

}

bool pandaReaching::taskComplete(int dataIndex){
    double cumError = 0.0f;
    MatrixXd X = returnStateVector(dataIndex);

    for(int i = 0; i < dof; i++){
        double diff = X_desired(i) - X(i);
        cumError += diff;
    }

    if(cumError < 0.01){
        return true;
    }
    else{
        return false;
    }
}

MatrixXd pandaReaching::returnRandomStartState(){
    MatrixXd randomStartState(stateVectorSize, 1);

    randomStartState << -1, 0.5, 0, -1, 0, 0.6, 1,
                        0, 0, 0, 0, 0, 0, 0;

    return randomStartState;
}

MatrixXd pandaReaching::returnRandomGoalState(MatrixXd X0){
    MatrixXd randomGoalState(stateVectorSize, 1);

    float randomNum = randFloat(0, 1);
    // stable down position
    if(randomNum > 0.5){
        randomGoalState << 0, 0, 0, -1, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0;
    }
    // Unstable up position
    else{
        randomGoalState << 0, 0, 0, -1, 0, 0, 0, 
                            0, 0, 0, 0, 0, 0, 0;
    }

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

            for(int i = 0; i < num_ctrl; i++){
                control(i) = gravCompensation[i];
                double diff = X_desired(i) - Xt(i);
                control(i) += diff * gains[i];
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

        cout << "start state: " << startState << endl;
        cout << "goal state: " << goalState << endl;

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

        cout << "final control: " << initControls[initControls.size() - 1];
    }

    

    return initControls;
}