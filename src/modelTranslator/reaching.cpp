#include "reaching.h"

pandaReaching::pandaReaching(): modelTranslator(){
    std::string yamlFilePath = "/home/davidrussell/catkin_ws/src/autoTOTask/taskConfigs/reachingConfig.yaml";

    initModelTranslator(yamlFilePath);
    analyticalCostDerivatives = true;

    X_desired << 1, 1.5, 2, -2, 0, 0.6, 1, 
                 0, 0, 0, 0, 0, 0, 0;
    std::cout << "initialised reaching model translator" << std::endl;
}

MatrixXd pandaReaching::returnRandomStartState(){
    MatrixXd randomStartState(stateVectorSize, 1);

    randomStartState << -1, 0.5, 0, -1, 0, 0.6, 1,
                        0, 0, 0, 0, 0, 0, 0;

    return randomStartState;
}

MatrixXd pandaReaching::returnRandomGoalState(){
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

std::vector<MatrixXd> pandaReaching::createInitControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    if(myStateVector.robots[0].torqueControlled){

        MatrixXd control(num_ctrl, 1);
        vector<double> gravCompensation;
        for(int i = 0; i < horizonLength; i++){

            activePhysicsSimulator->getRobotJointsGravityCompensaionControls(myStateVector.robots[0].name, gravCompensation, MAIN_DATA_STATE);
            for(int i = 0; i < num_ctrl; i++){
                control(i) = gravCompensation[i];
            }
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