#include "reaching.h"

pandaReaching::pandaReaching(): modelTranslator(){
    filePath = "/home/davidrussell/catkin_ws/src/physicsSimSwitching/Franka-emika-panda-arm/V1/reaching_scene.xml";

    vector<robot> robots;
    robot panda;
    panda.name = "panda";
    panda.jointNames = {"panda0_joint1", "panda0_joint2", "panda0_joint3", "panda0_joint4", "panda0_joint5", "panda0_joint6", "panda0_joint7"};
    panda.numActuators = 7;
    panda.jointPosCosts = {10, 10, 10, 10, 10, 10, 10};
    panda.jointVelCosts = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    panda.jointControlCosts = {0, 0, 0, 0, 0, 0, 0};
    robots.push_back(panda);

    vector<bodyStateVec> bodies;

    initModelTranslator(filePath, 7, robots, bodies);
    analyticalCostDerivatives = true;

    X_desired << 0, 0, 0, -1, 0, 0, 0, 
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

    return initControls;
}