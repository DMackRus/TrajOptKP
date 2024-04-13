#include "Pentabot.h"

Pentabot::Pentabot(): ModelTranslator(){
    std::string yamlFilePath = "/taskConfigs/pentabot_config.yaml";
    InitModelTranslator(yamlFilePath);
}

// TODO fix
bool Pentabot::TaskComplete(mjData *d, double &dist){
    dist = 0.0;

    MatrixXd Xt = ReturnStateVector(d);

    std::vector<double> robot_joints;
    MuJoCo_helper->GetRobotJointsPositions("pentabot", robot_joints, d);

    for(int i = 0; i < dof; i++){
        std::cout << "robot joint: " << robot_joints[i] << std::endl;
        dist += abs(active_state_vector.robots[0].goalPos[i] - robot_joints[i]);
    }
    std::cout << "dist" << dist << std::endl;

    if(dist < 0.01){
        return true;
    }
    return false;
}

void Pentabot::GenerateRandomGoalAndStartState() {

}

MatrixXd Pentabot::ReturnRandomStartState(){
    MatrixXd randomStartState(state_vector_size, 1);

    float arm1Pos = randFloat(0, 3);
    float arm2Pos = randFloat(0, 3);

    randomStartState << arm1Pos, arm2Pos, 0, 0;

    return randomStartState;
}

MatrixXd Pentabot::ReturnRandomGoalState(MatrixXd X0){
    MatrixXd randomGoalState(state_vector_size, 1);

    float randomNum = randFloat(0, 1);
    // stable down position
    if(randomNum < 0.33){
        randomGoalState << 3.1415, 0, 0, 0;
    }
        // Half up unstable
    else if(randomNum > 0.33 && randomNum < 0.66){
        randomGoalState << 3.1415, 3.1415, 0, 0;
    }
        // Unstable up position
    else{
        randomGoalState << 0, 0, 0, 0;
    }

    return randomGoalState;
}