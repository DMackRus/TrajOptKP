#include "Pentabot.h"

Pentabot::Pentabot(): ModelTranslator(){
    std::string yamlFilePath = "/taskConfigs/pentabot_config.yaml";
    InitModelTranslator(yamlFilePath);
}

void Pentabot::ReturnRandomStartState(){
    active_state_vector.robots[0].startPos[0] = 3.1415;
    active_state_vector.robots[0].startPos[1] = randFloat(-0.1, 0.1);
    active_state_vector.robots[0].startPos[2] = randFloat(-0.2, 0.2);
    active_state_vector.robots[0].startPos[3] = randFloat(-0.1, 0.1);
    active_state_vector.robots[0].startPos[4] = randFloat(-0.4, 0.4);
}

void Pentabot::ReturnRandomGoalState(){

    for(int i = 0; i < dof; i++){
        active_state_vector.robots[0].goalPos[i] = 0.0;
        active_state_vector.robots[0].goalVel[i] = 0.0;
    }
}

// TODO - fix
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