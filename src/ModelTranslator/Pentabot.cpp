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