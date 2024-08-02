#include "ModelTranslator/Pentabot.h"

Pentabot::Pentabot(): ModelTranslator(){
    std::string yamlFilePath = "/TaskConfigs/toys/pentabot.yaml";
    InitModelTranslator(yamlFilePath);
}

void Pentabot::ReturnRandomStartState(){
    current_state_vector.robots[0].start_pos[0] = 3.1415;
    current_state_vector.robots[0].start_pos[1] = randFloat(-0.1, 0.1);
    current_state_vector.robots[0].start_pos[2] = randFloat(-0.2, 0.2);
    current_state_vector.robots[0].start_pos[3] = randFloat(-0.1, 0.1);
    current_state_vector.robots[0].start_pos[4] = randFloat(-0.4, 0.4);
}

void Pentabot::ReturnRandomGoalState(){

//    for(int i = 0; i < full_state_vector.dof; i++){
//        current_state_vector.robots[0].goal_pos[i] = 0.0;
//        current_state_vector.robots[0].goal_vel[i] = 0.0;
//    }
}

bool Pentabot::TaskComplete(mjData *d, double &dist){
    dist = 0.0;
//    std::vector<double> pentabot_joints;
//    MuJoCo_helper->GetRobotJointsPositions("pentabot", pentabot_joints, d);
//
//    for(int i = 0; i < full_state_vector.dof; i++){
//        std::cout << "joint pos " << pentabot_joints[i] << "\n";
//        dist += abs(current_state_vector.robots[0].goal_pos[i] - pentabot_joints[i]);
//    }
//
//    std::cout << "dist" << dist << std::endl;
//
//    if(dist < 0.01){
//        return true;
//    }
    return false;
}