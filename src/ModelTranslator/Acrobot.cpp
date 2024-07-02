#include "ModelTranslator/Acrobot.h"

Acrobot::Acrobot(): ModelTranslator(){
    std::string yamlFilePath = "/TaskConfigs/toys/acrobot.yaml";
    InitModelTranslator(yamlFilePath);
}

bool Acrobot::TaskComplete(mjData *d, double &dist){

    dist = 0.0;
    std::vector<double> acrobot_joints;
    MuJoCo_helper->GetRobotJointsPositions("acrobot", acrobot_joints, d);

    for(int i = 0; i < full_state_vector.dof; i++){
        std::cout << "joint pos " << acrobot_joints[i] << "\n";
        dist += abs(current_state_vector.robots[0].goal_pos[i] - acrobot_joints[i]);
    }

    if(dist < 0.01){
        return true;
    }

    return false;
}

MatrixXd Acrobot::Residuals(mjData *d, const struct stateVectorList &state_vector){
    MatrixXd residuals(2*state_vector.dof + state_vector.num_ctrl, 1);

    std::vector<double> acrobot_joints;
    std::vector<double> acrobot_velocities;
    std::vector<double> acrobot_control;
    MuJoCo_helper->GetRobotJointsPositions("acrobot", acrobot_joints, d);
    MuJoCo_helper->GetRobotJointsVelocities("acrobot", acrobot_velocities, d);
    MuJoCo_helper->GetRobotJointsControls("acrobot", acrobot_control, d);

    for(int i = 0; i < state_vector.dof; i++){
        residuals(i, 0) = acrobot_joints[i] - state_vector.robots[0].goal_pos[i];
        residuals(i+state_vector.dof, 0) = acrobot_velocities[i] - state_vector.robots[0].goal_vel[i];
    }

    residuals(4, 0) = acrobot_control[0];

    return residuals;
}

//MatrixXd Acrobot::ReturnRandomStartState(){
//    MatrixXd randomStartState(state_vector_size, 1);
//
//    float arm1Pos = randFloat(0, 3);
//    float arm2Pos = randFloat(0, 3);
//
//    randomStartState << arm1Pos, arm2Pos, 0, 0;
//
//    return randomStartState;
//}
//
//MatrixXd Acrobot::ReturnRandomGoalState(MatrixXd X0){
//    MatrixXd randomGoalState(state_vector_size, 1);
//
//    float randomNum = randFloat(0, 1);
//    // stable down position
//    if(randomNum < 0.33){
//        randomGoalState << 3.1415, 0, 0, 0;
//    }
//        // Half up unstable
//    else if(randomNum > 0.33 && randomNum < 0.66){
//        randomGoalState << 3.1415, 3.1415, 0, 0;
//    }
//        // Unstable up position
//    else{
//        randomGoalState << 0, 0, 0, 0;
//    }
//
//    return randomGoalState;
//}
