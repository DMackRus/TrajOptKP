#include "ModelTranslator/Reaching.h"

pandaReaching::pandaReaching(): ModelTranslator(){
    std::string yamlFilePath = "/taskConfigs/reachingConfig.yaml";

    InitModelTranslator(yamlFilePath);
}

bool pandaReaching::TaskComplete(mjData *d, double &dist){
    double cum_error = 0.0f;

    std::vector<double> robot_joints;
    MuJoCo_helper->GetRobotJointsPositions("panda", robot_joints, d);

    for(int i = 0; i < full_state_vector.dof; i++){
        double diff = full_state_vector.robots[0].goalPos[i] - robot_joints[i];
        cum_error += diff;
    }

    dist = cum_error;

    if(cum_error < 0.05){
        return true;
    }
    else{
        return false;
    }
}

//void pandaReaching::GenerateRandomGoalAndStartState() {
//    bool validStartAndGoal = false;
//    vector<double> jointStartPositions;
//    vector<double> jointGoalPositions;
//
//    while(!validStartAndGoal){
//
//        // Generate a random starting state
//        bool validStartState = false;
//
//
//        for(int i = 0; i < 7; i++){
//            jointStartPositions.push_back(0.0f);
//        }
//
//        while(!validStartState){
//            // Generate a random robot configuration
//            std::string robotName = "panda";
//            for(int i = 0; i < 7; i++){
//                float randomJoint = randFloat(jointLimsMin[i], jointLimsMax[i]);
//                jointStartPositions[i] = randomJoint;
//            }
//
//            MuJoCo_helper->SetRobotJointPositions(robotName, jointStartPositions, MuJoCo_helper->main_data);
//
//            // Check if current configuration is valid
//            if(MuJoCo_helper->CheckSystemForCollisions(MuJoCo_helper->main_data)){
//                cout << "invalid robot position \n";
//            }
//            else{
//                validStartState = true;
//            }
//        }
//
//        // Generate a random goal state
//        double jointOffsets[7] = {0.5, 0.5, 0.5, 0.5, 0.8, 0.8, 0.8};
//        double jointOffsetNoise[7] = {0.1, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2};
//        int resetCounter = 0;
//
//        bool validGoalState = false;
//
//        for(int i = 0; i < 7; i++){
//            jointGoalPositions.push_back(0.0f);
//        }
//
//        while(!validGoalState){
//            // Generate a random robot configuration
//            std::string robotName = "panda";
//            for(int i = 0; i < 7; i++){
//                float targetJointVal;
//                if(jointStartPositions[i] - jointOffsets[i] > jointLimsMin[i]){
//                    targetJointVal = jointStartPositions[i] - jointOffsets[i];
//                }
//                else{
//                    targetJointVal = jointStartPositions[i] + jointOffsets[i];
//                }
//
//                float randomJoint = randFloat(targetJointVal - jointOffsetNoise[i], targetJointVal + jointOffsetNoise[i]);
//                jointGoalPositions[i] = randomJoint;
//            }
//
//            MuJoCo_helper->SetRobotJointPositions(robotName, jointGoalPositions, MuJoCo_helper->main_data);
//
//            // Check if current configuration is valid
//            if(MuJoCo_helper->CheckSystemForCollisions(MuJoCo_helper->main_data)){
//                cout << "invalid robot position \n";
//                resetCounter++;
//            }
//            else{
//                validGoalState = true;
//            }
//
//            if(resetCounter > 100){
//                cout << "regenerate random start state \n";
//            }
//
//        }
//
//        if(validStartState && validGoalState){
//            validStartAndGoal = true;
//        }
//
//    }
//
//    for(int i = 0; i < dof; i++){
//        active_state_vector.robots[0].startPos[i] = jointStartPositions[i];
//        active_state_vector.robots[0].goalPos[i] = jointGoalPositions[i];
//    }
//}

void pandaReaching::ReturnRandomStartState(){
    bool valid_start_state = false;
    vector<double> joint_positions;

    for(int i = 0; i < 7; i++){
        joint_positions.push_back(0.0f);
    }

    while(!valid_start_state){
        // Generate a random robot configuration
        std::string robotName = "panda";
        for(int i = 0; i < 7; i++){
            double random_joint_pos = randFloat(jointLimsMin[i], jointLimsMax[i]);
            joint_positions[i] = random_joint_pos;
        }

        MuJoCo_helper->SetRobotJointPositions(robotName, joint_positions, MuJoCo_helper->main_data);

        // Check if current configuration is valid
        if(MuJoCo_helper->CheckSystemForCollisions(MuJoCo_helper->main_data)){
            cout << "invalid robot position \n";
        }
        else{
            valid_start_state = true;
        }
    }

    for(int i = 0; i < dof; i++){
        current_state_vector.robots[0].startPos[i] = joint_positions[i];
    }

}

void pandaReaching::ReturnRandomGoalState(){

    double jointOffsets[7] = {0.5, 0.5, 0.5, 0.5, 0.8, 0.8, 0.8};
    double jointOffsetNoise[7] = {0.1, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2};

    bool valid_start_state = false;
    vector<double> joints_positions;

    for(int i = 0; i < 7; i++){
        joints_positions.push_back(0.0f);
    }

    while(!valid_start_state){
        // Generate a random robot configuration
        std::string robotName = "panda";
        for(int i = 0; i < 7; i++){
            double target_joint_val;
            if(current_state_vector.robots[0].startPos[i] - jointOffsets[i] > jointLimsMin[i]){
                target_joint_val = current_state_vector.robots[0].startPos[i] - jointOffsets[i];
            }
            else{
                target_joint_val = current_state_vector.robots[0].startPos[i] + jointOffsets[i];
            }

            double randomJoint = randFloat(target_joint_val - jointOffsetNoise[i], target_joint_val + jointOffsetNoise[i]);
            joints_positions[i] = randomJoint;
        }

        MuJoCo_helper->SetRobotJointPositions(robotName, joints_positions, MuJoCo_helper->main_data);

        // Check if current configuration is valid
        if(MuJoCo_helper->CheckSystemForCollisions(MuJoCo_helper->main_data)){
            cout << "invalid robot position \n";
        }
        else{
            valid_start_state = true;
        }

    }

    for(int i = 0; i < dof; i++){
        current_state_vector.robots[0].goalPos[i] = joints_positions[i];
        current_state_vector.robots[0].goalVel[i] = 0.0;
    }
}

std::vector<MatrixXd> pandaReaching::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    if(current_state_vector.robots[0].torqueControlled){

        MatrixXd control(num_ctrl, 1);
        double gains[7] = {10, 10, 10, 10, 5, 5, 5};
//        MatrixXd Xt;
        vector<double> gravCompensation;
        for(int i = 0; i < horizonLength; i++){

            MuJoCo_helper->GetRobotJointsGravityCompensationControls(current_state_vector.robots[0].name, gravCompensation, MuJoCo_helper->main_data);

//            Xt = ReturnStateVector(MuJoCo_helper->main_data);

            for(int j = 0; j < num_ctrl; j++){
                control(j) = gravCompensation[j];
//                double diff = X_desired(j) - Xt(j);
//                control(j) += diff * gains[j];
            }

            SetControlVector(control, MuJoCo_helper->main_data, full_state_vector);
            mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);
            initControls.push_back(control);
        }
    }
    else{

    }

    

    return initControls;
}