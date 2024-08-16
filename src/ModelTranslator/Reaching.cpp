#include "ModelTranslator/Reaching.h"

pandaReaching::pandaReaching(): ModelTranslator(){
    std::string yamlFilePath = "/TaskConfigs/free_motion/reaching.yaml";
    InitModelTranslator(yamlFilePath);
}

bool pandaReaching::TaskComplete(mjData *d, double &dist){
    pose_7 EE_pose;
    MuJoCo_helper->GetBodyPoseQuatViaXpos("franka_gripper", EE_pose, d);

    double diff_x = EE_pose.position(0) - residual_list[0].target[0];
    double diff_y = EE_pose.position(1) - residual_list[0].target[1];
    double diff_z = EE_pose.position(2) - residual_list[0].target[2];

    dist = sqrt(pow(diff_x, 2)
            + pow(diff_y, 2)
            + pow(diff_z, 2));

    if(dist < 0.05){
        return true;
    }
    else{
        return false;
    }
}

void pandaReaching::Residuals(mjData *d, MatrixXd &residuals){
    int resid_index = 0;

    // Compute kinematics chain to compute site poses
//    mj_kinematics(MuJoCo_helper->model, d);
//
//    pose_7 EE_pose;
//    MuJoCo_helper->GetBodyPoseQuatViaXpos("franka_gripper", EE_pose, d);
//    double diff_x = EE_pose.position(0) - residual_list[0].target[0];
//    double diff_y = EE_pose.position(1) - residual_list[0].target[1];
//    double diff_z = EE_pose.position(2) - residual_list[0].target[2];
//    residuals(resid_index++, 0) = sqrt(pow(diff_x, 2)
//                                       + pow(diff_y, 2)
//                                       + pow(diff_z, 2));

    std::vector<double> joint_positions;
    MuJoCo_helper->GetRobotJointsPositions("panda", joint_positions, d);
    for(int i = 0; i < joint_positions.size(); i++){
        residuals(resid_index++, 0) = joint_positions[i] - residual_list[0].target[i];
    }

    std::vector<double> joint_velocities;
    MuJoCo_helper->GetRobotJointsVelocities("panda", joint_velocities, d);

    for(double velocity : joint_velocities){
        residuals(resid_index++, 0) = velocity;
    }

    if(resid_index != residual_list.size()){
        std::cerr << "Error: Residuals size mismatch\n";
        exit(1);
    }
}

void pandaReaching::SetGoalVisuals(mjData *d){

    // Set the joint configurations from the target list
    std::vector<double> joint_positions;
    for(int i = 0; i < 7; i++){
        joint_positions.push_back(residual_list[0].target[i]);
    }
    MuJoCo_helper->SetRobotJointPositions("panda", joint_positions, d);

    // Call mj_kinematics
    mj_kinematics(MuJoCo_helper->model, d);

    // Compute end-effector positions
    pose_6 EE_pose;
    MuJoCo_helper->GetBodyPoseAngleViaXpos("franka_gripper", EE_pose, d);

    // Set EE position for the goal sphere
    pose_6 EE_target_pose;
    EE_target_pose.position(0) = EE_pose.position(0);
    EE_target_pose.position(1) = EE_pose.position(1);
    EE_target_pose.position(2) = EE_pose.position(2);

    MuJoCo_helper->SetBodyPoseAngle("target", EE_target_pose, d);
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

    for(int i = 0; i < full_state_vector.dof; i++){
        current_state_vector.robots[0].start_pos[i] = joint_positions[i];
    }

}

void pandaReaching::ReturnRandomGoalState(){

    double jointOffsets[7] = {0.5, 0.5, 0.5, 0.5, 0.8, 0.8, 0.8};
    double jointOffsetNoise[7] = {0.1, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2};

    bool valid_start_state = false;
    vector<double> joints_positions;

    for(int i = 0; i < 7; i++){
        joints_positions.push_back(0.0);
    }

    while(!valid_start_state){
        // Generate a random robot configuration
        std::string robotName = "panda";
        for(int i = 0; i < 7; i++){
            double target_joint_val;
            if(current_state_vector.robots[0].start_pos[i] - jointOffsets[i] > jointLimsMin[i]){
                target_joint_val = current_state_vector.robots[0].start_pos[i] - jointOffsets[i];
            }
            else{
                target_joint_val = current_state_vector.robots[0].start_pos[i] + jointOffsets[i];
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

    // Get the EE position!
//    pose_6 EE_pose;
//    MuJoCo_helper->GetBodyPoseAngleViaXpos("franka_gripper", EE_pose, MuJoCo_helper->main_data);
//
//    // Sample a 3D point in the workspace?
//    residual_list[0].target[0] = EE_pose.position(0);
//    residual_list[0].target[1] = EE_pose.position(1);
//    residual_list[0].target[2] = EE_pose.position(2);

    // Joint positions
    for(int i = 0; i < 7; i++){
        residual_list[0].target[i] = joints_positions[i];
    }

    // Joint velocities
    for(int i = 0; i < 7; i++){
        residual_list[1].target[i] = 0.0;
    }

}

std::vector<MatrixXd> pandaReaching::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> init_controls;
    int num_ctrl = current_state_vector.num_ctrl;

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
        init_controls.push_back(control);
    }

    return init_controls;
}