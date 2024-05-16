#include "ModelTranslator/PistonBlock.h"

PistonBlock::PistonBlock(){
    std::string task_config = "/taskConfigs/piston_box_config.yaml";
    InitModelTranslator(task_config);
}

void PistonBlock::ReturnRandomStartState(){
    current_state_vector.robots[0].start_pos[0] = 0.0;

    // Initialise the body
    current_state_vector.bodies[0].start_linear_pos[0] = 0.0;
    current_state_vector.bodies[0].start_linear_pos[1] = 0.9;
    current_state_vector.bodies[0].start_linear_pos[2] = 0.1;

    current_state_vector.bodies[0].start_angular_pos[0] = 0.0;
    current_state_vector.bodies[0].start_angular_pos[1] = 0.0;
    current_state_vector.bodies[0].start_angular_pos[2] = 0.0;
}

void PistonBlock::ReturnRandomGoalState(){
    current_state_vector.robots[0].goal_pos[0] = 0.0;
    current_state_vector.robots[0].goal_vel[0] = 0.0;

    // Initialise the body
    current_state_vector.bodies[0].goal_linear_pos[0] = 0.0;
    current_state_vector.bodies[0].goal_linear_pos[1] = randFloat(1.0, 1.5);
    current_state_vector.bodies[0].goal_linear_pos[2] = 0.0;

    current_state_vector.bodies[0].goal_angular_pos[0] = 0.0;
    current_state_vector.bodies[0].goal_angular_pos[1] = 0.0;
    current_state_vector.bodies[0].goal_angular_pos[2] = 0.0;
}

std::vector<MatrixXd> PistonBlock::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> init_controls;

    // Set the goal position so that we can see where we are pushing to
    std::string goalMarkerName = "display_goal";
    pose_6 display_goal_pose;
    MuJoCo_helper->GetBodyPoseAngle("display_goal", display_goal_pose, MuJoCo_helper->master_reset_data);
    display_goal_pose.position[0] = current_state_vector.bodies[0].goal_linear_pos[0];
    display_goal_pose.position[1] = current_state_vector.bodies[0].goal_linear_pos[1];
    MuJoCo_helper->SetBodyPoseAngle(goalMarkerName, display_goal_pose, MuJoCo_helper->master_reset_data);

    // Create controls where we move forward to contact the box
    for(int t = 0; t < horizonLength; t++){
        MatrixXd control(current_state_vector.num_ctrl, 1);
        control(0) = 0.1;
        init_controls.push_back(control);
    }

    return init_controls;
}