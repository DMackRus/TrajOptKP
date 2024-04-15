#include "PistonBlock.h"

PistonBlock::PistonBlock(){
    std::string task_config = "/taskConfigs/piston_box_config.yaml";
    InitModelTranslator(task_config);
}

std::vector<MatrixXd> PistonBlock::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> init_controls;

    // Set the goal position so that we can see where we are pushing to
    std::string goalMarkerName = "display_goal";
    pose_6 display_goal_pose;
    MuJoCo_helper->GetBodyPoseAngle("display_goal", display_goal_pose, MuJoCo_helper->master_reset_data);
    display_goal_pose.position[0] = active_state_vector.bodiesStates[0].goalLinearPos[0];
    display_goal_pose.position[1] = active_state_vector.bodiesStates[0].goalLinearPos[1];
//    display_goal_pose.position[2] = 0.2f;
    MuJoCo_helper->SetBodyPoseAngle(goalMarkerName, display_goal_pose, MuJoCo_helper->master_reset_data);

    // Create controls where we move forward to contact the box
    for(int t = 0; t < horizonLength; t++){
        MatrixXd control(num_ctrl, 1);
        control(0) = 0.1;
        init_controls.push_back(control);
    }

    return init_controls;
}