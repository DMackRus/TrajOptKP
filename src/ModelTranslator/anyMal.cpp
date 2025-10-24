#include "ModelTranslator/anyMal.h"

anyMal::anyMal() : ModelTranslator() {
    InitModelTranslator("/TaskConfigs/locomotion/anyMal.yaml");
}

void anyMal::ReturnRandomGoalState() {

}

void anyMal::ReturnRandomStartState() {
    double start_config[12] = {0, 0, 0, 1, -1, 0.2, 0, 0, 0};

    for(int i = 0; i < 12; i++){
        current_state_vector.robots[0].start_pos[i] = start_config[i];
    }
}

void anyMal::Residuals(mjData *d, MatrixXd &residuals) {
    int resid_index = 0;

    mj_kinematics(MuJoCo_helper->model, d);
//    mj_fwdActuation(MuJoCo_helper->model, d);
//    mj_forward(MuJoCo_helper->model, d);

    std::vector<double> anyMal_controls;
    MuJoCo_helper->GetRobotJointsControls("anyMal", anyMal_controls, d);
//    for(int i = 0; i < MuJoCo_helper->model->nu; i++){
//        anyMal_controls.push_back(d->actuator_force[i]);
//    }

    double body_x = d->site_xpos[3*0];
    double body_y = d->site_xpos[3*0+1];
    double body_z = d->site_xpos[3*0+2];

    // ------------ Residual 0: Body height -------------
    residuals(resid_index++, 0) = body_z - residual_list[0].target[0];

    // ------------- Residual 1: body upright -------------
    pose_7 body_pose;
    MuJoCo_helper->GetBodyPoseQuat("body", body_pose, d);
    m_point temp_eul = quat2Eul(body_pose.quat);
    Eigen::Matrix3d current_rot_mat = eul2RotMat(temp_eul);

    // TODO - Temporary, unit quaternion
    m_quat desired = {1, 0, 0, 0};
    temp_eul = quat2Eul(desired);
    Eigen::Matrix3d desired_rot_mat = eul2RotMat(temp_eul);

    double dot_x, dot_y, dot_z;
//    dot_x = acos(current_rot_mat(0, 0) * desired_rot_mat(0, 0)
//                 + current_rot_mat(1, 0) * desired_rot_mat(1, 0)
//                 + current_rot_mat(2, 0) * desired_rot_mat(2, 0));
//    dot_y = acos(current_rot_mat(0, 1) * desired_rot_mat(0, 1)
//                 + current_rot_mat(1, 1) * desired_rot_mat(1, 1)
//                 + current_rot_mat(2, 1) * desired_rot_mat(2, 1));
    dot_z = acos(current_rot_mat(0, 2) * desired_rot_mat(0, 2)
                 + current_rot_mat(1, 2) * desired_rot_mat(1, 2)
                 + current_rot_mat(2, 2) * desired_rot_mat(2, 2));

//    std::cout << "dot_z: " << dot_z << std::endl;

    residuals(resid_index++, 0) = dot_z;


    // --------------- Residual 1: Body x ---------------
//    residuals(resid_index++, 0) = body_x - residual_list[1].target[0];
//
//    // --------------- Residual 2: Body y ---------------
//    residuals(resid_index++, 0) = body_y - residual_list[2].target[0];

    // --------------- Residual 2 onwards: Joints controls -------------
    for(int i = 0; i < anyMal_controls.size(); i++){
        residuals(resid_index++, 0) = anyMal_controls[i] - residual_list[2+i].target[0];
    }

    if(resid_index != residual_list.size()){
        std::cerr << "Error: Residuals size mismatch\n";
        exit(1);
    }
}

bool anyMal::TaskComplete(mjData *d, double &dist) {
    dist = 0.0;
    return false;
}