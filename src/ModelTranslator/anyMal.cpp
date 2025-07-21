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

    std::vector<double> anyMal_controls;
    MuJoCo_helper->GetRobotJointsControls("anyMal", anyMal_controls, d);

    double body_x = d->site_xpos[3*0];
    double body_y = d->site_xpos[3*0+1];
    double body_z = d->site_xpos[3*0+2];

    // ------------ Residual 0: Body height -------------
    residuals(resid_index++, 0) = body_z - residual_list[0].target[0];

    // --------------- Residual 1: Body x ---------------
    residuals(resid_index++, 0) = body_x - residual_list[1].target[0];

    // --------------- Residual 2: Body y ---------------
    residuals(resid_index++, 0) = body_y - residual_list[2].target[0];

    // --------------- Residual 3: Joints controls -------------
    for(int i = 0; i < anyMal_controls.size(); i++){
        residuals(resid_index++, 0) = anyMal_controls[i] - residual_list[3+i].target[0];
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