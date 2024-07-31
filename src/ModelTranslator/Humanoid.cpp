#include "ModelTranslator/Humanoid.h"

Humanoid::Humanoid(){
    std::string yamlFilePath = "/TaskConfigs/locomotion/humanoid.yaml";
    InitModelTranslator(yamlFilePath);
}

bool Humanoid::TaskComplete(mjData *d, double &dist){
    dist = 0;
    return false;
}
void Humanoid::Residuals(mjData *d, MatrixXd &residuals){
    int resid_index = 0;

    mj_forward(MuJoCo_helper->model, d);
//    mj_forwardSkip()
//    mj_sensorPos(MuJoCo_helper->model, d);
//    mj_sensorVel(MuJoCo_helper->model, d);
//    mj_sensorAcc(MuJoCo_helper->model, d);

    // --------------- Residual 1 - Stand upright ----------------
    double* f1_position = MuJoCo_helper->SensorState(d, "sp0");
    double* f2_position = MuJoCo_helper->SensorState(d, "sp1");
    double* f3_position = MuJoCo_helper->SensorState(d, "sp2");
    double* f4_position = MuJoCo_helper->SensorState(d, "sp3");
    double* head_position = MuJoCo_helper->SensorState(d, "head_position");
    double head_feet_error =
            head_position[2] - 0.25 * (f1_position[2] + f2_position[2] +
                                       f3_position[2] + f4_position[2]);
    // Residual 1 - Stand upright
    residuals(resid_index++, 0) = head_feet_error - 1.5;
//    residuals(resid_index++, 0) = head_position[2] - 1.5;

    // ---------------- Residual 2 -  Balance ------------------------
    double* com_position = MuJoCo_helper->SensorState(d, "torso_subtreecom");
    double* com_velocity = MuJoCo_helper->SensorState(d, "torso_subtreelinvel");
    double kFallTime = 0.2;
    double capture_point[3] = {com_position[0], com_position[1], com_position[2]};
    mju_addToScl3(capture_point, com_velocity, kFallTime);

    // average feet xy position
    double fxy_avg[2] = {0.0};
    mju_addTo(fxy_avg, f1_position, 2);
    mju_addTo(fxy_avg, f2_position, 2);
    mju_addTo(fxy_avg, f3_position, 2);
    mju_addTo(fxy_avg, f4_position, 2);
    mju_scl(fxy_avg, fxy_avg, 0.25, 2);

    mju_subFrom(fxy_avg, capture_point, 2);
    double com_feet_distance = mju_norm(fxy_avg, 2);
    residuals(resid_index++, 0) = com_feet_distance;

    // ---------------- Residuals 3 - 24 - Controls -------------------------
    MatrixXd controls = ReturnControlVector(d, full_state_vector);
    for (int i = 0; i < controls.rows(); i++){
        residuals(resid_index++, 0) = controls(i, 0);
    }
}


