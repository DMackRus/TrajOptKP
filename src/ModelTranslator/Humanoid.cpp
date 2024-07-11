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

//    mj_forward(MuJoCo_helper->model, d);
    mj_sensorPos(MuJoCo_helper->model, d);

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
}


