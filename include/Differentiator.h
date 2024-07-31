// -------------------------
//
//  NOTE - a lot of the ideas in this class are taken from MJPC on github.
//  We dont use the inbuilt function mjd_transitionFD as it doesnt enable us
// to do two key things:
// 1. Choose to not compute derivatives for certain key-points.
// 2. Choose what is in our state vector, rather than considering the whole system

#pragma once

#include "ModelTranslator/ModelTranslator.h"
#include "MuJoCoHelper.h"
#include "mujoco.h"

class Differentiator{
public:
    Differentiator(std::shared_ptr<ModelTranslator> model_translator, std::shared_ptr<MuJoCoHelper> MuJoCo_helper);

    void DynamicsDerivatives(MatrixXd &A, MatrixXd &B, const std::vector<int> &cols,
                             int data_index, int thread_id,
                             bool central_diff, double eps);

    void ResidualDerivatives(vector<MatrixXd> &r_x, vector<MatrixXd> &r_u,
                             int data_index, int tid, bool central_diff, double eps);

    // timing variables
    double time_mj_forwards = 0.0f;
    int count_integrations = 0;

private:

    std::shared_ptr<ModelTranslator> model_translator;
    std::shared_ptr<MuJoCoHelper> MuJoCo_helper;

    int dof = 0;
    int dim_state = 0;
    int num_ctrl = 0;
};