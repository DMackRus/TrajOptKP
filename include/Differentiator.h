#pragma once

#define USE_DQACC 0

#include "ModelTranslator.h"
#include "MuJoCoHelper.h"
#include "mujoco.h"

class Differentiator{
public:
    Differentiator(std::shared_ptr<ModelTranslator> _modelTranslator, std::shared_ptr<MuJoCoHelper> _MuJoCo_helper);

    void ComputeDerivatives(MatrixXd &A, MatrixXd &B, const std::vector<int> &cols,
                            MatrixXd &l_x, MatrixXd &l_u, MatrixXd &l_xx, MatrixXd &l_uu,
                            int dataIndex, int threadId, bool terminal, bool costDerivs,
                            bool central_diff, double eps);

    void FD_Controls(MatrixXd &dqveldctrl, MatrixXd &dqposdctrl, const std::vector<int> &cols,
                     int dataIndex, int tid, bool central_diff, double eps,
                     MatrixXd &dcostdctrl, bool fd_costDerivs, bool terminal);

    void FD_Velcoities(MatrixXd &dqveldqvel, MatrixXd &dqposdqvel, const std::vector<int> &cols,
                       int dataIndex, int tid, bool central_diff, double eps,
                       MatrixXd &dcostdvel, bool fd_costDerivs, bool terminal);

    void FD_Positions(MatrixXd &dqveldqpos, MatrixXd &dqposdqpos, const std::vector<int> &cols,
                      int dataIndex, int tid, bool central_diff, double eps,
                      MatrixXd &dcostdpos, bool fd_costDerivs, bool terminal);

    // Just use mj_forward to compute qacc. Then integrate them.
    void calc_dqaccdctrl(MatrixXd &dqaccdctrl, const std::vector<int> &cols, int dataIndex,
                         int tid, MatrixXd &dcostdctrl, bool fd_costDerivs, bool terminal);

    void calc_dqaccdqvel(MatrixXd &dqaccdqvel, const std::vector<int> &cols, int dataIndex,
                         int tid, MatrixXd &dcostdvel, bool fd_costDerivs, bool terminal);

    void calc_dqaccdqpos(MatrixXd &dqaccdqpos, const std::vector<int> &cols, int dataIndex,
                         int tid, MatrixXd &dcostdpos, bool fd_costDerivs, bool terminal);

    double time_mj_forwards = 0.0f;
    int count_integrations = 0;

private:

    std::shared_ptr<ModelTranslator> activeModelTranslator;
    std::shared_ptr<MuJoCoHelper> MuJoCo_helper;

    // temp variable

    int dof = 0;
    int dim_state = 0;
    int num_ctrl = 0;
};