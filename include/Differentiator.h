#pragma once

#define USE_DQACC 0

#include "ModelTranslator.h"
#include "MuJoCoHelper.h"
#include "mujoco.h"

//#define DQACCDQ_MAX 100

class Differentiator{
public:
    Differentiator(std::shared_ptr<ModelTranslator> _modelTranslator, std::shared_ptr<MuJoCoHelper> _MuJoCo_helper);

    void getDerivatives(MatrixXd &A, MatrixXd &B, const std::vector<int> &cols,
                        MatrixXd &l_x, MatrixXd &l_u, MatrixXd &l_xx,
                        MatrixXd &l_uu, bool costDerivs, int dataIndex, bool terminal, int threadId);

    void calc_dqveldctrl(MatrixXd &dqveldctrl, const std::vector<int> &cols, int dataIndex,
                         int tid, MatrixXd &dcostdctrl, bool fd_costDerivs, bool terminal);

    void calc_dqaccdctrl(MatrixXd &dqaccdctrl, const std::vector<int> &cols, int dataIndex,
                         int tid, MatrixXd &dcostdctrl, bool fd_costDerivs, bool terminal);

    void calc_dqveldqvel(MatrixXd &dqveldqvel, const std::vector<int> &cols, int dataIndex,
                         int tid, MatrixXd &dcostdvel, bool fd_costDerivs, bool terminal);

    void calc_dqaccdqvel(MatrixXd &dqaccdqvel, const std::vector<int> &cols, int dataIndex,
                         int tid, MatrixXd &dcostdvel, bool fd_costDerivs, bool terminal);

    void calc_dqveldqpos(MatrixXd &dqveldqpos, const std::vector<int> &cols, int dataIndex,
                         int tid, MatrixXd &dcostdpos, bool fd_costDerivs, bool terminal);

    void calc_dqaccdqpos(MatrixXd &dqaccdqpos, const std::vector<int> &cols, int dataIndex,
                         int tid, MatrixXd &dcostdpos, bool fd_costDerivs, bool terminal);

private:
    double epsControls = 1e-6;
    double epsVelocities = 1e-6;
    double epsPositions = 1e-6;

    std::shared_ptr<ModelTranslator> activeModelTranslator;
    std::shared_ptr<MuJoCoHelper> MuJoCo_helper;

    // temp variable
    double time_mj_forwards = 0.0f;
    int count_integrations = 0;

    int dof = 0;
    int num_ctrl = 0;
};