//#ifndef PHYSICSSIMSWITCHING_DIFFERENTIATOR_H
//#define PHYSICSSIMSWITCHING_DIFFERENTIATOR_H

#ifndef DIFFERENTIATOR_H
#define DIFFERENTIATOR_H

#define USE_DQACC 1
#define HESSIAN_APPROXIMATION 0

#include "modelTranslator.h"
#include "MuJoCoHelper.h"
#include "mujoco.h"

#define DQACCDQ_MAX 100

class differentiator{
public:
    differentiator(std::shared_ptr<modelTranslator> _modelTranslator, std::shared_ptr<MuJoCoHelper> _physicsSimulator);

    void getDerivatives(MatrixXd &A, MatrixXd &B, std::vector<int> cols, MatrixXd &l_x, MatrixXd &l_u, MatrixXd &l_xx, MatrixXd &l_uu, bool costDerivs, int dataIndex, bool terminal, int threadId);
    MatrixXd calc_dqveldctrl(std::vector<int> cols, int dataIndex, int physicsHelperId, MatrixXd &dcostdctrl, bool fd_costDerivs, bool terminal);
    MatrixXd calc_dqaccdctrl(std::vector<int> cols, int dataIndex, int physicsHelperId, MatrixXd &dcostdctrl, bool fd_costDerivs, bool terminal);

    MatrixXd calc_dqveldqvel(std::vector<int> cols, int dataIndex, int physicsHelperId, MatrixXd &dcostdvel, bool fd_costDerivs, bool terminal);
    MatrixXd calc_dqaccdqvel(std::vector<int> cols, int dataIndex, int physicsHelperId, MatrixXd &dcostdvel, bool fd_costDerivs, bool terminal);

    MatrixXd calc_dqveldqpos(std::vector<int> cols, int dataIndex, int physicsHelperId, MatrixXd &dcostdpos, bool fd_costDerivs, bool terminal);
    MatrixXd calc_dqaccdqpos(std::vector<int> cols, int dataIndex, int physicsHelperId, MatrixXd &dcostdpos, bool fd_costDerivs, bool terminal);

private:
    double epsControls = 1e-6;
    double epsVelocities = 1e-6;
    double epsPositions = 1e-6;

    std::shared_ptr<modelTranslator> activeModelTranslator;
    std::shared_ptr<MuJoCoHelper> activePhysicsSimulator;
};

#endif