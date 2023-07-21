//#ifndef PHYSICSSIMSWITCHING_DIFFERENTIATOR_H
//#define PHYSICSSIMSWITCHING_DIFFERENTIATOR_H

#ifndef DIFFERENTIATOR_H
#define DIFFERENTIATOR_H

#include "modelTranslator.h"
#include "MuJoCoHelper.h"
#include "mujoco.h"

#define DQACCDQ_MAX 100

class differentiator{
public:
    differentiator(std::shared_ptr<modelTranslator> _modelTranslator, std::shared_ptr<MuJoCoHelper> _physicsSimulator);

    void getDerivatives(MatrixXd &A, MatrixXd &B, std::vector<int> cols, MatrixXd &l_x, MatrixXd &l_u, MatrixXd &l_xx, MatrixXd &l_uu, bool costDerivs, int dataIndex, bool terminal);
//    MatrixXd calc_dqveldctrl(std::vector<int> cols, int dataIndex, int physicsHelperId);
//    MatrixXd calc_dqveldqvel(std::vector<int> cols, int dataIndex, int physicsHelperId);
//    MatrixXd calc_dqveldqpos(std::vector<int> cols, int dataIndex, int physicsHelperId);

private:
//    modelTranslator *activeModelTranslator;
//    MuJoCoHelper *activePhysicsSimulator;
    std::shared_ptr<modelTranslator> activeModelTranslator;
    std::shared_ptr<MuJoCoHelper> activePhysicsSimulator;
//    mjModel *m;

};

#endif