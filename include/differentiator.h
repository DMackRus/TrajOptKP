//#ifndef PHYSICSSIMSWITCHING_DIFFERENTIATOR_H
//#define PHYSICSSIMSWITCHING_DIFFERENTIATOR_H

#ifndef DIFFERENTIATOR_H
#define DIFFERENTIATOR_H

#include "modelTranslator.h"
#include "MuJoCoHelper.h"
#include "mujoco.h"

#define DQACCDQ_MAX 250

class differentiator{
public:
    differentiator(modelTranslator *_modelTranslator, MuJoCoHelper *_physicsSimulator);

    void getDerivatives(MatrixXd &A, MatrixXd &B, MatrixXd &l_x, MatrixXd &l_u, MatrixXd &l_xx, MatrixXd &l_uu, bool costDerivs, int dataIndex, bool terminal);
    void initModelForFiniteDifferencing();
    void resetModelAfterFiniteDifferencing();

private:
    modelTranslator *activeModelTranslator;
    MuJoCoHelper *activePhysicsSimulator;
    mjModel *m;

    int save_iterations;
    mjtNum save_tolerance;

};

#endif