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

    void getDerivatives(MatrixXd &A, MatrixXd &B, bool costDerivs, int dataIndex);
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