#include "../modelTranslator/modelTranslator.h"
#include "../physicsSimulators/MuJoCoHelper.h"
#include "mujoco.h"

#ifndef PHYSICSSIMSWITCHING_DIFFERENTIATOR_H
#define PHYSICSSIMSWITCHING_DIFFERENTIATOR_H

class differentiator{
public:
    differentiator(modelTranslator *_modelTranslator);

    void getDerivatives(MatrixXd &A, MatrixXd &B, bool costDerivs, int dataIndex);

private:
    modelTranslator *activeModelTranslator;
    MuJoCoHelper *activePhysicsSimulator;
    mjModel *m;

};

#endif