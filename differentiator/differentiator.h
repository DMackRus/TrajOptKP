#include "../modelTranslator/modelTranslator.h"
#include "../physicsSimulators/MuJoCoHelper.h"
#include "mujoco.h"


class differentiator{
public:
    differentiator(modelTranslator *_modelTranslator, MuJoCoHelper *_physicsSimulator);

    getDerivatives(MatrixXd &A, MatrixXd &B, bool costDerivs, mjData *d, mjModel *m);

private:
    modelTranslator *activeModelTranslator;
    MuJoCoHelper *activePhysicsSimulator;

};