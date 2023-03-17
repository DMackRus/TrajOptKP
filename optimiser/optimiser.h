#ifndef OPTIMISER_H
#define OPTIMISER_H


#include "../stdInclude/stdInclude.h"
#include "../modelTranslator/modelTranslator.h"
#include "../physicsSimulators/physicsSimulator.h"

class optimiser{
public:
    optimiser(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator);

    virtual double rolloutTrajectory(int initialDataIndex) = 0;
    virtual std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIterations, int horizonLength) = 0;

private:
    modelTranslator *activeModelTranslator;
    physicsSimulator *activePhysicsSimulator;

};



#endif