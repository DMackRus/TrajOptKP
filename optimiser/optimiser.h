#ifndef OPTIMISER_H
#define OPTIMISER_H


#include "../stdInclude/stdInclude.h"
#include "../modelTranslator/modelTranslator.h"
#include "../physicsSimulators/physicsSimulator.h"

class optimiser{
public:
    optimiser(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator);

    virtual double rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) = 0;
    virtual std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIterations, int _horizonLength) = 0;
    virtual bool checkForConvergence(double oldCost, double newCost);

protected:
    modelTranslator *activeModelTranslator;
    physicsSimulator *activePhysicsSimulator;

    int dof;
    int num_ctrl;
    int horizonLength;

private:
    double epsConverge = 0.02;
    

};



#endif