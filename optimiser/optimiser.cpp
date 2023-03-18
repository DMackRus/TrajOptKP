
#include "optimiser.h"

optimiser::optimiser(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator){
    std::cout << "initialised optimiser \n";
    activeModelTranslator = _modelTranslator;
    activePhysicsSimulator = _physicsSimulator;

    dof = activeModelTranslator->dof;
    num_ctrl = activeModelTranslator->num_ctrl;
}

bool optimiser::checkForConvergence(double oldCost, double newCost){
    double costGrad = (oldCost - newCost)/newCost;
    cout << "old cost: " << oldCost << endl;
    cout << "new cost: " << newCost << endl;
    cout << "cost grad: " << costGrad << endl;

    if(costGrad < epsConverge){
        return true;
    }
    return false;
}

