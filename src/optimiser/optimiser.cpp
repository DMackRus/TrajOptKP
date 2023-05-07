
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

    if(costGrad < epsConverge){
        return true;
    }
    return false;
}

void optimiser::setupTestingExtras(int _trajecNumber, int _interpMethod, int _keyPointsMethod, int minN){
    currentTrajecNumber = _trajecNumber;
    interpMethod = _interpMethod;
    keyPointsMethod = _keyPointsMethod;

    min_interval = minN;
}

void optimiser::returnOptimisationData(double &_optTime, double &_costReduction, int &_avgNumDerivs, double &_avgTimeGettingDerivs){

    for(int i = 0; i < numDerivsPerIter.size(); i++){
        avgNumDerivs += numDerivsPerIter[i];
    }
    avgNumDerivs = avgNumDerivs/numDerivsPerIter.size();

    for(int i = 0; i < timeDerivsPerIter.size(); i++){
        avgTimePerDerivs += timeDerivsPerIter[i];
    }
    avgTimePerDerivs = avgTimePerDerivs/timeDerivsPerIter.size();

    _optTime = optTime;
    _costReduction = costReduction;
    _avgNumDerivs = avgNumDerivs;
    _avgTimeGettingDerivs = avgTimePerDerivs;
}

