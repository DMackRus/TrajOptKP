#include "interpolated_iLQR.h"

interpolatediLQR::interpolatediLQR(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator) : optimiser(_modelTranslator, _physicsSimulator){

}

double interpolatediLQR::rolloutTrajectory(int initialDataIndex){
    double cost = 0.0f;

    return cost;
}

std::vector<MatrixXd> interpolatediLQR::optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIterations, int horizonLength){
    std::vector<MatrixXd> optimisedControls;



    return optimisedControls;
}