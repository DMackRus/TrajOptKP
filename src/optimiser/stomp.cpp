#include "stomp.h"


stomp::stomp(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator, int _maxHorizon): optimiser(_modelTranslator, _physicsSimulator){

}

double stomp::rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls){

}

std::vector<MatrixXd> stomp::optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIterations, int _horizonLength){

}