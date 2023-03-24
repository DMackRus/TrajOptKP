#ifndef STOMP_H
#define STOMP_H

#include "optimiser.h"


class stomp: public optimiser{
public:
    stomp(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator, int _maxHorizon);

    double rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) override;
    std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIterations, int _horizonLength) override;
};


#endif