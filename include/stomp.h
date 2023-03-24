#ifndef STOMP_H
#define STOMP_H

#include "optimiser.h"


class stomp: public optimiser{
public:
    stomp(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator, int _maxHorizon, int rolloutsPerIter);

    double rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) override;
    std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIterations, int _horizonLength) override;


    MatrixXd returnNoisyControl(MatrixXd Ut, MatrixXd noise);
    std::vector<MatrixXd> createNoisyTrajec(std::vector<MatrixXd> nominalTrajectory);

    MatrixXd noiseProfile;
    int maxHorizon;

private:
    int rolloutsPerIter;

    vector<vector<MatrixXd>> U_noisy;
    vector<MatrixXd> U_best;

    

};


#endif