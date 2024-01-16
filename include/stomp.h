#ifndef STOMP_H
#define STOMP_H

#include "optimiser.h"


class stomp: public optimiser{
public:
    stomp(std::shared_ptr<ModelTranslator> _modelTranslator, std::shared_ptr<physicsSimulator> _physicsSimulator, std::shared_ptr<fileHandler> _yamlReader, std::shared_ptr<differentiator> _differentiator, int _maxHorizon, int _rolloutsPerIter);

    double rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) override;
    std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength) override;

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