#ifndef STOMP_H
#define STOMP_H

#include "Optimiser/Optimiser.h"


class PredictiveSampling: public Optimiser{
public:
    PredictiveSampling(std::shared_ptr<ModelTranslator> _modelTranslator, std::shared_ptr<MuJoCoHelper> MuJoCo_helper, std::shared_ptr<FileHandler> _yamlReader, std::shared_ptr<Differentiator> _differentiator, int _maxHorizon, int _rolloutsPerIter);

    double RolloutTrajectory(mjData *d, bool saveStates, std::vector<MatrixXd> initControls) override;
    std::vector<MatrixXd> Optimise(mjData *d, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength) override;

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