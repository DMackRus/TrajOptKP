#ifndef INTERPOLATED_ILQR_H
#define INTERPOLATED_ILQR_H

#include "optimiser.h"

class interpolatediLQR: public optimiser{
public:
    interpolatediLQR(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator);

    double rolloutTrajectory(int initialDataIndex) override;
    std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIterations, int horizonLength) override;


private:
    double lambda = 0.1;
    double lambdaFactor = 10;

};


#endif