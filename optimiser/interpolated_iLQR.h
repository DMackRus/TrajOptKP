#ifndef INTERPOLATED_ILQR_H
#define INTERPOLATED_ILQR_H

#include "optimiser.h"

class interpolatediLQR: public optimiser{
public:
    interpolatediLQR(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator);

    double rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) override;
    std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIterations, int horizonLength) override;

    void generateEvalWaypoints();

    void getDerivatvies();

    void backwardsPass_Quu_reg();
    void forwardsPass();



private:
    double lambda = 0.1;
    double lambdaFactor = 10;

};


#endif