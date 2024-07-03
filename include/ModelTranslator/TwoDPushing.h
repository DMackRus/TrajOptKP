#pragma once

#include "ModelTranslator/ModelTranslator.h"
#include "ModelTranslator/PushBaseClass.h"

class TwoDPushing: virtual public ModelTranslator, public PushBaseClass{
public:
    TwoDPushing(int clutterLevel);

    void ReturnRandomStartState() override;
    void ReturnRandomGoalState() override;
    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;
    std::vector<MatrixXd> CreateInitSetupControls(int horizonLength) override;

    void InstantiateResiduals() override;
    MatrixXd Residuals(mjData *d) override;

    bool TaskComplete(mjData *d, double &dist) override;

private:
    int clutterLevel = noClutter;
    double randomGoalX = 0.0;
    double randomGoalY = 0.0;

    Matrix<double, 6, 6> cost_reach;

};
