#pragma once

#include "ModelTranslator.h"
#include "PushBaseClass.h"

class TwoDPushing: virtual public ModelTranslator, public PushBaseClass{
public:
    TwoDPushing(int clutterLevel);

    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;
    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;
    std::vector<MatrixXd> CreateInitSetupControls(int horizonLength) override;

//    double CostFunction(mjData *d, bool terminal) override;

//    void CostDerivatives(mjData *d, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal) override;

    bool TaskComplete(mjData *d, double &dist) override;

private:
    int clutterLevel = noClutter;
    double randomGoalX = 0.0;
    double randomGoalY = 0.0;

    Matrix<double, 6, 6> cost_reach;

};
