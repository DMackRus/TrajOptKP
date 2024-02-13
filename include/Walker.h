#pragma once

#include "ModelTranslator.h"

enum terrains{
    PLANE,
    UNEVEN
};

class walker : public ModelTranslator {
public:
    walker(int terrain);

    bool TaskComplete(mjData *d, double &dist) override;
    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;

    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;

    double CostFunction(mjData *d, bool terminal) override;
    void CostDerivatives(mjData *d, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal) override;

};
