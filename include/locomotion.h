#pragma once

#include "ModelTranslator.h"

class walker : public ModelTranslator {
public:
    walker();

    bool TaskComplete(int dataIndex, double &dist) override;
    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;

    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;

    double CostFunction(int dataIndex, bool terminal) override;
    void CostDerivatives(int dataIndex, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal) override;

};
