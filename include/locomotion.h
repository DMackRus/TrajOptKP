#pragma once

#include "modelTranslator.h"

class walker : public modelTranslator {
public:
    walker();

    bool taskComplete(int dataIndex, double &dist) override;
    void generateRandomGoalAndStartState() override;
    MatrixXd returnRandomStartState() override;
    MatrixXd returnRandomGoalState(MatrixXd X0) override;

    std::vector<MatrixXd> createInitOptimisationControls(int horizonLength) override;

    double costFunction(int dataIndex, bool terminal) override;
    void costDerivatives(int dataIndex, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal) override;

};
