#pragma once

#include "ModelTranslator.h"

class Hopper: public ModelTranslator {
    public:
    Hopper();

    bool TaskComplete(int dataIndex, double &dist) override;
    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;

};
