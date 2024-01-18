#pragma once

#include "ModelTranslator.h"

class hopper: public ModelTranslator {
    public:
    hopper();

    bool TaskComplete(int dataIndex, double &dist) override;
    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;

};
