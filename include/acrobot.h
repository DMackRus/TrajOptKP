#pragma once
#include "ModelTranslator.h"

class acrobot : public ModelTranslator {
public:
    acrobot();

    bool TaskComplete(int dataIndex, double &dist) override;
    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;

};