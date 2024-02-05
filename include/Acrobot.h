#pragma once
#include "ModelTranslator.h"

class Acrobot : public ModelTranslator {
public:
    Acrobot();

    bool TaskComplete(int dataIndex, double &dist) override;
    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;

};