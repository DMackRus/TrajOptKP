#pragma once
#include "modelTranslator.h"

class acrobot : public modelTranslator {
public:
    acrobot();

    bool taskComplete(int dataIndex, double &dist) override;
    void generateRandomGoalAndStartState() override;
    MatrixXd returnRandomStartState() override;
    MatrixXd returnRandomGoalState(MatrixXd X0) override;

};