#pragma once

#include "ModelTranslator.h"

class Hopper: public ModelTranslator {
    public:
    Hopper();

    bool TaskComplete(mjData *d, double &dist) override;
    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;

};
