#pragma once

#include "ModelTranslator.h"

class Pentabot : public ModelTranslator{

public:
    Pentabot();

    bool TaskComplete(mjData *d, double &dist) override;
    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;
};
