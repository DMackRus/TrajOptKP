#pragma once

#include "ModelTranslator.h"

class humanoid: public ModelTranslator {
public:
    humanoid();

    bool TaskComplete(mjData *d, double &dist) override;
    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;

};

