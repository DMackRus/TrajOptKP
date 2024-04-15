#pragma once

#include "ModelTranslator.h"

class PistonBlock : public ModelTranslator{
public:
    PistonBlock();

    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;

    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;

    bool TaskComplete(mjData *d, double &dist) override;

};