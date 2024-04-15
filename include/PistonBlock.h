#pragma once

#include "ModelTranslator.h"

class PistonBlock : public ModelTranslator{
public:
    PistonBlock();

    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;

//    void GenerateRandomGoalAndStartState() override;
//    void ReturnRandomStartState() override;
//    void ReturnRandomGoalState() override;

//    bool TaskComplete(mjData *d, double &dist) override;

};