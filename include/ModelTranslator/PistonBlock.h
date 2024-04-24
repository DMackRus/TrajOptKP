#pragma once

#include "ModelTranslator/ModelTranslator.h"

class PistonBlock : public ModelTranslator{
public:
    PistonBlock();

    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;

    void ReturnRandomStartState();
    void ReturnRandomGoalState();

//    bool TaskComplete(mjData *d, double &dist) override;

};