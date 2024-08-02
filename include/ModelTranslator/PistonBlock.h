#pragma once

#include "ModelTranslator/ModelTranslator.h"

class PistonBlock : public ModelTranslator{
public:
    PistonBlock();

    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;

    void ReturnRandomStartState();
    void ReturnRandomGoalState();

    void Residuals(mjData *d, MatrixXd &residuals) override;

    void SetGoalVisuals(mjData *d) override;

//    bool TaskComplete(mjData *d, double &dist) override;

};