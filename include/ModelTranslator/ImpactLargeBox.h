#pragma once

#include "ModelTranslator.h"

class ImpactLargeBox: virtual public ModelTranslator{
public:
    ImpactLargeBox();

    void ReturnRandomStartState() override;
    void ReturnRandomGoalState() override;

    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;

    void SetGoalVisuals(mjData *d) override;
    void Residuals(mjData *d, MatrixXd &residuals) override;

private:
    double randomGoalX = 0.0;
    double randomGoalY = 0.0;
};