#pragma once

#include "ModelTranslator/ModelTranslator.h"

class pandaReaching : public ModelTranslator {
public:
    pandaReaching();

    bool TaskComplete(mjData *d, double &dist) override;
    void ReturnRandomStartState() override;
    void ReturnRandomGoalState() override;
    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;

    void Residuals(mjData *d, MatrixXd &residuals) override;

    void SetGoalVisuals(mjData *d) override;

    float jointLimsMax[7] = {2.2, 0.6, 2.97, 0, 2.97, 1.5, 2.5};
    float jointLimsMin[7] = {-2.2, -0.6, -2.97, -1.5, -2.97, -1.1, -2.5};
};