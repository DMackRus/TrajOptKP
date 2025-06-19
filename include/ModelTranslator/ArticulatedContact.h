#pragma once
#include "ModelTranslator.h"

class ArticulatedContact : public ModelTranslator {
public:
    ArticulatedContact();
    void ReturnRandomStartState() override;
    void ReturnRandomGoalState() override;
    void Residuals(mjData *d, MatrixXd &residuals) override;
    void SetGoalVisuals(mjData *d) override;
    bool TaskComplete(mjData *d, double &dist) override;
};