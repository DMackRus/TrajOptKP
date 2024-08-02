#pragma once
#include "ModelTranslator.h"

class Acrobot : public ModelTranslator {
public:
    Acrobot();
    void ReturnRandomStartState() override;
    void ReturnRandomGoalState() override;
    void Residuals(mjData *d, MatrixXd &residuals) override;
    bool TaskComplete(mjData *d, double &dist) override;
};