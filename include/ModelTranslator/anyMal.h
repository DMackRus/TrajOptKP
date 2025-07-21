#pragma once

#include "ModelTranslator/ModelTranslator.h"

class anyMal : public ModelTranslator {
public:
    anyMal();

    bool TaskComplete(mjData *d, double &dist) override;
    void ReturnRandomStartState() override;
    void ReturnRandomGoalState() override;

    void Residuals(mjData *d, MatrixXd &residuals) override;

private:

};