#pragma once
#include "ModelTranslator.h"

class Piston : public ModelTranslator {
public:
    Piston();
    void ReturnRandomStartState() override;
    void ReturnRandomGoalState() override;
    void Residuals(mjData *d, MatrixXd &residuals) override;
    void SetGoalVisuals(mjData *d) override;
};
