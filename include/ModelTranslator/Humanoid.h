#pragma once

#include "ModelTranslator/ModelTranslator.h"

class Humanoid: public ModelTranslator {
public:
    Humanoid();

    bool TaskComplete(mjData *d, double &dist) override;
    void Residuals(mjData *d, MatrixXd &residuals) override;
//    void ReturnRandomStartState() override;
//    void ReturnRandomGoalState() override;

};

