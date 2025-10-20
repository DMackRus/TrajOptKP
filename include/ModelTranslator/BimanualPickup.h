#pragma once

#include "ModelTranslator/ModelTranslator.h"

class BimanualPickup : public ModelTranslator {
public:
    BimanualPickup();

    bool TaskComplete(mjData *d, double &dist) override;
    void ReturnRandomStartState() override;
    void ReturnRandomGoalState() override;

    void Residuals(mjData *d, MatrixXd &residuals) override;

    void SetGoalVisuals(mjData *d) override;
private:

};