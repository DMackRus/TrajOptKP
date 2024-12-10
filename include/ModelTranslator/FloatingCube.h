#pragma once

#include "ModelTranslator.h"

class FloatingCube: public ModelTranslator{
public:
    FloatingCube();
//
//    void ReturnRandomStartState() override;
//    void ReturnRandomGoalState() override;

    void SetGoalVisuals(mjData *d) override;
    void Residuals(mjData *d, MatrixXd &residuals) override;

    bool TaskComplete(mjData *d, double &dist) override;

private:

};