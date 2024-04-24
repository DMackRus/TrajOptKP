#pragma once

#include "ModelTranslator/ModelTranslator.h"

class Pentabot : public ModelTranslator{

public:
    Pentabot();

    bool TaskComplete(mjData *d, double &dist) override;

    void ReturnRandomStartState() override;
    void ReturnRandomGoalState() override;
};
