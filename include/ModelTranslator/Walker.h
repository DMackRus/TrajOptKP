#pragma once

#include "ModelTranslator/ModelTranslator.h"

enum terrains{
    PLANE,
    UNEVEN
};

enum locomotion_types{
    WALK,
    RUN
};

class walker : public ModelTranslator {
public:
    walker(int terrain, int locomotion_type);

    bool TaskComplete(mjData *d, double &dist) override;
    void ReturnRandomStartState() override;
    void ReturnRandomGoalState() override;

    void Residuals(mjData *d, MatrixXd &residuals) override;

private:
    float low_bound_velocity;
    float high_bound_velocity;

};
