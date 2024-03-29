#pragma once

#include "ModelTranslator.h"

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
    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;

    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;

private:
    double low_bound_velocity;
    double high_bound_velocity;

};
