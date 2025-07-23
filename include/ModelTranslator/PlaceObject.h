#pragma once

#include "StdInclude.h"
#include "MuJoCoHelper.h"
#include "ModelTranslator/ModelTranslator.h"

class PlaceObject: virtual public ModelTranslator{
public:

    PlaceObject(std::string EE_name, std::string body_name, int _clutter_level);

    void ReturnRandomStartState() override;
    void ReturnRandomGoalState() override;

    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;

    void Residuals(mjData *d, MatrixXd &residuals) override;

    bool TaskComplete(mjData *d, double &dist) override;

    void SetGoalVisuals(mjData *d) override;

protected:
    std::string EE_name;
    std::string body_name;
    int complete_counter = 0;
    int clutterLevel = lowClutter;
    double random_goal_x = 0.0;
    double random_goal_y = 0.0;
private:

};