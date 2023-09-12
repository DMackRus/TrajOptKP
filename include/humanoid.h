#pragma once

#include "modelTranslator.h"

class humanoid: public modelTranslator {
public:
    humanoid();

    bool taskComplete(int dataIndex, double &dist) override;
    void generateRandomGoalAndStartState() override;
    MatrixXd returnRandomStartState() override;
    MatrixXd returnRandomGoalState(MatrixXd X0) override;

};

