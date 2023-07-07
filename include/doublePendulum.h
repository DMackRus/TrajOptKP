//
// Created by dave on 09/03/23.
//

#ifndef PHYSICSSIMSWITCHING_DOUBLEPENDULUM_H
#define PHYSICSSIMSWITCHING_DOUBLEPENDULUM_H

#include "modelTranslator.h"

class doublePendulum : public modelTranslator {
public:
    doublePendulum();

    bool taskComplete(int dataIndex, double &dist) override;
    MatrixXd returnRandomStartState() override;
    MatrixXd returnRandomGoalState(MatrixXd X0) override;

};

#endif //PHYSICSSIMSWITCHING_DOUBLEPENDULUM_H
