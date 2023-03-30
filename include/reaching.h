//
// Created by dave on 09/03/23.
//

#ifndef PHYSICSSIMSWITCHING_REACHING_H
#define PHYSICSSIMSWITCHING_REACHING_H

#include "modelTranslator.h"

class pandaReaching : public modelTranslator {
public:
    pandaReaching();

    bool taskComplete(int dataIndex) override;
    MatrixXd returnRandomStartState() override;
    MatrixXd returnRandomGoalState() override;
    std::vector<MatrixXd> createInitControls(int horizonLength) override;

    char* filePath; 

};

#endif //PHYSICSSIMSWITCHING_REACHING_H
