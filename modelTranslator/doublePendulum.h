//
// Created by dave on 09/03/23.
//

#ifndef PHYSICSSIMSWITCHING_DOUBLEPENDULUM_H
#define PHYSICSSIMSWITCHING_DOUBLEPENDULUM_H

#include "modelTranslator.h"

class doublePendulum : public modelTranslator {
public:
    doublePendulum(int taskNumber);

    MatrixXd returnStateVector();
    bool setStateVector(MatrixXd _stateVector);

};

#endif //PHYSICSSIMSWITCHING_DOUBLEPENDULUM_H
