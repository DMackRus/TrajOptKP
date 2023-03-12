//
// Created by dave on 09/03/23.
//

#ifndef PHYSICSSIMSWITCHING_DOUBLEPENDULUM_H
#define PHYSICSSIMSWITCHING_DOUBLEPENDULUM_H

#include "modelTranslator.h"

class doublePendulum : public modelTranslator {
public:
    doublePendulum();
    ~doublePendulum(){

    }

    double costFunction(MatrixXd Xt, MatrixXd Ut, MatrixXd X_last, MatrixXd U_last) override;

    char* filePath; 
    int pendulumDOF;
    int pendulumNumCtrl;



};

#endif //PHYSICSSIMSWITCHING_DOUBLEPENDULUM_H
