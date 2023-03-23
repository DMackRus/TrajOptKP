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

    //double costFunction(MatrixXd Xt, MatrixXd Ut, MatrixXd X_last, MatrixXd U_last);
    //void costDerivatives(MatrixXd Xt, MatrixXd Ut, MatrixXd X_last, MatrixXd U_last, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu);
    MatrixXd returnRandomStartState() override;
    MatrixXd returnRandomGoalState() override;

    
    char* filePath; 
};

#endif //PHYSICSSIMSWITCHING_DOUBLEPENDULUM_H
