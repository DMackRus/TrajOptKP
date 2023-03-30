//
// Created by dave on 03/03/23.
//

#ifndef PHYSICSSIMSWITCHING_MODELTRANSLATOR_H
#define PHYSICSSIMSWITCHING_MODELTRANSLATOR_H

#include "stdInclude.h"
#include "MuJoCoHelper.h"
#include <yaml-cpp/yaml.h>

struct bodyStateVec{
    string name;
    bool activeLinearDOF[3];
    bool activeAngularDOF[3];
    double linearPosCost[3];
    double linearVelCost[3];
    double angularPosCost[3];
    double angularVelCost[3];
};

struct stateVectorList{
    vector<robot> robots;
    vector<bodyStateVec> bodiesStates;
};

class modelTranslator {
public:
    modelTranslator();

    // - Functions that work for all tasks in base class
    void loadRobotsandBodiesFromYAML(std::string yamlFilePath, vector<robot> &_robots, vector<bodyStateVec> &_bodies);
    void initModelTranslator(std::string filePath);

    MatrixXd returnStateVector(int dataIndex);
    bool setStateVector(MatrixXd _stateVector, int dataIndex);
    MatrixXd returnControlVector(int dataIndex);
    bool setControlVector(MatrixXd _controlVector, int dataIndex);

    MatrixXd returnPositionVector(int dataIndex);
    MatrixXd returnVelocityVector(int dataIndex);
    MatrixXd returnAccelerationVector(int dataIndex);
    bool setPositionVector(MatrixXd _positionVector, int dataIndex);
    bool setVelocityVector(MatrixXd _velocityVector, int dataIndex);

    // - Optional override functions, have default implementations but can be overwritten
    virtual double costFunction(MatrixXd Xt, MatrixXd Ut, MatrixXd X_last, MatrixXd U_last, bool terminal);
    virtual void costDerivatives(MatrixXd Xt, MatrixXd Ut, MatrixXd X_last, MatrixXd U_last, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal);
    virtual bool taskComplete(int dataIndex);

    // - Pure virtual functions that HAVE to be overwritten
    virtual std::vector<MatrixXd> createInitControls(int horizonLength);
    virtual MatrixXd returnRandomStartState() = 0;
    virtual MatrixXd returnRandomGoalState() = 0;

    int dof;
    int num_ctrl;
    int stateVectorSize;
    struct stateVectorList myStateVector;

    physicsSimulator *activePhysicsSimulator;
    MuJoCoHelper *myHelper;
    std::string modelFilePath;

protected:
    MatrixXd Q;
    MatrixXd Q_terminal;
    MatrixXd R;
    MatrixXd J;

    MatrixXd X_desired;
    bool analyticalCostDerivatives;

    char* filePath; 

private:

};

#endif //PHYSICSSIMSWITCHING_MODELTRANSLATOR_H
