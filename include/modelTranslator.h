//
// Created by dave on 03/03/23.
//

#ifndef PHYSICSSIMSWITCHING_MODELTRANSLATOR_H
#define PHYSICSSIMSWITCHING_MODELTRANSLATOR_H

#include "stdInclude.h"
#include "MuJoCoHelper.h"
#include "fileHandler.h"

enum clutterLevels{
    noClutter = 0,
    lowClutter = 1,
    heavyClutter = 2,
    constrainedClutter = 3
};

class modelTranslator {
public:
    modelTranslator();

    // - Functions that work for all tasks in base class
    void initModelTranslator(std::string filePath);

    MatrixXd returnStateVector(std::shared_ptr<mjData> d);
    bool setStateVector(MatrixXd _stateVector, std::shared_ptr<mjData> d);
    MatrixXd returnControlVector(std::shared_ptr<mjData> d);
    bool setControlVector(MatrixXd _controlVector, std::shared_ptr<mjData> d);

    MatrixXd returnPositionVector(std::shared_ptr<mjData> d);
    MatrixXd returnVelocityVector(std::shared_ptr<mjData> d);
    MatrixXd returnAccelerationVector(std::shared_ptr<mjData> d);
    bool setPositionVector(MatrixXd _positionVector, std::shared_ptr<mjData> d);
    bool setVelocityVector(MatrixXd _velocityVector, std::shared_ptr<mjData> d);

    // - Optional override functions, have default implementations but can be overwritten
    virtual double costFunction(std::shared_ptr<mjData> d, bool terminal);
    virtual void costDerivatives(std::shared_ptr<mjData> d, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal);
    virtual bool taskComplete(std::shared_ptr<mjData> d, double &dist);
    virtual std::vector<MatrixXd> createInitSetupControls(int horizonLength);

    // - Pure virtual functions that HAVE to be overwritten
    virtual std::vector<MatrixXd> createInitOptimisationControls(int horizonLength);
    virtual MatrixXd returnRandomStartState() = 0;
    virtual MatrixXd returnRandomGoalState(MatrixXd X0) = 0;

    int dof;
    int num_ctrl;
    int stateVectorSize;
    struct stateVectorList myStateVector;
    MatrixXd X_desired;
    MatrixXd X_start;

    std::shared_ptr<MuJoCoHelper> mujocoHelper;
    std::string modelFilePath;
    std::string modelName;

protected:
    DiagonalMatrix<double, Eigen::Dynamic> Q;
    DiagonalMatrix<double, Eigen::Dynamic> Q_terminal;
    DiagonalMatrix<double, Eigen::Dynamic> R;
    DiagonalMatrix<double, Eigen::Dynamic> J;

private:

};

#endif //PHYSICSSIMSWITCHING_MODELTRANSLATOR_H
